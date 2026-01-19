#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

#include <xarm_msgs/msg/robot_msg.hpp>

#include <array>
#include <mutex>

/**
 * \brief Generates the path to follow
 */
std::vector<Eigen::Vector3d> getPath()
{
  const double start_angle = M_PI / 2 + (M_PI / 8);
  const double end_angle = M_PI;
  const double step = 0.01745329;
  std::vector<Eigen::Vector3d> traj;

  for (double i = start_angle; i < end_angle; i = i + step)
  {
    double x = 0.8 + (0.5 * cos(i));
    double y = 0.0 + (0.5 * sin(i));
    auto vec = Eigen::Vector3d(x, y, 0.4);
    traj.push_back(vec);
  }
  return traj;
}

/**
 * \brief Creates an Rviz marker message to represent a waypoint in the path.
 */
visualization_msgs::msg::Marker getMarker(int id, const Eigen::Vector3d& position, const std::string& frame)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = rclcpp::Time(0.0);
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  id++;
  return marker;
}

/**
 * \brief Generates a PoseStamped message with the given position and orientation.
 */
geometry_msgs::msg::PoseStamped getPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation)
{
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "panda_link0";
  target_pose.pose.orientation.w = rotation.w();
  target_pose.pose.orientation.x = rotation.x();
  target_pose.pose.orientation.y = rotation.y();
  target_pose.pose.orientation.z = rotation.z();
  target_pose.pose.position.x = position.x();
  target_pose.pose.position.y = position.y();
  target_pose.pose.position.z = position.z();

  return target_pose;
}

//void robotStateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
//{
//  RCLCPP_INFO(rclcpp::get_logger("robot_state"), "Received robot state");
//}

using std::placeholders::_1;

class TrackPoseNode : public rclcpp::Node
{
  public:
    TrackPoseNode()
    : Node("track_pose_node")
    {
      robot_state_subscriber = this->create_subscription<xarm_msgs::msg::RobotMsg>(
      "/ufactory/robot_states", rclcpp::SystemDefaultsQoS(), std::bind(&TrackPoseNode::robot_state_callback, this, _1));

      switch_input_client = this->create_client<moveit_msgs::srv::ServoCommandType>("/servo_server/switch_command_type");
    }

    /**
     * \brief Get the end-effector pose from the stored robot state.
     * \return geometry_msgs::PoseStamped with position in meters and orientation as quaternion
     */
    geometry_msgs::msg::PoseStamped get_eef_pose()
    {
      std::lock_guard<std::mutex> lock(_pose_mutex);
      
      geometry_msgs::msg::PoseStamped eef_pose;
      eef_pose.header.frame_id = "link_base";  // Adjust frame_id as needed
      eef_pose.header.stamp = this->now();
      
      // Convert position from mm to meters
      eef_pose.pose.position.x = raw_pose[0] / 1000.0;
      eef_pose.pose.position.y = raw_pose[1] / 1000.0;
      eef_pose.pose.position.z = raw_pose[2] / 1000.0;
      
      // Convert RPY (roll, pitch, yaw) to quaternion
      Eigen::AngleAxisd roll_angle(raw_pose[3], Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitch_angle(raw_pose[4], Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yaw_angle(raw_pose[5], Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
      
      eef_pose.pose.orientation.x = q.x();
      eef_pose.pose.orientation.y = q.y();
      eef_pose.pose.orientation.z = q.z();
      eef_pose.pose.orientation.w = q.w();
      
      return eef_pose;
    }

    /**
     * \brief Switch to POSE input type
     * \return true if successful, false otherwise
     */
    bool switch_to_pose_command()
    {
      auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
      request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
      
      if (switch_input_client->wait_for_service(std::chrono::seconds(5)))
      {
        auto result = switch_input_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
          if (result.get()->success)
          {
            RCLCPP_INFO_STREAM(this->get_logger(), "Switched to input type: POSE");
            return true;
          }
          else
          {
            RCLCPP_WARN_STREAM(this->get_logger(), "Could not switch input to: POSE");
            return false;
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Service call failed");
          return false;
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service /servo_server/switch_command_type");
        return false;
      }
    }


  private:
  
    // Callbacks
    void robot_state_callback(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
    {
      {
        std::lock_guard<std::mutex> lock(_pose_mutex);
        // Store the raw pose: [x, y, z, roll, pitch, yaw]
        for (size_t i = 0; i < 6; ++i) {
          raw_pose[i] = msg->pose[i];
        }
      }

      // Print the eef pose
      //auto eef_pose = get_eef_pose();
      //RCLCPP_INFO(
      //  this->get_logger(), "I heard robot state: %f, %f, %f, %f, %f, %f, %f", 
      //  eef_pose.pose.position.x, eef_pose.pose.position.y, eef_pose.pose.position.z, 
      //  eef_pose.pose.orientation.x, eef_pose.pose.orientation.y, eef_pose.pose.orientation.z,
      //  eef_pose.pose.orientation.w
      //);
      //RCLCPP_INFO(this->get_logger(), "I heard robot state: %f, %f, %f, %f, %f, %f", raw_pose[0], raw_pose[1], raw_pose[2], raw_pose[3], raw_pose[4], raw_pose[5]);
    }

    // Subscribers
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_subscriber;

    // Service clients
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_client;

    std::array<float, 6> raw_pose;  // [x, y, z, roll, pitch, yaw] - position in mm, orientation in rad
    mutable std::mutex _pose_mutex;  // Protects raw_pose_ for thread-safe access

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackPoseNode>();

  node->switch_to_pose_command();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

//int main(int argc, char* argv[])
//{
//  rclcpp::init(argc, argv);
//  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("servo_tutorial");
//
//  // Publishers
//  auto marker_publisher =
//      node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", rclcpp::SystemDefaultsQoS());
//  auto pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/servo_server/pose_cmds",
//                                                                                rclcpp::SystemDefaultsQoS());
//  // Subscribers
//  auto robot_state_subscriber = node->create_subscription<xarm_msgs::msg::RobotMsg>("/ufactory/robot_states", rclcpp::SystemDefaultsQoS(), std::bind(&robotStateCallback, std::placeholders::_1));
//
//  // Service clients
//  auto switch_input_client = node->create_client<moveit_msgs::srv::ServoCommandType>("/servo_server/switch_command_type");
//
//  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//  executor->add_node(node);
//
//  // Spin the node.
//  std::thread executor_thread([&executor]() { executor->spin(); });
//
//  // Generate path
//  std::vector<Eigen::Vector3d> path = getPath();
//
//  // Switch to POSE input type
//  switch_input_client = node->create_client<moveit_msgs::srv::ServoCommandType>("/servo_server/switch_command_type");
//  auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
//  request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
//  if (switch_input_client->wait_for_service(std::chrono::seconds(5)))
//  {
//    auto result = switch_input_client->async_send_request(request);
//    if (result.get()->success)
//    {
//      RCLCPP_INFO_STREAM(node->get_logger(), "Switched to input type: POSE");
//    }
//    else
//    {
//      RCLCPP_WARN_STREAM(node->get_logger(), "Could not switch input to: POSE");
//    }
//  }
//  else
//  {
//    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to call service /servo_server/switch_command_type");
//  }
//
//  //// Follow the trajectory
//  //const double publish_period = 0.15;
//  //rclcpp::WallRate rate(1.0 / publish_period);
//  //for (auto& waypoint : path)
//  //{
//  //  auto target_pose = getPose(waypoint, Eigen::Quaterniond(ee_pose.rotation()));
//  //  target_pose.header.stamp = node->now();
//  //  pose_publisher->publish(target_pose);
//  //  rate.sleep();
//  //}
//
//  executor->cancel();
//  if (executor_thread.joinable())
//  {
//    executor_thread.join();
//  }
//  rclcpp::shutdown();
//}