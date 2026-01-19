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

#define PUBLISH_RATE 100.0

/**
 * \brief Generates a linear trajectory that moves the end-effector 0.1m in the z direction
 * \param start_pose The starting pose of the end-effector
 * \return Vector of waypoint positions (x, y, z) with z increasing by 0.1m
 */
std::vector<Eigen::Vector3d> getPath(const geometry_msgs::msg::PoseStamped& start_pose)
{
  // Hardcoded trajectory parameters
  const double duration = 2.0;  // Duration in seconds
  const double z_displacement = -0.1;  // Move 0.1m in z direction
  
  // Extract starting position
  const double start_x = start_pose.pose.position.x;
  const double start_y = start_pose.pose.position.y;
  const double start_z = start_pose.pose.position.z;
  
  // Calculate target position
  const double target_z = start_z + z_displacement;
  
  // Calculate number of waypoints based on duration and publish rate
  const int num_waypoints = static_cast<int>(duration * PUBLISH_RATE);
  const double step_size = 1.0 / num_waypoints;  // Normalized step (0.0 to 1.0)
  
  std::vector<Eigen::Vector3d> traj;
  traj.reserve(num_waypoints);
  
  // Generate waypoints with linear interpolation in z direction
  for (int i = 0; i <= num_waypoints; ++i)
  {
    const double t = i * step_size;  // Interpolation parameter [0.0, 1.0]
    const double z = start_z + t * z_displacement;  // Linear interpolation in z
    
    // x and y remain constant, only z changes
    auto vec = Eigen::Vector3d(start_x, start_y, z);
    traj.push_back(vec);
  }
  
  return traj;
}

/**
 * \brief Generates a PoseStamped message with the given position and orientation.
 * \param position The position of the end-effector
 * \param rotation The orientation of the end-effector
 * \return geometry_msgs::PoseStamped with position in meters and orientation as quaternion
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


class TrackPoseNode : public rclcpp::Node
{
  public:

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_publisher;

    TrackPoseNode()
    : Node("track_pose_node")
    {
      robot_state_subscriber = this->create_subscription<xarm_msgs::msg::RobotMsg>(
      "/ufactory/robot_states", rclcpp::SystemDefaultsQoS(), std::bind(&TrackPoseNode::robot_state_callback, this, std::placeholders::_1));

      switch_input_client = this->create_client<moveit_msgs::srv::ServoCommandType>("/servo_server/switch_command_type");

      desired_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/servo_server/pose_cmds", rclcpp::SystemDefaultsQoS());
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

  // Create an executor and spin the node in a separate thread
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  
  std::thread executor_thread([&executor]() {
    executor->spin();
  });

  // Wait a bit for the subscriber to receive the first pose
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Wait for valid pose data (check a few times)
  geometry_msgs::msg::PoseStamped start_pose;
  int attempts = 0;
  const int max_attempts = 20;
  while (attempts < max_attempts)
  {
    start_pose = node->get_eef_pose();
    // Check if we have valid data (non-zero or reasonable values)
    if (start_pose.pose.position.z > 0.1)  // Adjust threshold as needed
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    attempts++;
  }

  if (attempts >= max_attempts)
  {
    RCLCPP_WARN(node->get_logger(), "Timeout waiting for valid pose data");
  }
  else
  {
    // Get path and print the waypoints
    RCLCPP_INFO(node->get_logger(), "Start pose: %f, %f, %f", 
                start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    auto path = getPath(start_pose);
    RCLCPP_INFO(node->get_logger(), "Generated %zu waypoints", path.size());
    
    for (size_t i = 0; i < path.size(); ++i)
    {
      RCLCPP_INFO(node->get_logger(), "Waypoint %zu: %f, %f, %f", 
                  i, path[i][0], path[i][1], path[i][2]);
    }

    // Follow the trajectory while the node continues spinning
    const double publish_period = 1.0 / PUBLISH_RATE;
    rclcpp::WallRate rate(1.0 / publish_period);
    
    // Create quaternion from start pose orientation
    Eigen::Quaterniond start_orientation(
      start_pose.pose.orientation.w,
      start_pose.pose.orientation.x,
      start_pose.pose.orientation.y,
      start_pose.pose.orientation.z
    );
    
    for (auto& waypoint : path)
    {
      auto target_pose = getPose(waypoint, start_orientation);
      target_pose.header.stamp = node->now();
      target_pose.header.frame_id = start_pose.header.frame_id;  // Use same frame_id as start
      node->desired_pose_publisher->publish(target_pose);
      rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Trajectory execution complete");
  }

  // Stop the executor and wait for thread to finish
  executor->cancel();
  if (executor_thread.joinable())
  {
    executor_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
