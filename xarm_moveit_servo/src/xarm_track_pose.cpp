#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

#include <xarm_msgs/msg/robot_msg.hpp>

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

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
      "/ufactory/robot_states", rclcpp::SystemDefaultsQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const xarm_msgs::msg::RobotMsg::SharedPtr msg) const
    // Print 6 float values in msg->pose
    {
      RCLCPP_INFO(this->get_logger(), "I heard robot state: %f, %f, %f, %f, %f, %f", msg->pose[0], msg->pose[1], msg->pose[2], msg->pose[3], msg->pose[4], msg->pose[5]);
    }
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
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