#include "xarm_planner/xarm_planner.h"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometric_shapes/shape_operations.h>
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <mutex>

#define Z_AXIS_RAISE -0.016 // in m
#define TRAIL_WINDOW 0.3 // in seconds
// #define DISTAL_LOC_X 0.235 // in m
// #define DISTAL_LOC_Y 0.449
#define DISTAL_LOC_X 0.236 // in m
#define DISTAL_LOC_Y 0.4523
#define CALI_BOX_LOC_X 0.236
#define CALI_BOX_LOC_Y 0.4643

#define DISTAL_CENTER_SCALE 0.3

bool logging_active = false;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[probe_random] Ctrl-C caught, exit process...\n");
    exit(-1);
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "link_base";
    collision_object.id = "table";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.0;
    primitive.dimensions[primitive.BOX_Y] = 1220.0/1000 + 0.002;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.05 + Z_AXIS_RAISE;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObjects({collision_object});
}


void addCollisionObject_Mesh(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // 1. Add box
    // 29.6cm to end of the table
    // 244cm for whole table
    moveit_msgs::msg::CollisionObject table1;
    table1.header.frame_id = "link_base";
    table1.id = "table1";

    shape_msgs::msg::SolidPrimitive table_primitive1;
    table_primitive1.type = table_primitive1.BOX;
    table_primitive1.dimensions.resize(3);
    table_primitive1.dimensions[table_primitive1.BOX_X] = 312.0/1000.0;
    table_primitive1.dimensions[table_primitive1.BOX_Y] = 714.0/1000.0;
    table_primitive1.dimensions[table_primitive1.BOX_Z] = 0.110;

    geometry_msgs::msg::Pose table_pose1;
    table_pose1.orientation.w = 1.0;
    table_pose1.position.x = table_primitive1.dimensions[table_primitive1.BOX_X]/2.0;
    table_pose1.position.y = table_primitive1.dimensions[table_primitive1.BOX_Y]/2.0;
    table_pose1.position.z = -table_primitive1.dimensions[table_primitive1.BOX_Z]/2.0;

    table1.primitives.push_back(table_primitive1);
    table1.primitive_poses.push_back(table_pose1);
    table1.operation = table1.ADD;

    collision_objects.push_back(table1);

    moveit_msgs::msg::CollisionObject table2;
    table2.header.frame_id = "link_base";
    table2.id = "table2";

    shape_msgs::msg::SolidPrimitive table_primitive2;
    table_primitive2.type = table_primitive2.BOX;
    table_primitive2.dimensions.resize(3);
    table_primitive2.dimensions[table_primitive2.BOX_X] = 312.0/1000.0;
    table_primitive2.dimensions[table_primitive2.BOX_Y] = 787.0/1000.0;
    table_primitive2.dimensions[table_primitive2.BOX_Z] = 0.110;

    geometry_msgs::msg::Pose table_pose2;
    table_pose2.orientation.w = 1.0;
    table_pose2.position.x = table_primitive2.dimensions[table_primitive2.BOX_X]/2.0;
    table_pose2.position.y = -table_primitive2.dimensions[table_primitive2.BOX_Y]/2.0;
    table_pose2.position.z = -table_primitive2.dimensions[table_primitive2.BOX_Z]/2.0;

    table2.primitives.push_back(table_primitive2);
    table2.primitive_poses.push_back(table_pose2);
    table2.operation = table2.ADD;

    collision_objects.push_back(table2);

    moveit_msgs::msg::CollisionObject table3;
    table3.header.frame_id = "link_base";
    table3.id = "table3";

    shape_msgs::msg::SolidPrimitive table_primitive3;
    table_primitive3.type = table_primitive3.BOX;
    table_primitive3.dimensions.resize(3);
    table_primitive3.dimensions[table_primitive3.BOX_X] = 589.0/1000.0;
    table_primitive3.dimensions[table_primitive3.BOX_Y] = 787.0/1000.0;
    table_primitive3.dimensions[table_primitive3.BOX_Z] = 0.110;

    geometry_msgs::msg::Pose table_pose3;
    table_pose3.orientation.w = 1.0;
    table_pose3.position.x = -table_primitive3.dimensions[table_primitive3.BOX_X]/2.0;
    table_pose3.position.y = -table_primitive3.dimensions[table_primitive3.BOX_Y]/2.0;
    table_pose3.position.z = -table_primitive3.dimensions[table_primitive3.BOX_Z]/2.0;

    table3.primitives.push_back(table_primitive3);
    table3.primitive_poses.push_back(table_pose3);
    table3.operation = table3.ADD;

    collision_objects.push_back(table3);

    moveit_msgs::msg::CollisionObject table4;
    table4.header.frame_id = "link_base";
    table4.id = "table4";

    shape_msgs::msg::SolidPrimitive table_primitive4;
    table_primitive4.type = table_primitive4.BOX;
    table_primitive4.dimensions.resize(3);
    table_primitive4.dimensions[table_primitive4.BOX_X] = 589.0/1000.0;
    table_primitive4.dimensions[table_primitive4.BOX_Y] = 714.0/1000.0;
    table_primitive4.dimensions[table_primitive4.BOX_Z] = 0.110;

    geometry_msgs::msg::Pose table_pose4;
    table_pose4.orientation.w = 1.0;
    table_pose4.position.x = -table_primitive4.dimensions[table_primitive4.BOX_X]/2.0;
    table_pose4.position.y = table_primitive4.dimensions[table_primitive4.BOX_Y]/2.0;
    table_pose4.position.z = -table_primitive4.dimensions[table_primitive4.BOX_Z]/2.0;

    table4.primitives.push_back(table_primitive4);
    table4.primitive_poses.push_back(table_pose4);
    table4.operation = table4.ADD;

    collision_objects.push_back(table4);

    moveit_msgs::msg::CollisionObject wall1;
    wall1.header.frame_id = "link_base";
    wall1.id = "wall1";

    shape_msgs::msg::SolidPrimitive wall_primitive1;
    wall_primitive1.type = wall_primitive1.BOX;
    wall_primitive1.dimensions.resize(3);
    wall_primitive1.dimensions[wall_primitive1.BOX_X] = 760.0/1000.0;
    wall_primitive1.dimensions[wall_primitive1.BOX_Y] = 250.0/1000.0 + 787.0/1000.0;
    wall_primitive1.dimensions[wall_primitive1.BOX_Z] = 1.0;

    geometry_msgs::msg::Pose wall_pose1;
    wall_pose1.orientation.w = 1.0;
    wall_pose1.position.x = -589.0/1000.0 - wall_primitive1.dimensions[wall_primitive1.BOX_X]/2.0;
    wall_pose1.position.y = -wall_primitive1.dimensions[wall_primitive1.BOX_Y]/2.0;
    wall_pose1.position.z = wall_primitive1.dimensions[wall_primitive1.BOX_Z]/2.0;

    wall1.primitives.push_back(wall_primitive1);
    wall1.primitive_poses.push_back(wall_pose1);
    wall1.operation = wall1.ADD;

    collision_objects.push_back(wall1);

    moveit_msgs::msg::CollisionObject wall2;
    wall2.header.frame_id = "link_base";
    wall2.id = "wall2";

    shape_msgs::msg::SolidPrimitive wall_primitive2;
    wall_primitive2.type = wall_primitive2.BOX;
    wall_primitive2.dimensions.resize(3);
    wall_primitive2.dimensions[wall_primitive2.BOX_X] = 760.0/1000.0;
    wall_primitive2.dimensions[wall_primitive2.BOX_Y] = 1500.0/1000.0 + 714.0/1000.0;
    wall_primitive2.dimensions[wall_primitive2.BOX_Z] = 1.0;

    geometry_msgs::msg::Pose wall_pose2;
    wall_pose2.orientation.w = 1.0;
    wall_pose2.position.x = -589.0/1000.0 - wall_primitive2.dimensions[wall_primitive2.BOX_X]/2.0;
    wall_pose2.position.y = wall_primitive2.dimensions[wall_primitive2.BOX_Y]/2.0;
    wall_pose2.position.z = wall_primitive2.dimensions[wall_primitive2.BOX_Z]/2.0;

    wall2.primitives.push_back(wall_primitive2);
    wall2.primitive_poses.push_back(wall_pose2);
    wall2.operation = wall2.ADD;

    collision_objects.push_back(wall2);


    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_scene", node_options);
    // last_stop_time = node->get_clock()->now();
    RCLCPP_INFO(node->get_logger(), "add_scene start");

    signal(SIGINT, exit_sig_handler);

    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));
    if (prefix != "") {
        group_name = prefix + group_name;
    }

    RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(node->get_logger(), "Model Frame %s", kinematic_model->getModelFrame().c_str());

    xarm_planner::XArmPlanner planner(node, group_name);

    // Create planning scene interface and add collision object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // addCollisionObject(planning_scene_interface);
    addCollisionObject_Mesh(planning_scene_interface);

    // Sleep briefly to allow the collision object to propagate
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(node->get_logger(), "add_scene over");

    rclcpp::shutdown();
    return 0;
}
