/* Copyright 2024 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: OpenAI Assistant, modified by Google Gemini for diff publishing
 ============================================================================*/

#include <rclcpp/rclcpp.hpp>
// ADDED: The message for publishing scene updates
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <utility>
#include <string>
#include <vector>

#define Z_AXIS_RAISE -0.016 // in m

// --- (Helper structs and signal handler remain the same) ---
void exit_sig_handler(int signum)
{
    fprintf(stderr, "[probe_random] Ctrl-C caught, exit process...\n");
    exit(-1);
}

enum ShapeType { BOX, SPHERE, CYLINDER };

struct ObjectSpec {
  std::string id;
  ShapeType shape;
  std::vector<double> dims;
  double px, py, pz;
  double qx, qy, qz, qw;
};

// MODIFIED: This function now takes a publisher instead of the PlanningSceneInterface
void addCollisionObject(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& scene_pub,
    const rclcpp::Node::SharedPtr& node)
{
    // ---- Define all objects here (meters) ----
      std::vector<ObjectSpec> specs = {
        // Table
        { "table1", BOX, {0.296, 0.633, 0.1},
          0.296/2.0, (0.633 + 0.002)/2.0, -0.05 + Z_AXIS_RAISE,
          0, 0, 0, 1 },
        { "table2", BOX, {0.296, 0.587 + 0.002, 0.1},
          0.296/2.0, -(0.587 + 0.002)/2.0, -0.05 + Z_AXIS_RAISE,
          0, 0, 0, 1 },
        { "table3", BOX, {2.144, 0.633 + 0.002, 0.1},
          -2.144/2.0, (0.633 + 0.002)/2.0, -0.05 + Z_AXIS_RAISE,
          0, 0, 0, 1 },
        { "table4", BOX, {2.144, 0.587 + 0.002, 0.1},
          -2.144/2.0, -(0.587 + 0.002)/2.0, -0.05 + Z_AXIS_RAISE,
          0, 0, 0, 1 },

        // Walls
        { "wall1", BOX, {0.150, 3.5, 1.0},
          1.05 + 0.150/2.0, 0.0, 0.5 + Z_AXIS_RAISE,
          0, 0, 0, 1 },
        { "wall2", BOX, {2.5, 0.150, 1.0},
          0.0,  -1.720 - 0.150/2.0, 0.5 + Z_AXIS_RAISE,
          0, 0, 0, 1 },
        { "wall3", BOX, {0.150, 2.5, 1.0},
          -0.7-0.150/2.0,  0.0, 0.5 + Z_AXIS_RAISE,
          0, 0, 0, 1 },
    };

    // --- (The logic to build the vector of CollisionObjects is identical) ---
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    for (const auto& s : specs) {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "link_base";
        obj.id = s.id;
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive prim;
        switch (s.shape) {
        case BOX:
            prim.type = shape_msgs::msg::SolidPrimitive::BOX;
            prim.dimensions = {s.dims[0], s.dims[1], s.dims[2]};
            break;
        case CYLINDER:
            prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            prim.dimensions = {s.dims[0], s.dims[1]};
            break;
        case SPHERE:
            prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            prim.dimensions = {s.dims[0]};
            break;
        }

        geometry_msgs::msg::Pose pose;
        pose.position.x = s.px;
        pose.position.y = s.py;
        pose.position.z = s.pz;
        pose.orientation.x = s.qx;
        pose.orientation.y = s.qy;
        pose.orientation.z = s.qz;
        pose.orientation.w = s.qw;

        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        collision_objects.push_back(std::move(obj));
    }

    // --- REPLACEMENT LOGIC ---
    // 1. Create a PlanningScene message
    auto scene_msg = std::make_unique<moveit_msgs::msg::PlanningScene>();

    // 2. Add the collision objects to this message
    scene_msg->world.collision_objects = std::move(collision_objects);

    // 3. Set the `is_diff` flag to true. This tells the receiver to only
    // apply the changes contained in the message.
    scene_msg->is_diff = true;

    // 4. Publish the message
    scene_pub->publish(std::move(scene_msg));
    RCLCPP_INFO(node->get_logger(), "Published planning scene diff to add collision objects.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("add_servo_scene", node_options);

    RCLCPP_INFO(node->get_logger(), "add_servo_scene start");
    signal(SIGINT, exit_sig_handler);

    // --- (Parameter handling logic is unchanged) ---
    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("uf850"));
    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));
    if (prefix != "") {
        group_name = prefix + group_name;
    }
    RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    // --- MODIFIED: Create a publisher instead of the PlanningSceneInterface ---
    auto planning_scene_diff_publisher =
        node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::QoS(1).transient_local());

    // Allow publisher to connect to subscribers
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Call the function with the new publisher
    addCollisionObject(planning_scene_diff_publisher, node);

    RCLCPP_INFO(node->get_logger(), "Scene updated. The node will now exit.");

    rclcpp::shutdown();
    return 0;
}
