/* Copyright 2024 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: OpenAI Assistant, modified for YAML-based scene loading
 ============================================================================*/

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <string>
#include <vector>

void exit_sig_handler(int signum __attribute__((unused)))
{
    fprintf(stderr, "[xarm_servo_scene_pilotus] Ctrl-C caught, exit process...\n");
    exit(-1);
}

static moveit_msgs::msg::CollisionObject makeCollisionObject(
    const std::string& frame_id,
    const std::string& id,
    const std::string& shape_str,
    const std::vector<double>& dims,
    const std::vector<double>& pos,
    const std::vector<double>& ori)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = id;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive prim;
    if (shape_str == "box") {
        prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        prim.dimensions = {dims[0], dims[1], dims[2]};
    } else if (shape_str == "sphere") {
        prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        prim.dimensions = {dims[0]};
    } else if (shape_str == "cylinder") {
        prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        prim.dimensions = {dims[0], dims[1]};
    } else {
        throw std::runtime_error("Unknown shape type: " + shape_str);
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];

    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    return obj;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("add_servo_scene", node_options);

    RCLCPP_INFO(node->get_logger(), "xarm_servo_scene_pilotus start");
    signal(SIGINT, exit_sig_handler);

    std::string scene_spec_file;
    node->get_parameter_or("scene_spec_file", scene_spec_file, std::string(""));

    if (scene_spec_file.empty()) {
        RCLCPP_INFO(node->get_logger(), "No scene_spec_file provided, skipping planning scene load.");
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Loading planning scene from: %s", scene_spec_file.c_str());

    YAML::Node config;
    try {
        config = YAML::LoadFile(scene_spec_file);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load scene spec file '%s': %s",
                     scene_spec_file.c_str(), e.what());
        rclcpp::shutdown();
        return 1;
    }

    std::string planning_frame = "link_base";
    if (config["planning_frame"]) {
        planning_frame = config["planning_frame"].as<std::string>();
    }

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    if (config["objects"]) {
        for (const auto& obj_node : config["objects"]) {
            std::string id = obj_node["id"].as<std::string>();
            std::string shape = obj_node["shape"].as<std::string>();
            auto dims = obj_node["dimensions"].as<std::vector<double>>();
            auto pos  = obj_node["position"].as<std::vector<double>>();
            auto ori  = obj_node["orientation"].as<std::vector<double>>();

            try {
                collision_objects.push_back(
                    makeCollisionObject(planning_frame, id, shape, dims, pos, ori));
                RCLCPP_INFO(node->get_logger(), "  Added object: %s (%s)", id.c_str(), shape.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(node->get_logger(), "  Skipping object '%s': %s", id.c_str(), e.what());
            }
        }
    }

    if (collision_objects.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid objects found in scene spec, nothing to publish.");
        rclcpp::shutdown();
        return 0;
    }

    auto planning_scene_diff_publisher =
        node->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(1).transient_local());

    // Allow publisher to connect to subscribers
    rclcpp::sleep_for(std::chrono::seconds(1));

    std::size_t num_objects = collision_objects.size();
    auto scene_msg = std::make_unique<moveit_msgs::msg::PlanningScene>();
    scene_msg->world.collision_objects = std::move(collision_objects);
    scene_msg->is_diff = true;
    planning_scene_diff_publisher->publish(std::move(scene_msg));

    RCLCPP_INFO(node->get_logger(), "Published planning scene diff (%zu objects). Node will now exit.",
                num_objects);

    rclcpp::shutdown();
    return 0;
}
