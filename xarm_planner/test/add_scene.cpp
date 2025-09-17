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

bool logging_active = false;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[probe_random] Ctrl-C caught, exit process...\n");
    exit(-1);
}

enum ShapeType { BOX, SPHERE, CYLINDER };

// Helper struct for object spec
struct ObjectSpec {
  std::string id;
  ShapeType shape;

  // Dimensions (meaning depends on shape):
  // BOX: [x,y,z], CYLINDER: [height, radius], SPHERE: [radius]
  std::vector<double> dims;

  // Pose
  double px, py, pz;
  double qx, qy, qz, qw;
};

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
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
    };

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    for (const auto& s : specs) {
        shape_msgs::msg::SolidPrimitive prim;

        switch (s.shape) {
        case BOX:
            prim.type = shape_msgs::msg::SolidPrimitive::BOX;
            prim.dimensions.resize(3);
            prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = s.dims[0];
            prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = s.dims[1];
            prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = s.dims[2];
            break;

        case CYLINDER:
            prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            prim.dimensions.resize(2);
            prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = s.dims[0];
            prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = s.dims[1];
            break;

        case SPHERE:
            prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            prim.dimensions.resize(1);
            prim.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = s.dims[0];
            break;
        }

        // Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = s.px;
        pose.position.y = s.py;
        pose.position.z = s.pz;
        pose.orientation.x = s.qx;
        pose.orientation.y = s.qy;
        pose.orientation.z = s.qz;
        pose.orientation.w = s.qw;

        // Collision object
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "link_base";
        obj.id = s.id;
        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        collision_objects.push_back(std::move(obj));
    }

    // Apply
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
    addCollisionObject(planning_scene_interface);

    // Sleep briefly to allow the collision object to propagate
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(node->get_logger(), "add_scene over");

    rclcpp::shutdown();
    return 0;
}
