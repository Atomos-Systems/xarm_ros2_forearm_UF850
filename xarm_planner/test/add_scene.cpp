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

// #define DISTAL_LOC_X 0.235 // in m
// #define DISTAL_LOC_Y 0.449
#define DISTAL_LOC_X 0.236 // in m
#define DISTAL_LOC_Y (-0.512)
#define CALIBRATION_X 0.002
#define CALIBRATION_Y -0.002
#define CALIBRATION_Z -0.010
#define DISTAL_LOC_CAL_X (DISTAL_LOC_X + CALIBRATION_X)
#define DISTAL_LOC_CAL_Y (DISTAL_LOC_Y + CALIBRATION_Y)

#define DISTAL_CENTER_SCALE 0.3
#define BELOW_DIST_X_DIM 0.0175
#define BELOW_DIST_Y_DIM 0.0185
#define BELOW_DIST_Z_DIM 0.01

inline void mirror_about_distal_anchor(double& x, double& y) {
  x = 2.0 * DISTAL_LOC_CAL_X - x;
  y = 2.0 * DISTAL_LOC_CAL_Y - y;
}

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

void addCollisionObject_Mesh(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

    // ---- Define all objects here (meters) ----
      std::vector<ObjectSpec> specs = {
        // Table
        { "table1", BOX, {0.312 + 0.01, 0.714 + 0.01, 0.110},
            (0.312 + 0.01)/2.0, (0.714 + 0.01)/2.0, -(0.110)/2.0,
            0, 0, 0, 1 },
        { "table2", BOX, {0.312 + 0.01, 0.787, 0.110},
            (0.312 + 0.01)/2.0, -0.787/2.0, -(0.110)/2.0,
            0, 0, 0, 1 },
        { "table3", BOX, {0.589, 0.787, 0.110},
            -0.589/2.0, -0.787/2.0, -(0.110)/2.0,
            0, 0, 0, 1 },
        { "table4", BOX, {0.589, 0.714 + 0.01, 0.110},
            -0.589/2.0, (0.714 + 0.01)/2.0, -(0.110)/2.0,
            0, 0, 0, 1 },

        // Walls
        { "wall1", BOX, {0.760, 0.250 + 0.787, 1.0},
            -0.589 - 0.760/2.0, -(0.250 + 0.787)/2.0, (1.0)/2.0,
            0, 0, 0, 1 },
        { "wall2", BOX, {0.760, 1.5 + 0.714, 1.0},
            -0.589 - 0.760/2.0,  (1.5 + 0.714)/2.0, (1.0)/2.0,
            0, 0, 0, 1 },

        // Control Box
        { "ctrl_Box", BOX, {0.345, 0.135, 0.140},
            -0.589+(0.345/2.0), 0.714-(0.135/2.0), 0.140/2.0,
            0,0,0,1.0},

        // Table 2 Person Work Area
        { "person_area1", BOX, {0.312, 1.5, 1.0},
            0.312/2.0, 0.714+1.5/2.0, 0.5,
            0,0,0,1.0},
        { "person_area2", BOX, {0.589, 1.5, 1.0},
            -0.589/2.0, 0.714+1.5/2.0, 0.5,
            0,0,0,1.0},

        // Distal Risers
        { "dRiser1", BOX, {0.040, 0.040, 0.06 + 0.004},
            DISTAL_LOC_CAL_X, DISTAL_LOC_CAL_Y, (0.06 + 0.004)/2,
            0,0,0,1.0},

        // Below Distal
        { "below_dist", BOX, {BELOW_DIST_X_DIM, BELOW_DIST_Y_DIM, BELOW_DIST_Z_DIM},
            DISTAL_LOC_CAL_X, DISTAL_LOC_CAL_Y + 0.006, (0.004 + 0.06) + BELOW_DIST_Z_DIM/2.0,
            0,0,0,1.0},

        // Cylinder (your screw example)
        { "screw1", CYLINDER, {0.006, 0.005},
            /*px*/ DISTAL_LOC_CAL_X - 0.02 + 0.005,
            /*py*/ DISTAL_LOC_CAL_Y + 0.02 - 0.005,
            /*pz*/ (0.06 + 0.004) + 0.006/2,
            0, 0, 0, 1 },
        { "screw2", CYLINDER, {0.006, 0.005},
            /*px*/ DISTAL_LOC_CAL_X + 0.02 - 0.005,
            /*py*/ DISTAL_LOC_CAL_Y + 0.02 - 0.005,
            /*pz*/ (0.06 + 0.004) + 0.006/2,
            0, 0, 0, 1 },
        { "screw3", CYLINDER, {0.006, 0.005},
            /*px*/ DISTAL_LOC_CAL_X - 0.02 + 0.005,
            /*py*/ DISTAL_LOC_CAL_Y - 0.02 + 0.005,
            /*pz*/ (0.06 + 0.004) + 0.006/2,
            0, 0, 0, 1 },
        { "screw4", CYLINDER, {0.01, 0.005},
            /*px*/ DISTAL_LOC_CAL_X + 0.02 - 0.005,
            /*py*/ DISTAL_LOC_CAL_Y - 0.02 + 0.005,
            /*pz*/ (0.06 + 0.004) + 0.01/2,
            0, 0, 0, 1 },

        // FX3
        // { "FX3", BOX, {0.200, 0.130, 0.06},
        //     DISTAL_LOC_CAL_X - 0.200/2.0 + 0.075, DISTAL_LOC_CAL_Y - 0.130/2.0 - 0.080, 0.06/2.0,
        //     0,0,0,1.0},
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
        pose.position.z = s.pz + (CALIBRATION_Z);
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

    struct DistalSpec {
        const char* id;
        double sx, sy, sz;   // BOX dims (x,y,z)
    };

    /* ______ DISTAL OBJECT______ */

    // Shared Z plane (same expression you used, without b_dist_prim dependency).
    const double z_plane = (0.004 + 0.06) + BELOW_DIST_Z_DIM;

    // Starting Y anchor (same as your original for distal1)
    const double y_anchor =
        DISTAL_LOC_CAL_Y - BELOW_DIST_Y_DIM;

    // =========================
    // 1) Base 10 slices (stacked)
    // =========================
    struct DistalBaseNominal {
        const char* id;
        double x_nom;  // UN-SCALED nominal X for this base (your constants 0.00857, 0.0107, etc.)
        double y_thk;  // Y thickness
        double z_h;    // Z height
    };

    const DistalBaseNominal base_nom[] = {
    { "distal1",  0.00857,   0.00036, 0.00591 },
    { "distal2",  0.00857,   0.00076, 0.00723 },
    { "distal3",  0.00857,   0.00088, 0.00884 },
    { "distal4",  0.01070,   0.00060, 0.00989 },
    { "distal5",  0.01070,   0.00074, 0.01051 },
    { "distal6",  0.01311,   0.00081, 0.01105 },
    { "distal7",  0.01311,   0.00186, 0.01153 },
    { "distal8",  0.014228,  0.00217, 0.01237 },
    { "distal9",  0.01471,   0.00351, 0.01306 },
    { "distal10", 0.01561,   0.00881, 0.01377 },
    };

    std::vector<geometry_msgs::msg::Pose> base_poses;
    base_poses.reserve(std::size(base_nom));

    double prev_center_y = 0.0;
    double prev_y_thk   = 0.0;

    for (size_t i = 0; i < std::size(base_nom); ++i) {
        const auto& b = base_nom[i];

        const double sx = b.x_nom * DISTAL_CENTER_SCALE;  // only bases get center scaling
        const double sy = b.y_thk;
        const double sz = b.z_h;

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        prim.dimensions.resize(3);
        prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sx;
        prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sy;
        prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sz;

        // Y stacking (first anchored at y_anchor + sy/2)
        const double center_y = (i == 0)
            ? (y_anchor + sy/2.0)
            : (prev_center_y + prev_y_thk/2.0 + sy/2.0);

        geometry_msgs::msg::Pose pose;
        pose.position.x = DISTAL_LOC_CAL_X;         // centered in X, like your originals
        pose.position.y = center_y;
        pose.position.z = z_plane + sz/2.0 + (CALIBRATION_Z);
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        mirror_about_distal_anchor(pose.position.x, pose.position.y);

        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "link_base";
        obj.id = b.id;
        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        collision_objects.push_back(obj);
        base_poses.push_back(pose);

        prev_center_y = center_y;
        prev_y_thk    = sy;
    }

    // =========================
    // 2) Extras per base (3 each)
    //    X uses NOMINAL base X (no center scale) to match your original constants.
    //    Z uses your original pattern: first 3 bases {0.8,0.7,0.5}, others {0.9,0.7,0.5}.
    // =========================
    const double x_scales[3] = { 0.9, 0.5, 1.0 };

    // Toggle: should extras also get DISTAL_CENTER_SCALE on X?
    constexpr bool APPLY_CENTER_SCALE_TO_EXTRAS = false;

    auto z_scale_for = [](size_t base_idx, int which)->double {
        if (which == 0) return (base_idx < 3) ? 0.8 : 0.9;
        if (which == 1) return 0.7;
        return 0.5;
    };

    int extra_id_counter = 11; // distal11…distal40

    for (size_t g = 0; g < std::size(base_nom); ++g) {
        const auto& b = base_nom[g];
        const auto& base_pose = base_poses[g];

        for (int e = 0; e < 3; ++e) {
            const double sx_nom   = b.x_nom * x_scales[e];
            const double sx_extra = APPLY_CENTER_SCALE_TO_EXTRAS ? (sx_nom * DISTAL_CENTER_SCALE) : sx_nom;
            const double sy       = b.y_thk;                        // match base’s Y thickness
            const double sz       = b.z_h * z_scale_for(g, e);

            shape_msgs::msg::SolidPrimitive prim;
            prim.type = shape_msgs::msg::SolidPrimitive::BOX;
            prim.dimensions.resize(3);
            prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sx_extra;
            prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sy;
            prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sz;

            geometry_msgs::msg::Pose pose;
            // Centered in X like the base slice:
            pose.position.x = DISTAL_LOC_CAL_X;

            // If you ever want left/right alignment with the base, uncomment one of these:
            // const double base_sx = b.x_nom * DISTAL_CENTER_SCALE;
            // pose.position.x = DISTAL_LOC_CAL_X - (base_sx - sx_extra) * 0.5; // left-align faces
            // pose.position.x = DISTAL_LOC_CAL_X + (base_sx - sx_extra) * 0.5; // right-align faces

            pose.position.y = base_pose.position.y; // same Y center as base slice
            pose.position.z = z_plane + sz/2.0 + (CALIBRATION_Z);
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;

            moveit_msgs::msg::CollisionObject obj;
            obj.header.frame_id = "link_base";
            obj.id = "distal" + std::to_string(extra_id_counter++);
            obj.primitives.push_back(prim);
            obj.primitive_poses.push_back(pose);
            obj.operation = moveit_msgs::msg::CollisionObject::ADD;

            collision_objects.push_back(obj);
        }
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
    // addCollisionObject(planning_scene_interface);
    addCollisionObject_Mesh(planning_scene_interface);

    // Sleep briefly to allow the collision object to propagate
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(node->get_logger(), "add_scene over");

    rclcpp::shutdown();
    return 0;
}
