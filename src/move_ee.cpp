#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

//lib for protection zone 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char * argv[])
{
  // ----------------------------
  // 1. ROS2 INIT
  // ----------------------------
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("kinova_move_ee");

  RCLCPP_INFO(node->get_logger(), "Starting MoveIt Cartesian goal example");

  // ----------------------------
  // 2. MOVE GROUP INTERFACE
  // ----------------------------
  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
            move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(node->get_logger(), "End effector link: %s",
            move_group.getEndEffectorLink().c_str());

  geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose();

  RCLCPP_INFO(node->get_logger(),
            "Current position: x=%.4f y=%.4f z=%.4f",
            current.pose.position.x,
            current.pose.position.y,
            current.pose.position.z);

  RCLCPP_INFO(node->get_logger(),
            "Current orientation: x=%.4f y=%.4f z=%.4f w=%.4f",
            current.pose.orientation.x,
            current.pose.orientation.y,
            current.pose.orientation.z,
            current.pose.orientation.w);
  
  //creare planning scene for protection zone
  //commento?
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // ----------------------------
  // 3. SET START STATE
  // ----------------------------
  move_group.setStartStateToCurrentState();

  // ----------------------------
  // PROTECTION ZONE (BOX)
  // ----------------------------
  //commento?
  //moveit_msgs::msg::CollisionObject collision_object;
  //collision_object.header.frame_id = "base_link";  // importante
  //collision_object.id = "forbidden_box";

  // dimensioni (metri)
  //commento?
  //shape_msgs::msg::SolidPrimitive primitive;
  //primitive.type = primitive.BOX;
  //primitive.dimensions = {0.1, 0.1, 0.1};  // 20cm x 20cm x 20cm

  // posizione del box
  //commento?
  //geometry_msgs::msg::Pose box_pose;
  //box_pose.orientation.w = 1.0;
  //box_pose.position.x = -0.14;
  //box_pose.position.y = -0.36;
  //box_pose.position.z = 0.13;

  // assegnazione
  //commento?
  //collision_object.primitives.push_back(primitive);
  //collision_object.primitive_poses.push_back(box_pose);
  //collision_object.operation = collision_object.ADD;

  // aggiungi alla scena
  //planning_scene_interface.applyCollisionObject(collision_object);

  // fondamentale: aspetta che MoveIt aggiorni la scena
  rclcpp::sleep_for(std::chrono::seconds(1));

  // ----------------------------
  // 4. DEFINE GOAL POSE (END EFFECTOR)
  // ----------------------------
  geometry_msgs::msg::Pose target_pose;

  //target_pose.position.x = 0.4;
  //target_pose.position.y = 0.0;
  //target_pose.position.z = 0.3;

  //target_pose.position.x = 0.4;
  //target_pose.position.y = -0.18;
  //target_pose.position.z = 0.88;

  //simulation robot test
  //target_pose.position.x = -0.32;
  //target_pose.position.y = 0.00;
  //target_pose.position.z = 0.8;

  //real robot test
  //target_pose.position.x = -0.1435;
  //target_pose.position.y = -0.6285;
  //target_pose.position.z = 0.4315;

  // Quaternion (orientazione)
  //target_pose.orientation.x = 0.0;
  //target_pose.orientation.y = 1.0;
  //target_pose.orientation.z = 0.0;
  //target_pose.orientation.w = 0.0;

  //target_pose.orientation.x = 1.19;
  //target_pose.orientation.y = -0.4;
  //target_pose.orientation.z = -1.91;
  //target_pose.orientation.w = 0.0;

  //simulation robot test
  //target_pose.orientation.x = 0.868;
  //target_pose.orientation.y = 0.00;
  //target_pose.orientation.z = -0.496;
  //target_pose.orientation.w = 0.0;

  //real robot test
  //target_pose.orientation.x = 0.6996;
  //target_pose.orientation.y = 0.0385;
  //target_pose.orientation.z = 0.0759;
  //target_pose.orientation.w = 0.7093;
  
  //position for real arm on spot -> home position
  target_pose.position.x = 0.396;
  target_pose.position.y = 0.018;
  target_pose.position.z = 0.333;

  target_pose.orientation.x = 0.488;
  target_pose.orientation.y = 0.535;
  target_pose.orientation.z = 0.497;
  target_pose.orientation.w = 0.478;

  RCLCPP_INFO(node->get_logger(), "Setting pose goal...");

  // ----------------------------
  // 5. SET GOAL TO MOVEIT
  // ----------------------------
  move_group.setPoseTarget(target_pose);

  // ----------------------------
  // 6. PLAN
  // ----------------------------
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success =
    (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // ----------------------------
  // 7. EXECUTE
  // ----------------------------
  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Plan OK → executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  // ----------------------------
  // 8. SHUTDOWN
  // ----------------------------
  rclcpp::shutdown();
  return 0;
}
