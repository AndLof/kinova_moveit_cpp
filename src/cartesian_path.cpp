#include <memory>
#include <vector>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // ----------------------------
  // 1. ROS2 INIT
  // ----------------------------
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);  // 🔥 FONDAMENTALE
  node_options.parameter_overrides({{"use_sim_time", true}});

  auto node = rclcpp::Node::make_shared("kinova_cartesian_path", node_options);

  // 🔥 SPIN (necessario per joint_states)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  RCLCPP_INFO(node->get_logger(), "Starting Cartesian trajectory example");

  // ----------------------------
  // 2. MOVE GROUP
  // ----------------------------
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // ----------------------------
  // 3. ATTENDI STATO ROBOT
  // ----------------------------
  RCLCPP_INFO(node->get_logger(), "Waiting for robot state...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  move_group.setStartStateToCurrentState();

  // ----------------------------
  // 4. POSE ATTUALE
  // ----------------------------
  auto current_pose = move_group.getCurrentPose();

  if (current_pose.header.stamp.sec == 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid current pose received!");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  geometry_msgs::msg::Pose start_pose = current_pose.pose;

  RCLCPP_INFO(node->get_logger(), "Current pose received");

  // ----------------------------
  // 5. WAYPOINTS
  // ----------------------------
  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose target = start_pose;

  // piccoli movimenti → più stabile
  target.position.x -= 0.05;
  waypoints.push_back(target);

  target.position.z += 0.02;
  waypoints.push_back(target);

  target.position.y -= 0.00;
  waypoints.push_back(target);

  // ----------------------------
  // 6. CARTESIAN PATH
  // ----------------------------
  moveit_msgs::msg::RobotTrajectory trajectory;

  const double eef_step = 0.005;      // 5 mm
  const double jump_threshold = 0.0;

  RCLCPP_INFO(node->get_logger(), "Computing Cartesian path...");

  double fraction = move_group.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory
  );

  RCLCPP_INFO(node->get_logger(), "Path computed: %.2f%%", fraction * 100.0);

  // ----------------------------
  // 7. EXECUTE
  // ----------------------------
  if (fraction > 0.9)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    RCLCPP_INFO(node->get_logger(), "Executing trajectory...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Cartesian path failed (low fraction)");
  }

  // ----------------------------
  // 8. SHUTDOWN
  // ----------------------------
  rclcpp::shutdown();
  spinner.join();

  return 0;
}