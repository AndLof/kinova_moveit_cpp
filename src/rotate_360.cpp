#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "smooth_rotate_360",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = rclcpp::get_logger("smooth_rotate_360");

    MoveGroupInterface move_group(node, "manipulator");

    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    // ------------------------------------------------------------
    // IMPORTANT: let MoveIt sync state properly
    // ------------------------------------------------------------
    rclcpp::sleep_for(std::chrono::seconds(2));
    move_group.setStartStateToCurrentState();

    // ------------------------------------------------------------
    // Get a safe robot state snapshot
    // ------------------------------------------------------------
    moveit::core::RobotStatePtr current_state =
        move_group.getCurrentState(10);

    if (!current_state)
    {
        RCLCPP_ERROR(logger, "Failed to get current state");
        return -1;
    }

    const moveit::core::JointModelGroup* jmg =
        current_state->getJointModelGroup("manipulator");

    std::vector<double> joints;
    current_state->copyJointGroupPositions(jmg, joints);

    if (joints.empty())
    {
        RCLCPP_ERROR(logger, "Empty joint state");
        return -1;
    }

    // ------------------------------------------------------------
    // Smooth 360° rotation on joint_1
    // ------------------------------------------------------------
    const double TWO_PI = 2.0 * M_PI;

    double start_angle = joints[0];
    double target_angle = start_angle + TWO_PI;

    joints[0] = target_angle;

    RCLCPP_INFO(logger,
        "Rotating joint_1 from %f to %f (360° smooth trajectory)",
        start_angle,
        target_angle);

    // ------------------------------------------------------------
    // Set goal
    // ------------------------------------------------------------
    move_group.setJointValueTarget(joints);

    // ------------------------------------------------------------
    // PLAN (single trajectory)
    // ------------------------------------------------------------
    MoveGroupInterface::Plan plan;

    bool success =
        (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        RCLCPP_ERROR(logger, "Planning failed");
        return -1;
    }

    RCLCPP_INFO(logger, "Plan successful, executing...");

    // ------------------------------------------------------------
    // EXECUTE (smooth motion)
    // ------------------------------------------------------------
    auto result = move_group.execute(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Execution failed");
        return -1;
    }

    RCLCPP_INFO(logger, "360° smooth rotation completed");

    rclcpp::shutdown();
    return 0;
}