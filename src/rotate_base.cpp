#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class RotateBase360 : public rclcpp::Node
{
public:
    RotateBase360()
    : Node("rotate_360_direct"),
      initialized_(false),
      accumulated_rotation_(0.0),
      previous_angle_(0.0)
    {
        // --------------------------------------------------------
        // Subscriber
        // --------------------------------------------------------
        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                    &RotateBase360::jointStateCallback,
                    this,
                    std::placeholders::_1));

        // --------------------------------------------------------
        // Publisher
        // --------------------------------------------------------
        traj_pub_ =
            this->create_publisher<
                trajectory_msgs::msg::JointTrajectory>(
                    "/joint_trajectory_controller/joint_trajectory",
                    10);

        // --------------------------------------------------------
        // Control loop
        // --------------------------------------------------------
        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &RotateBase360::controlLoop,
                    this));

        RCLCPP_INFO(this->get_logger(),
                    "Rotate360 node started");
    }

private:

    // ============================================================
    // JOINT STATE CALLBACK
    // ============================================================
    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Find indices of robot joints
        std::vector<double> ordered_positions(6);

        bool found_all = true;

        for (int joint_num = 1; joint_num <= 6; ++joint_num)
        {
            std::string joint_name =
                "joint_" + std::to_string(joint_num);

            bool found = false;

            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                if (msg->name[i] == joint_name)
                {
                    ordered_positions[joint_num - 1] =
                        msg->position[i];

                    found = true;
                    break;
                }
            }

            if (!found)
            {
                found_all = false;
                break;
            }
        }

        if (!found_all)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Not all joints found yet");
            return;
        }

        // --------------------------------------------------------
        // First callback = save initial posture
        // --------------------------------------------------------
        if (!initialized_)
        {
            initial_positions_ = ordered_positions;

            start_angle_ = initial_positions_[0];
            previous_angle_ = start_angle_;
            current_angle_ = start_angle_;

            initialized_ = true;

            RCLCPP_INFO(this->get_logger(),
                        "Initial posture saved");

            for (size_t i = 0; i < initial_positions_.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(),
                            "joint_%ld = %.3f",
                            i + 1,
                            initial_positions_[i]);
            }

            return;
        }

        // --------------------------------------------------------
        // Update current angle
        // --------------------------------------------------------
        current_angle_ = ordered_positions[0];

        // --------------------------------------------------------
        // Wrapped angular difference
        // --------------------------------------------------------
        double delta = current_angle_ - previous_angle_;

        while (delta > M_PI)
            delta -= 2.0 * M_PI;

        while (delta < -M_PI)
            delta += 2.0 * M_PI;

        accumulated_rotation_ += delta;

        previous_angle_ = current_angle_;

        RCLCPP_INFO(this->get_logger(),
                    "joint_1 = %.3f | accumulated = %.3f",
                    current_angle_,
                    accumulated_rotation_);
    }

    // ============================================================
    // CONTROL LOOP
    // ============================================================
    void controlLoop()
    {
        if (!initialized_)
            return;

        // --------------------------------------------------------
        // Stop after full rotation
        // --------------------------------------------------------
        if (std::fabs(accumulated_rotation_) >=
            (2.0 * M_PI - 0.05))
        {
            RCLCPP_INFO(this->get_logger(),
                        "360 degree rotation completed");

            rclcpp::shutdown();
            return;
        }

        // --------------------------------------------------------
        // Build trajectory message
        // --------------------------------------------------------
        trajectory_msgs::msg::JointTrajectory traj;

        traj.joint_names = {
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;

        // --------------------------------------------------------
        // Keep INITIAL posture
        // --------------------------------------------------------
        point.positions = initial_positions_;

        // Rotate ONLY joint_1
        point.positions[0] = current_angle_ + 0.05;

        // Motion duration
        point.time_from_start.sec = 1;

        traj.points.push_back(point);

        traj_pub_->publish(traj);
    }

    // ============================================================
    // ROS interfaces
    // ============================================================
    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr
            joint_state_sub_;

    rclcpp::Publisher<
        trajectory_msgs::msg::JointTrajectory>::SharedPtr
            traj_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ============================================================
    // State variables
    // ============================================================
    bool initialized_;

    std::vector<double> initial_positions_;

    double start_angle_;
    double current_angle_;
    double previous_angle_;

    double accumulated_rotation_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RotateBase360>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}