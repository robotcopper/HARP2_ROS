#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robot_arm_motion_planner/robot_arm_motion_planner.hpp"
#include <kdl/jntarray.hpp>
#include <vector>

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher"),
          q_start_kdl(3), 
          q_end_kdl(3),
          current_point_index_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/target_joint_positions", 10);

        /// ############# Settings ############# //

        // double q_end[3] = {(-97.097842583*(-(0.375))), 2.0944, 2.0944}; // Point1
        double q_start[3] = {(97.097842583*(2.02*(0.1)-0.034)), 2.0944, 2.0944}; // Point1 
        double q_end[3]   = {0.0, 2.0944, 0.523599}; // Point2
        v = {5.0, 1.0, 1.0};
        a = {0.0, 1.0, 1.0};
        //v = 1.0; // joints velocity
        //a = 1.0; // joints acceleration
        
        /// ############# Settings ############# //

        for (int i = 0; i < 3; ++i) {
            q_start_kdl(i) = q_start[i];
            q_end_kdl(i) = q_end[i];
        }

        generateTrajectory();

        // Calling publishNextPoint each 0.01
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.01),
            std::bind(&JointTrajectoryPublisher::publishNextPoint, this));
    }

private:
    void generateTrajectory()
    {
        trajectory_data_ = robot_arm_motion_planner::JointTrajectoryPlanner::interpolateJointMotion(
            q_start_kdl, q_end_kdl, v, a);

        current_point_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "\033[1;32mNew trajectory generated!\033[0m");
    }

    void publishNextPoint()
    {
        // if (trajectory_data_.empty() || current_point_index_ * 3 + 2 >= trajectory_data_.size()) {
        //     // Switching points
        //     std::swap(q_start_kdl, q_end_kdl);
        //     generateTrajectory();
        //     return;
        // }
        if (trajectory_data_.empty() || current_point_index_ * 3 + 2 >= trajectory_data_.size()) {
        //     if (!waiting_before_switch_) {
        //         waiting_before_switch_ = true;
    
        //         switch_timer_ = this->create_wall_timer(
        //             std::chrono::seconds(3),
        //             [this]() {
        //                 std::swap(q_start_kdl, q_end_kdl);
        //                 generateTrajectory();
        //                 current_point_index_ = 0;
        //                 waiting_before_switch_ = false;
        //                 switch_timer_->cancel(); // pour éviter que ça tourne
        //             });
    
        //         return;  // on sort pour attendre les 3 secondes
        //     }
            return;
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;

        const auto &positions = trajectory_data_[current_point_index_ * 3];
        const auto &velocities = trajectory_data_[current_point_index_ * 3 + 1];
        const auto &accelerations = trajectory_data_[current_point_index_ * 3 + 2];

        point.positions = positions;
        point.velocities = velocities;
        point.accelerations = accelerations;
        point.time_from_start = rclcpp::Duration::from_seconds(current_point_index_ * 0.01); // Elapsed time

        publisher_->publish(point);

        current_point_index_++;
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    KDL::JntArray q_start_kdl, q_end_kdl;
    std::vector<std::vector<double>> trajectory_data_;
    size_t current_point_index_;

    std::vector<double> v;
    std::vector<double> a;


    rclcpp::TimerBase::SharedPtr switch_timer_;
    bool waiting_before_switch_ = false;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
