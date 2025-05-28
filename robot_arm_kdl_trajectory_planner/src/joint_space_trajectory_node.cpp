#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_arm_motion_planner/robot_arm_motion_planner.hpp"
#include <kdl/jntarray.hpp>
#include <vector>

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher"),
          q_current_kdl(3),
          q_target_kdl(3),
          current_point_index_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/target_joint_positions", 10);

        subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_joint_command", 10,
            std::bind(&JointTrajectoryPublisher::targetCallback, this, std::placeholders::_1));

        // Initial position (à adapter selon ton besoin)
        double q_init[3] = {(97.097842583*(2.02*(0.1)-0.033)), 2.0944, 2.0944}; //POSITION DE DEPART DU BRAS (DOIS ETRE MIS A LA POSITION BANNIERE) ///// position relative à 0 , absolu du servo 2 , absolu du servo 3
        for (int i = 0; i < 3; ++i) {
            q_current_kdl(i) = q_init[i];
            q_target_kdl(i) = q_init[i];
        }

        v = {5.0, 1.0, 1.0};
        a = {0.0, 1.0, 1.0};

        // Génération initiale de trajectoire "nulle" (pas obligatoire)
        generateTrajectory();

        // Timer pour publier la trajectoire point par point toutes les 10 ms
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&JointTrajectoryPublisher::publishNextPoint, this));
    }

private:
    void targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 3) {
            RCLCPP_WARN(this->get_logger(), "Received target of wrong size (expected 3)");
            return;
        }

        for (int i = 0; i < 3; ++i) {
            q_target_kdl(i) = msg->data[i];
        }

        generateTrajectory();

        trajectory_ready_ = true; // Une trajectoire est prête à être publiée

        RCLCPP_INFO(this->get_logger(), "New target received, trajectory regenerated");
    }



    void generateTrajectory()
    {
        // Génère la trajectoire entre q_current_kdl (start) et q_target_kdl (end)
        trajectory_data_ = robot_arm_motion_planner::JointTrajectoryPlanner::interpolateJointMotion(
            q_current_kdl, q_target_kdl, v, a);

        current_point_index_ = 0;
    }

    void publishNextPoint()
    {
        if (!trajectory_ready_) {
            // Pas de trajectoire à publier, ne fait rien
            return;
        }

        if (current_point_index_ >= trajectory_data_.size() / 3) {
            // Trajectoire terminée, arrêt de la publication
            for (int i = 0; i < 3; ++i) {
                q_current_kdl(i) = q_target_kdl(i);
            }
            trajectory_ready_ = false; // plus rien à publier
            return;
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;

        const auto &positions = trajectory_data_[current_point_index_ * 3];
        const auto &velocities = trajectory_data_[current_point_index_ * 3 + 1];
        const auto &accelerations = trajectory_data_[current_point_index_ * 3 + 2];

        point.positions = positions;
        point.velocities = velocities;
        point.accelerations = accelerations;
        point.time_from_start = rclcpp::Duration::from_seconds(current_point_index_ * 0.01);

        publisher_->publish(point);

        current_point_index_++;

    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    KDL::JntArray q_current_kdl;  // Position courante réelle (dernière position atteinte)
    KDL::JntArray q_target_kdl;   // Nouvelle cible reçue

    std::vector<std::vector<double>> trajectory_data_;
    size_t current_point_index_;

    std::vector<double> v;
    std::vector<double> a;
    
    bool trajectory_ready_ = false;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
