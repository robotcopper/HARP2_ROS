#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TwistFilter : public rclcpp::Node {
public:
    TwistFilter() : Node("twist_acc_filter") {
        // Paramètres d'accélération maximale
        max_linear_accel_ = this->declare_parameter("max_linear_accel", 0.5);  // m/s²
        max_angular_accel_ = this->declare_parameter("max_angular_accel", 0.5);  // rad/s²

        // Initialisation des abonnements et publications
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/keyboard_cmd_vel", 10, std::bind(&TwistFilter::twist_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/omnidirectional_controller/cmd_vel_unstamped", 10);

        last_time_ = this->get_clock()->now();

        // Créer un timer pour publier périodiquement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Timer de 50 ms
            std::bind(&TwistFilter::timer_callback, this));  // Appel de la méthode périodique

        // Initialiser la dernière vitesse reçue avec des valeurs nulles
        last_received_twist_ = geometry_msgs::msg::Twist();
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Mise à jour de la vitesse reçue
        last_received_twist_ = *msg;
        last_time_ = this->get_clock()->now();  // Reset last_time when receiving a new message
    }

    void timer_callback() {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();  // Durée écoulée en secondes

        // Si aucun message n'a été reçu, forcer l'accélération à zéro
        if ((now - last_time_).seconds() > 1.0) {  // Si plus d'une seconde sans message
            last_received_twist_.linear.x = 0.0;
            last_received_twist_.angular.z = 0.0;
        }

        // Appliquer la limitation d'accélération même si aucune nouvelle commande n'est arrivée
        if (std::abs(last_received_twist_.angular.z) > 0.1) {
            // Si la vitesse angulaire est significative, on peut augmenter la limite d'accélération pour les rotations
            current_twist_.angular.z = limit_accel(current_twist_.angular.z, last_received_twist_.angular.z, max_angular_accel_ * 2, dt);  // Augmenter la vitesse d'accélération angulaire
        } else {
            current_twist_.angular.z = limit_accel(current_twist_.angular.z, last_received_twist_.angular.z, max_angular_accel_, dt);
        }

        current_twist_.linear.x = limit_accel(current_twist_.linear.x, last_received_twist_.linear.x, max_linear_accel_, dt);

        // Publication du message filtré
        publisher_->publish(current_twist_);
    }

    // Fonction pour limiter l'accélération
    double limit_accel(double current, double target, double max_accel, double dt) {
        double diff = target - current;
        if (std::abs(diff) > max_accel * dt) {
            return current + max_accel * dt * ((diff > 0) ? 1.0 : -1.0);
        } else {
            return target;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;  // Timer pour la mise à jour périodique

    geometry_msgs::msg::Twist current_twist_;
    geometry_msgs::msg::Twist last_received_twist_;  // Dernière commande reçue

    rclcpp::Time last_time_;  // Dernier moment de réception de message

    // Paramètres d'accélération maximale
    double max_linear_accel_;
    double max_angular_accel_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
