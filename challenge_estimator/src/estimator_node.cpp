#include <rclcpp/rclcpp.hpp>
#include <challenge_interfaces/msg/mag_timestamped.hpp>
#include <challenge_interfaces/msg/ang_timestamped.hpp>
#include <challenge_interfaces/msg/azi_timestamped.hpp>
#include <challenge_interfaces/msg/ele_timestamped.hpp>

using MagTimestamped = challenge_interfaces::msg::MagTimestamped;
using AngTimestamped = challenge_interfaces::msg::AngTimestamped;
using AziTimestamped = challenge_interfaces::msg::AziTimestamped;
using EleTimestamped = challenge_interfaces::msg::EleTimestamped;
using namespace std::placeholders;

class EstimatorNode : public rclcpp::Node {
public:
    EstimatorNode(std::string name) : Node(name) {
        mag_subscription = this->create_subscription<MagTimestamped>("sensor_mag_ts", 10, std::bind(&EstimatorNode::mag_subscription_callback, this, _1));
        ang_subscription = this->create_subscription<MagTimestamped>("vehicle_ang_ts", 10, std::bind(&EstimatorNode::ang_subscription_callback, this, _1));
        azi_publisher = this->create_publisher<AziTimestamped>("azimuth_ts", 10);
        ele_publisher = this->create_publisher<EleTimestamped>("elevation_ts", 10);
        timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Estimator::timer_callback, this));
    }
private:
    rclcpp::Subscription<MagTimestamped>::SharedPtr mag_subscription;
    rclcpp::Subscription<AngTimestamped>::SharedPtr ang_subscription;
    rclcpp::Publisher<AziTimestamped>::SharedPtr azi_publisher;
    rclcpp::Publisher<EleTimestamped>::SharedPtr ele_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    void mag_subscription_callback(MagTimestamped msg) {
    }

    void ang_subscription_callback(AngTimestamped msg) {
    }

    void timer_callback() {
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>("estimator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}