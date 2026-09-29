#include <rclcpp/rclcpp.hpp>
#include <estimator.hpp>
#include <challenge_interfaces/msg/mag_timestamped.hpp>
#include <challenge_interfaces/msg/ang_timestamped.hpp>
#include <challenge_interfaces/msg/azi_ele_timestamped.hpp>

using MagTimestamped = challenge_interfaces::msg::MagTimestamped;
using AngTimestamped = challenge_interfaces::msg::AngTimestamped;
using AziEleTimestamped = challenge_interfaces::msg::AziEleTimestamped;
using namespace std::placeholders;

class EstimatorNode : public rclcpp::Node {
public:
    EstimatorNode(std::string name) : Node(name) {
        estimator = std::make_shared<Estimator>();
        this->declare_parameter("spin_rate", 10000);
        mag_subscription = this->create_subscription<MagTimestamped>("sensor_mag_ts", 10, std::bind(&EstimatorNode::mag_subscription_callback, this, _1));
        ang_subscription = this->create_subscription<AngTimestamped>("vehicle_ang_ts", 10, std::bind(&EstimatorNode::ang_subscription_callback, this, _1));
        azi_ele_publisher = this->create_publisher<AziEleTimestamped>("azi_ele_ts", 10);
        timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&EstimatorNode::timer_callback, this));
    }
private:
    rclcpp::Subscription<MagTimestamped>::SharedPtr mag_subscription;
    rclcpp::Subscription<AngTimestamped>::SharedPtr ang_subscription;
    rclcpp::Publisher<AziEleTimestamped>::SharedPtr azi_ele_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    EstimatorPtr estimator;

    void mag_subscription_callback(MagTimestamped msg) {
        Eigen::Vector3d vals(msg.x, msg.y, msg.z);
        estimator->ingMagMessage(12, vals);
    }

    void ang_subscription_callback(AngTimestamped msg) {
        Eigen::Vector3d vals(msg.roll, msg.pitch, msg.yaw);
        estimator->ingAngMessage(12, vals);
    }

    void timer_callback() {
        rclcpp::Rate loop_rate(1 / 10000);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>("estimator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}