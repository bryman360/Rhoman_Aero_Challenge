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
        mag_subscription = this->create_subscription<MagTimestamped>("sensor_mag_ts", 10, std::bind(&EstimatorNode::mag_subscription_callback, this, _1));
        ang_subscription = this->create_subscription<AngTimestamped>("vehicle_ang_ts", 10, std::bind(&EstimatorNode::ang_subscription_callback, this, _1));
        azi_ele_publisher = this->create_publisher<AziEleTimestamped>("azi_ele_ts", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EstimatorNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Estimator is ready");
        timer->cancel();
    }
private:
    rclcpp::Subscription<MagTimestamped>::SharedPtr mag_subscription;
    rclcpp::Subscription<AngTimestamped>::SharedPtr ang_subscription;
    rclcpp::Publisher<AziEleTimestamped>::SharedPtr azi_ele_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    EstimatorPtr estimator;
    bool first_data_point_seen = false;
    std::chrono::steady_clock::time_point last_loop_chrono_timestamp;
    double current_sim_time_s;

    void mag_subscription_callback(const MagTimestamped::SharedPtr msg) {
        Eigen::Vector3d vals = Eigen::Vector3d(msg->x, msg->y, msg->z);
        estimator->ingMagMessage(msg->timestamp, vals);
        if (!first_data_point_seen) {
            first_data_point_seen = true;
            last_loop_chrono_timestamp = std::chrono::steady_clock::now();
            current_sim_time_s = msg->timestamp / 1000000;
            timer->reset();
        }
    }

    void ang_subscription_callback(const AngTimestamped::SharedPtr msg) {
        Eigen::Vector3d vals = Eigen::Vector3d(msg->roll, msg->pitch, msg->yaw);
        estimator->ingAngMessage(msg->timestamp, vals);
        if (!first_data_point_seen) {
            first_data_point_seen = true;
            last_loop_chrono_timestamp = std::chrono::steady_clock::now();
            current_sim_time_s = msg->timestamp / 1000000;
            timer->reset();
        }
    }

    void timer_callback() {
        std::chrono::steady_clock::time_point current_chrono_timestamp = std::chrono::steady_clock::now();
        std::chrono::duration<double> chrono_time_diff = current_chrono_timestamp - last_loop_chrono_timestamp;
        last_loop_chrono_timestamp = current_chrono_timestamp;

        double time_diff_s = chrono_time_diff.count();
        current_sim_time_s += time_diff_s;
        estimator->spin(current_sim_time_s);
        if (estimator->have_new_mag_meas) {
            auto msg = AziEleTimestamped();
            Eigen::Vector3d avg_az_el = estimator->getAvgAzEL();
            Eigen::Vector3d instant_az_el = estimator->getInstAzEL();
            msg.timestamp = avg_az_el[0];
            msg.average_azimuth = avg_az_el[1];
            msg.average_elevation = avg_az_el[2];
            msg.instant_azimuth = instant_az_el[1];
            msg.instant_elevation = instant_az_el[2];
            azi_ele_publisher->publish(msg);
            estimator->have_new_mag_meas = false;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>("estimator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}