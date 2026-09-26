#include <rclcpp/rclcpp.hpp>

class EstimatorNode : public rclcpp::Node {
public:
    EstimatorNode(std::string name) : Node(name) {

    }
private:
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>("estimator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}