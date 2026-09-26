import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt
from challenge_interfaces.msg import AngTimestamped, MagTimestamped

class MonitoringNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.mag_subscription = self.create_subscription(MagTimestamped, "sensor_mag_ts", self.mag_subscription_callback, 10)
        self.ang_subscription = self.create_subscription(AngTimestamped, "vehicle_ang_ts", self.ang_subscription_callback, 10)
        self.estimator_subscription = self.create_subscription(MagTimestamped, "estimator_vals_ts", self.estimator_subscription_callback, 10)

    def mag_subscription_callback(self, msg: MagTimestamped):
        pass

    def ang_subscription_callback(self, msg: AngTimestamped):
        pass

    def estimator_subscription_callback(self, msg: MagTimestamped):
        pass



def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode("monitoring")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()