import rclpy
from rclpy.node import Node
import numpy as np
from matplotlib import pyplot as plt
from challenge_interfaces.msg import AngTimestamped, MagTimestamped

class MonitoringNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("update_rate_hz", 1)
        self.declare_parameter("disable_visualization", False)
        self.mag_subscription = self.create_subscription(MagTimestamped, "sensor_mag_ts", self.mag_subscription_callback, 10)
        self.ang_subscription = self.create_subscription(AngTimestamped, "vehicle_ang_ts", self.ang_subscription_callback, 10)
        self.estimator_subscription = self.create_subscription(MagTimestamped, "estimator_vals_ts", self.estimator_subscription_callback, 10)
        self.mag_timestamps = np.array([], dtype=np.uint32)
        self.ang_timestamps = np.array([], dtype=np.uint32)
        self.mag_data = None
        self.ang_data = None
        self.get_logger().info("Monitoring is ready.")
        if not self.get_parameter("disable_visualization").value:
            update_rate_hz = self.get_parameter("update_rate_hz").value
            if type(update_rate_hz) != int:
                update_rate_hz = 1
            self.visualization_timer = self.create_timer((1 / update_rate_hz), self.visualization_timer_callback)
            self.plt_fig = plt.figure(figsize=(10, 6))
            self.gridspec = self.plt_fig.add_gridspec(2, 4)

    def mag_subscription_callback(self, msg: MagTimestamped):
        self.mag_timestamps = np.append(self.mag_timestamps, msg.timestamp)
        if self.mag_data is not None:
            self.mag_data = np.vstack((self.mag_data, np.array([msg.x, msg.y, msg.z], dtype=np.float32)))
        else:
            self.mag_data = np.array([[msg.x, msg.y, msg.z]], dtype=np.float32)

    def ang_subscription_callback(self, msg: AngTimestamped):
        self.ang_timestamps = np.append(self.ang_timestamps, msg.timestamp)
        if self.ang_data is not None:
            self.ang_data = np.vstack((self.ang_data, [msg.roll, msg.pitch, msg.yaw]))
        else:
            self.ang_data = np.array([[msg.roll, msg.pitch, msg.yaw]], dtype=np.float32)

    def estimator_subscription_callback(self, msg: MagTimestamped):
        pass

    def visualization_timer_callback(self):
        if len(self.mag_timestamps) < 1000:
            return
        # mag_subplot = self.plt_fig.add_subplot(self.gridspec[0:2, 0])
        scaled_mag_ts = self.mag_timestamps / 1000000
        plt.plot(scaled_mag_ts, self.mag_data[:, 0], color='red', linewidth=0.5)
        plt.plot(scaled_mag_ts, self.mag_data[:, 1], color='green', linewidth=0.5)
        plt.plot(scaled_mag_ts, self.mag_data[:, 2], color='blue', linewidth=0.5)
        plt.xlabel('Flight Time [s]')
        plt.ylabel('Relative Strength [-]')
        plt.title('Raw Mag x/y/z (r/g/b)')
        plt.xlim(scaled_mag_ts[0], scaled_mag_ts[0])

        plt.show()
        plt.waitforbuttonpress()



def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode("monitoring")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()