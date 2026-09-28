import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from challenge_interfaces.msg import AngTimestamped, MagTimestamped

import numpy as np
import csv
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class MonitoringNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("output_dir", "/home/bryman360/csv_files")
        self.mag_subscription = self.create_subscription(MagTimestamped, "sensor_mag_ts", self.mag_subscription_callback, 10)
        self.ang_subscription = self.create_subscription(AngTimestamped, "vehicle_ang_ts", self.ang_subscription_callback, 10)
        self.estimator_subscription = self.create_subscription(MagTimestamped, "estimator_vals_ts", self.estimator_subscription_callback, 10)

        self.mag_timestamps = np.array([], dtype=np.uint32)
        self.ang_timestamps = np.array([], dtype=np.uint32)
        self.mag_data = None
        self.ang_data = None

        self.output_dir = str(self.get_parameter("output_dir").value)
        self.lock = threading.Lock()

        with open(self.output_dir + "/mag_values.csv", "w", newline="", encoding="utf-8") as mag_file:
            csv_writer = csv.writer(mag_file)
            header_line = ["Time", "Mag X", "Mag Y", "Mag Z"]
            csv_writer.writerow(header_line)

        with open(self.output_dir + "/attitude_converted_to_rollpitchyaw.csv", "w", newline="", encoding="utf-8") as mag_file:
            csv_writer = csv.writer(mag_file)
            header_line = ["Time", "Roll Deg", "Pitch Deg", "Yaw Deg"]
            csv_writer.writerow(header_line)

        self.get_logger().info("Monitoring is ready.")
            

    def mag_subscription_callback(self, msg: MagTimestamped):
        with open(self.output_dir + "/mag_values.csv", 'a', newline="", encoding="utf-8") as mag_file:
            csv_writer = csv.writer(mag_file)
            csv_writer.writerow([msg.timestamp, msg.x, msg.y, msg.z])

        with self.lock:
            self.mag_timestamps = np.append(self.mag_timestamps, msg.timestamp)
            if self.mag_data is not None:
                self.mag_data = np.vstack((self.mag_data, np.array([msg.x, msg.y, msg.z], dtype=np.float32)))
            else:
                self.mag_data = np.array([[msg.x, msg.y, msg.z]], dtype=np.float32)


    def ang_subscription_callback(self, msg: AngTimestamped):
        with open(self.output_dir + "/attitude_converted_to_rollpitchyaw.csv", 'a', newline="", encoding="utf-8") as mag_file:
            csv_writer = csv.writer(mag_file)
            csv_writer.writerow([msg.timestamp, msg.roll, msg.pitch, msg.yaw])

        with self.lock:
            self.ang_timestamps = np.append(self.ang_timestamps, msg.timestamp)
            if self.ang_data is not None:
                self.ang_data = np.vstack((self.ang_data, [msg.roll, msg.pitch, msg.yaw]))
            else:
                self.ang_data = np.array([[msg.roll, msg.pitch, msg.yaw]], dtype=np.float32)

    def estimator_subscription_callback(self, msg: MagTimestamped):
        pass
        



def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode("monitoring")

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    fig = plt.figure(figsize=(10, 6))
    gridspec = fig.add_gridspec(2, 4)
    mag_subplot = fig.add_subplot(gridspec[0:2, 0])
    ang_rp_subplot = fig.add_subplot(gridspec[0, 1])
    ang_y_subplot = fig.add_subplot(gridspec[1, 1])
    azi_subplot = fig.add_subplot(gridspec[0, 2:])
    ele_subplot = fig.add_subplot(gridspec[1, 2:])
    scaled_mag_ts = []
    scaled_ang_ts = []


    x_line, = mag_subplot.plot(scaled_mag_ts, [], color='red', linewidth=0.5)
    y_line, = mag_subplot.plot(scaled_mag_ts, [], color='green', linewidth=0.5)
    z_line, = mag_subplot.plot(scaled_mag_ts, [], color='blue', linewidth=0.5)
    mag_subplot.set_xlabel('Flight Time [s]')
    mag_subplot.set_ylabel('Relative Strength [-]')
    mag_subplot.set_title('Raw Mag x/y/z (r/g/b)')
    mag_subplot.set_ylim(-0.4, 1)

    r_line, = ang_rp_subplot.plot(scaled_ang_ts, [], color='red', linewidth=0.5)
    p_line, = ang_rp_subplot.plot(scaled_ang_ts, [], color='green', linewidth=0.5)
    ang_rp_subplot.set_xlabel('Flight Time [s]')
    ang_rp_subplot.set_ylabel('Degrees')
    ang_rp_subplot.set_title('Roll (red) and Pitch (green)')
    ang_rp_subplot.set_ylim(-80, 80)

    yaw_line, = ang_y_subplot.plot(scaled_ang_ts, [], color='blue', linewidth=0.5)
    ang_y_subplot.set_xlabel('Flight Time [s]')
    ang_y_subplot.set_ylabel('Degrees')
    ang_y_subplot.set_title('Yaw')
    ang_y_subplot.set_ylim(0, 400)

    def update(frame):
        with node.lock:
            if node.ang_data is None or node.mag_data is None:
                return x_line, y_line, z_line, r_line, p_line, yaw_line
            scaled_mag_ts = node.mag_timestamps / 1000000
            scaled_ang_ts = node.ang_timestamps / 1000000
            x_data = node.mag_data[:, 0]
            y_data = node.mag_data[:, 1]
            z_data = node.mag_data[:, 2]
            r_data = node.ang_data[:, 0]
            p_data = node.ang_data[:, 1]
            yaw_data = node.ang_data[:, 2]
        yaw_data[yaw_data < 0] += 360

        x_line.set_data(scaled_mag_ts, x_data)
        y_line.set_data(scaled_mag_ts, y_data)
        z_line.set_data(scaled_mag_ts, z_data)
        mag_subplot.set_xlim(scaled_mag_ts[0], scaled_mag_ts[-1])

        r_line.set_data(scaled_ang_ts, r_data)
        p_line.set_data(scaled_ang_ts, p_data)
        ang_rp_subplot.set_xlim(scaled_ang_ts[0], scaled_ang_ts[-1])

        yaw_line.set_data(scaled_ang_ts, yaw_data)
        ang_y_subplot.set_xlim(scaled_ang_ts[0], scaled_ang_ts[-1])

        return x_line, y_line, z_line, r_line, p_line, yaw_line


    ani = FuncAnimation(fig, update, interval=500, blit=False)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()