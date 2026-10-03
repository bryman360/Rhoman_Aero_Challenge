import rclpy
from rclpy.node import Node
from challenge_interfaces.msg import AngTimestamped, MagTimestamped, AziEleTimestamped

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
        self.estimator_subscription = self.create_subscription(AziEleTimestamped, "azi_ele_ts", self.estimator_subscription_callback, 10)

        self.mag_timestamps = np.array([], dtype=np.uint32)
        self.ang_timestamps = np.array([], dtype=np.uint32)
        self.mag_data = None
        self.ang_data = None

        self.azi_ele_timestamps = np.array([], dtype=np.uint32)
        self.azi_avg_data = np.array([], dtype=np.float64)
        self.azi_inst_data = np.array([], dtype=np.float64)
        self.ele_avg_data = np.array([], dtype=np.float64)
        self.ele_inst_data = np.array([], dtype=np.float64)

        self.output_dir = str(self.get_parameter("output_dir").value)
        self.lock = threading.Lock()

        with open(self.output_dir + "/mag_values.csv", "w", newline="", encoding="utf-8") as mag_file:
            csv_writer = csv.writer(mag_file)
            header_line = ["Time", "Mag X", "Mag Y", "Mag Z"]
            csv_writer.writerow(header_line)

        with open(self.output_dir + "/attitude_converted_to_rollpitchyaw.csv", "w", newline="", encoding="utf-8") as ang_file:
            csv_writer = csv.writer(ang_file)
            header_line = ["Time", "Roll Deg", "Pitch Deg", "Yaw Deg"]
            csv_writer.writerow(header_line)

        with open(self.output_dir + "/azimuth_average_and_instant_values.csv", "w", newline="", encoding="utf-8") as azi_file:
            csv_writer = csv.writer(azi_file)
            header_line = ["Time", "Azimuth Avg Deg", "Azimuth Instant Deg"]
            csv_writer.writerow(header_line)

        with open(self.output_dir + "/elevation_average_and_instant_values.csv", "w", newline="", encoding="utf-8") as ele_file:
            csv_writer = csv.writer(ele_file)
            header_line = ["Time", "Elevation Avg Deg", "Elevation Instant Deg"]
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

    def estimator_subscription_callback(self, msg: AziEleTimestamped):

        with open(self.output_dir + "/azimuth_average_and_instant_values.csv", "a", newline="", encoding="utf-8") as azi_file:
            csv_writer = csv.writer(azi_file)
            csv_writer.writerow([msg.timestamp, msg.average_azimuth, msg.instant_azimuth])
        with open(self.output_dir + "/elevation_average_and_instant_values.csv", "a", newline="", encoding="utf-8") as ele_file:
            csv_writer = csv.writer(ele_file)
            csv_writer.writerow([msg.timestamp, msg.average_elevation, msg.instant_elevation])
        with self.lock:
            self.azi_ele_timestamps = np.append(self.azi_ele_timestamps, msg.timestamp)
            self.azi_avg_data = np.append(self.azi_avg_data, msg.average_azimuth)
            self.azi_inst_data = np.append(self.azi_inst_data, msg.instant_azimuth)
            self.ele_avg_data = np.append(self.ele_avg_data, msg.average_elevation)
            self.ele_inst_data = np.append(self.ele_inst_data, msg.instant_elevation)
        



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

    azi_inst_pts = azi_subplot.scatter([], [], color='green')
    azi_avg_pts = azi_subplot.scatter([], [], color='black')
    azi_subplot.set_xlabel('Flight Time [s]')
    azi_subplot.set_ylabel('Degrees East')
    azi_subplot.set_title('Estimated Instantaneous/Average Magnetic Field Azimuth (green/black)')
    azi_subplot.set_ylim(0, 100)
    
    ele_inst_pts = ele_subplot.scatter([], [], color='green')
    ele_avg_pts = ele_subplot.scatter([], [], color='black')
    ele_subplot.set_xlabel('Flight Time [s]')
    ele_subplot.set_ylabel('Degrees Down')
    ele_subplot.set_title('Estimated Instantaneous/Average Magnetic Field Elevation (green/black)')
    ele_subplot.set_ylim(54, 70)

    def update(frame):
        update_azi_and_ele = False
        with node.lock:
            if node.ang_data is None or node.mag_data is None:
                return x_line, y_line, z_line, r_line, p_line, yaw_line, azi_inst_pts, azi_avg_pts, ele_inst_pts, ele_avg_pts
            scaled_mag_ts = node.mag_timestamps / 1000000
            scaled_ang_ts = node.ang_timestamps / 1000000
            x_data = node.mag_data[:, 0]
            y_data = node.mag_data[:, 1]
            z_data = node.mag_data[:, 2]
            r_data = node.ang_data[:, 0]
            p_data = node.ang_data[:, 1]
            yaw_data = node.ang_data[:, 2]
            if len(node.azi_ele_timestamps) > 1:
                update_azi_and_ele = True
                azi_ele_scaled_ts = node.azi_ele_timestamps / 1000000
                azi_avg_data = node.azi_avg_data
                azi_inst_data = node.azi_inst_data
                ele_avg_data = node.ele_avg_data
                ele_inst_data = node.ele_inst_data
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

        if update_azi_and_ele:
            azi_avg_packed_pts = np.vstack((azi_ele_scaled_ts, azi_avg_data))
            azi_inst_packed_pts = np.vstack((azi_ele_scaled_ts, azi_inst_data))
            ele_avg_packed_pts = np.vstack((azi_ele_scaled_ts, ele_avg_data))
            ele_inst_packed_pts = np.vstack((azi_ele_scaled_ts, ele_inst_data))
            azi_avg_pts.set_offsets(azi_avg_packed_pts.T)
            azi_inst_pts.set_offsets(azi_inst_packed_pts.T)
            azi_subplot.set_xlim(azi_ele_scaled_ts[0] - 50, azi_ele_scaled_ts[-1])
            azi_subplot.set_ylim(min(azi_avg_data) - 10, max(azi_avg_data) + 10)
            ele_avg_pts.set_offsets(ele_avg_packed_pts.T)
            ele_inst_pts.set_offsets(ele_inst_packed_pts.T)
            ele_subplot.set_xlim(azi_ele_scaled_ts[0] - 50, azi_ele_scaled_ts[-1])
            ele_subplot.set_ylim(min(ele_avg_data) - 10, max(ele_inst_data) + 10)

        return x_line, y_line, z_line, r_line, p_line, yaw_line, azi_inst_pts, azi_avg_pts, ele_inst_pts, ele_avg_pts


    ani = FuncAnimation(fig, update, interval=1000, blit=False)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()