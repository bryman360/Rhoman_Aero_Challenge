import rclpy
from rclpy.node import Node
import time
import numpy as np
from pyulog.core import ULog
from challenge_interfaces.msg import AngTimestamped, MagTimestamped

def get_mag_and_att_values_from_ulog(ulog_file_path: str):
    ulog = ULog(ulog_file_path)
    data_list = ulog.data_list
    mag_data = None
    att_data = None
    mag_data_found = False
    att_data_found = False

    for data in data_list:
        if data.name == "sensor_mag" and not mag_data_found:
            mag_data_found = True
            mag_data = data.data
        if data.name == "vehicle_attitude" and not att_data_found:
            att_data_found = True
            att_data = data.data

        if mag_data_found and att_data_found:
            break

    return mag_data, att_data

def convert_quaternion_to_rpy(w: np.float64, x: np.float64, y: np.float64, z: np.float64):
    roll  = (180 / np.pi) * (np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2)))
    pitch = (180 / np.pi) * ( np.arcsin(2 * (w * y - z * x)))
    yaw   = (180 / np.pi) * (np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2)))
    return roll, pitch, yaw


class MagAngPublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.ang_publisher = self.create_publisher(AngTimestamped, "vehicle_ang_ts", 10)
        self.mag_publisher = self.create_publisher(MagTimestamped, "sensor_mag_ts", 10)
        self.mag_data, self.att_data = get_mag_and_att_values_from_ulog('/home/bryman360/Downloads/04_13_03.ulg')
        self.get_logger().info("Publisher node ready.")
        self.spin()


    def spin(self):
        if not self.mag_data and not self.att_data:
            self.get_logger().error("No Magnetometer Data or Attitude data loaded in. Exiting.")
            return

        mag_i = 0
        att_i = 0

        mag_ts = np.array(self.mag_data['timestamp'], dtype=np.uint64) * 1000
        mag_x = self.mag_data['x']
        mag_y = self.mag_data['y']
        mag_z = self.mag_data['z']

        att_ts = np.array(self.att_data['timestamp'], dtype=np.uint64) * 1000
        att_q0 = self.att_data['q[0]']
        att_q1 = self.att_data['q[1]']
        att_q2 = self.att_data['q[2]']
        att_q3 = self.att_data['q[3]']

        self.get_logger().info("About to begin publishing data")
        
        sim_time_ns = min(att_ts[0], mag_ts[0]) - 1000000000
        last_real_time_ns = time.time_ns()

        self.get_logger().info(f"Starting at {sim_time_ns}, first should be at {min(att_ts[0], mag_ts[0])}")
        while att_i < len(att_ts) or mag_i < len(mag_ts):
            current_real_time_ns = time.time_ns()
            delta_ns = current_real_time_ns - last_real_time_ns
            sim_time_ns += delta_ns

            while att_i < len(att_ts) and att_ts[att_i] <= sim_time_ns:
                msg = AngTimestamped()
                roll, pitch, yaw = convert_quaternion_to_rpy(att_q0[att_i], att_q1[att_i], att_q2[att_i], att_q3[att_i])
                msg.timestamp = int(att_ts[att_i] / 1000)
                msg.roll = float(roll)
                msg.pitch = float(pitch)
                msg.yaw = float(yaw)
                self.ang_publisher.publish(msg)
                att_i += 1

            while mag_i < len(mag_ts) and mag_ts[mag_i] <= sim_time_ns:
                msg = MagTimestamped()
                msg.timestamp = int(mag_ts[mag_i] / 1000)
                msg.x = float(mag_x[mag_i])
                msg.y = float(mag_y[mag_i])
                msg.z = float(mag_z[mag_i])
                self.mag_publisher.publish(msg)
                mag_i += 1

            last_real_time_ns = current_real_time_ns


def main(args=None):
    rclpy.init(args=args)
    node = MagAngPublisherNode("mag_ang_publisher")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()