#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from datetime import datetime
import os
import csv
os.chdir('/home/dronepi/Master_thesis/pi_setup')


from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import Vibration
from geographic_msgs.msg import GeoPoseStamped

class Logger(Node):
    def __init__(self):
        super().__init__('mavros_logger')

        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = f'logs/{now}'
        os.makedirs(self.log_dir, exist_ok=True)

        self.files = {}
        self.writers = {}
        self.get_logger().info(f"MAVROS Logger started. Logging to: {self.log_dir}")
        def setup_csv(topic, headers):
            f = open(os.path.join(self.log_dir, topic.replace('/', '_') + '.csv'), 'w', newline='')
            writer = csv.writer(f)
            writer.writerow(['timestamp'] + headers)
            # this will make sure to save the logs all time in case of powerout
            f.flush()
            os.fsync(f.fileno())
            self.files[topic] = f
            self.writers[topic] = writer

        setup_csv('/mavros/state', ['mode', 'armed'])
        setup_csv('/mavros/local_position/pose', ['x', 'y', 'z'])
        setup_csv('/mavros/local_position/velocity_local', ['vx', 'vy', 'vz'])
        setup_csv('/mavros/global_position/global', ['lat', 'lon', 'alt'])
        setup_csv('/mavros/global_position/raw/fix', ['lat', 'lon', 'alt'])
        setup_csv('/mavros/global_position/rel_alt', ['rel_alt'])
        setup_csv('/mavros/imu/data', ['ax', 'ay', 'az'])
        setup_csv('/mavros/imu/data_raw', ['raw_ax', 'raw_ay', 'raw_az'])
        setup_csv('/mavros/battery', ['voltage', 'current', 'percentage'])
        setup_csv('/mavros/altitude', ['altitude'])
        setup_csv('/mavros/vibration/raw/vibration', ['vibe_x', 'vibe_y', 'vibe_z'])

        # QoS profiles
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.create_subscription(State, '/mavros/state', self.state_cb, reliable)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, best_effort)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity_cb, best_effort)
        self.create_subscription(GeoPoseStamped, '/mavros/global_position/global', self.global_cb, best_effort)
        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self.gps_cb, best_effort)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.simple_cb('/mavros/global_position/rel_alt'), best_effort)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, best_effort)
        self.create_subscription(Imu, '/mavros/imu/data_raw', self.imu_raw_cb, best_effort)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_cb, best_effort)
        self.create_subscription(Float64, '/mavros/altitude', self.simple_cb('/mavros/altitude'), best_effort)
        self.create_subscription(Vibration, '/mavros/vibration/raw/vibration', self.vibe_cb, best_effort)

    def get_timestamp(self):
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    def state_cb(self, msg):
        self.writers['/mavros/state'].writerow([self.get_timestamp(), msg.mode, msg.armed])

    def pose_cb(self, msg):
        p = msg.pose.position
        self.writers['/mavros/local_position/pose'].writerow([self.get_timestamp(), p.x, p.y, p.z])

    def velocity_cb(self, msg):
        v = msg.twist.linear
        self.writers['/mavros/local_position/velocity_local'].writerow([self.get_timestamp(), v.x, v.y, v.z])

    def global_cb(self, msg):
        p = msg.pose.position
        self.writers['/mavros/global_position/global'].writerow([self.get_timestamp(), p.latitude, p.longitude, p.altitude])

    def gps_cb(self, msg):
        self.writers['/mavros/global_position/raw/fix'].writerow([self.get_timestamp(), msg.latitude, msg.longitude, msg.altitude])

    def imu_cb(self, msg):
        a = msg.linear_acceleration
        self.writers['/mavros/imu/data'].writerow([self.get_timestamp(), a.x, a.y, a.z])

    def imu_raw_cb(self, msg):
        a = msg.linear_acceleration
        self.writers['/mavros/imu/data_raw'].writerow([self.get_timestamp(), a.x, a.y, a.z])

    def battery_cb(self, msg):
        self.writers['/mavros/battery'].writerow([self.get_timestamp(), msg.voltage, msg.current, msg.percentage])

    def vibe_cb(self, msg):
        v = msg.vibration
        self.writers['/mavros/vibration/raw/vibration'].writerow([self.get_timestamp(), v.x, v.y, v.z])

    def simple_cb(self, topic):
        def inner(msg):
            self.writers[topic].writerow([self.get_timestamp(), msg.data])
        return inner

    def destroy_node(self):
        for f in self.files.values():
            f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
