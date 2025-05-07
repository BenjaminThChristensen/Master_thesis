#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from datetime import datetime
import os
import csv
import fcntl

os.chdir('/home/dronepi/Master_thesis/pi_setup')


from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import Vibration
from geographic_msgs.msg import GeoPoseStamped

class SafeCSV:
    def __init__(self, path, headers):
        # Open with O_SYNC so writes go directly to storage
        flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_SYNC
        fd = os.open(path, flags, 0o644)
        self.f = os.fdopen(fd, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['timestamp'] + headers)
        # One more sync after headers
        self.f.flush()
        os.fsync(self.f.fileno())

    def writerow(self, row):
        self.writer.writerow(row)
        # flush Python buffer
        self.f.flush()
        # flush OS buffer
        os.fsync(self.f.fileno())

    def close(self):
        try:
            self.f.flush()
            os.fsync(self.f.fileno())
        except Exception:
            pass
        self.f.close()


class Logger(Node):
    def __init__(self):
        super().__init__('mavros_logger')
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = f'logs/{now}'
        os.makedirs(self.log_dir, exist_ok=True)
        self.get_logger().info(f"MAVROS Logger started. Logging to: {self.log_dir}")

        # Prepare our CSV loggers
        def setup_csv(topic, headers):
            safe = SafeCSV(
                os.path.join(self.log_dir, topic.replace('/', '_') + '.csv'),
                headers
            )
            self.files[topic] = safe
            return safe

        self.files = {}
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        rel  = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Create one SafeCSV per topic
        self.logger_state   = setup_csv('/mavros/state', ['mode', 'armed'])
        self.logger_pose    = setup_csv('/mavros/local_position/pose', ['x', 'y', 'z'])
        self.logger_vel     = setup_csv('/mavros/local_position/velocity_local', ['vx', 'vy', 'vz'])
        self.logger_gpos    = setup_csv('/mavros/global_position/global', ['lat', 'lon', 'alt'])
        self.logger_fix     = setup_csv('/mavros/global_position/raw/fix', ['lat', 'lon', 'alt'])
        self.logger_relalt  = setup_csv('/mavros/global_position/rel_alt', ['rel_alt'])
        self.logger_imu     = setup_csv('/mavros/imu/data', ['ax', 'ay', 'az'])
        self.logger_imu_raw = setup_csv('/mavros/imu/data_raw', ['raw_ax', 'raw_ay', 'raw_az'])
        self.logger_batt    = setup_csv('/mavros/battery', ['voltage', 'current', 'percentage'])
        self.logger_alt     = setup_csv('/mavros/altitude', ['altitude'])
        self.logger_vibe    = setup_csv('/mavros/vibration/raw/vibration', ['vibe_x', 'vibe_y', 'vibe_z'])

        # Subscriptions
        self.create_subscription(State,   '/mavros/state',                        self.state_cb,  rel)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',      self.pose_cb,  best)
        self.create_subscription(TwistStamped,'/mavros/local_position/velocity_local', self.velocity_cb, best)
        self.create_subscription(GeoPoseStamped,'/mavros/global_position/global',self.global_cb, best)
        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix',     self.gps_cb,   best)
        self.create_subscription(Float64,   '/mavros/global_position/rel_alt',    self.simple_cb(self.logger_relalt), best)
        self.create_subscription(Imu,       '/mavros/imu/data',                   self.imu_cb,    best)
        self.create_subscription(Imu,       '/mavros/imu/data_raw',               self.imu_raw_cb,best)
        self.create_subscription(BatteryState,'/mavros/battery',                 self.battery_cb,best)
        self.create_subscription(Float64,   '/mavros/altitude',                   self.simple_cb(self.logger_alt), best)
        self.create_subscription(Vibration, '/mavros/vibration/raw/vibration',    self.vibe_cb,   best)

    def get_timestamp(self):
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    def state_cb(self, msg):
        self.logger_state.writerow([self.get_timestamp(), msg.mode, msg.armed])

    def pose_cb(self, msg):
        p = msg.pose.position
        self.logger_pose.writerow([self.get_timestamp(), p.x, p.y, p.z])

    def velocity_cb(self, msg):
        v = msg.twist.linear
        self.logger_vel.writerow([self.get_timestamp(), v.x, v.y, v.z])

    def global_cb(self, msg):
        p = msg.pose.position
        self.logger_gpos.writerow([self.get_timestamp(), p.latitude, p.longitude, p.altitude])

    def gps_cb(self, msg):
        self.logger_fix.writerow([self.get_timestamp(), msg.latitude, msg.longitude, msg.altitude])

    def imu_cb(self, msg):
        a = msg.linear_acceleration
        self.logger_imu.writerow([self.get_timestamp(), a.x, a.y, a.z])

    def imu_raw_cb(self, msg):
        a = msg.linear_acceleration
        self.logger_imu_raw.writerow([self.get_timestamp(), a.x, a.y, a.z])

    def battery_cb(self, msg):
        self.logger_batt.writerow([self.get_timestamp(), msg.voltage, msg.current, msg.percentage])

    def vibe_cb(self, msg):
        v = msg.vibration
        self.logger_vibe.writerow([self.get_timestamp(), v.x, v.y, v.z])

    def simple_cb(self, logger):
        def inner(msg):
            logger.writerow([self.get_timestamp(), msg.data])
        return inner

    def destroy_node(self):
        # close every SafeCSV
        for log in self.files.values():
            log.close()
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
