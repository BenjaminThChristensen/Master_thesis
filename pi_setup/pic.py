#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
import subprocess
import time
import threading

# Settings
GPIO_CHANNEL_INDEX = 5  # chan6_raw = index 5 (zero-based)
TRIGGER_THRESHOLD = 1500  # Switch ON when value > 1500
CAPTURE_INTERVAL = 4  # seconds
PHOTO_DIR = "/home/dronepi/photos"

class PhotoTriggerNode(Node):
    def __init__(self):
        super().__init__('photo_trigger_node')
        self.subscription = self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            10
        )
        self.photo_loop_running = False
        self.photo_thread = None
        self.last_state = False

    def rc_callback(self, msg):
        try:
            chan_val = msg.channels[GPIO_CHANNEL_INDEX]
            switch_on = chan_val > TRIGGER_THRESHOLD
        except IndexError:
            self.get_logger().warn("RC channel index out of range")
            return

        if switch_on and not self.photo_loop_running:
            self.get_logger().info("Switch ON – starting photo loop")
            self.photo_loop_running = True
            self.photo_thread = threading.Thread(target=self.photo_loop)
            self.photo_thread.start()

        elif not switch_on and self.photo_loop_running:
            self.get_logger().info("Switch OFF – stopping photo loop")
            self.photo_loop_running = False
            if self.photo_thread:
                self.photo_thread.join()

    def photo_loop(self):
        while self.photo_loop_running:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{PHOTO_DIR}/photo_{timestamp}.jpg"
            self.get_logger().info(f"Capturing image: {filename}")
            try:
                subprocess.run(['libcamera-jpeg', '-o', filename], check=True)
            except subprocess.CalledProcessError:
                self.get_logger().error("Photo capture failed")
            time.sleep(CAPTURE_INTERVAL)

    def destroy_node(self):
        if self.photo_thread and self.photo_thread.is_alive():
            self.photo_loop_running = False
            self.photo_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhotoTriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
