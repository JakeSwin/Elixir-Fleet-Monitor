import sys
import time
import rclpy
import cv2
import base64
import time

from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState

from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE, SIG_DFL)

class FleetStateReader(Node):

    def __init__(self):
        super().__init__('elixir_fleet_state_reader')

        self.subscription = self.create_subscription(
            FleetState,
            "/fleet_states",
            self.listener_callback,
            1
        )
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.fleet_data = None
        self.subscription

    def listener_callback(self, msg:FleetState):
        data = {
            "name": msg.name,
            "robots": [{
                "name": i.name, 
                "task_id": i.task_id,
                "seq": i.seq,
                "mode": {
                    "mode": i.mode.mode,
                    "mode_request_id": i.mode.mode_request_id
                },
                "battery_percent": i.battery_percent,
                "location": {
                    "x": i.location.x,
                    "y": i.location.y,
                    "yaw": i.location.yaw
                },
                "path": [{
                    "x": i2.x,
                    "y": i2.y,
                    "yaw": i2.yaw
                } for i2 in i.path]
            } for i in msg.robots]
        }
        self.fleet_data = data

    def timer_callback(self):
        if self.fleet_data != None:
            sys.stdout.write(f"{self.fleet_data}")
            sys.stdout.flush()

def main(args=None):
    try: 
        rclpy.init(args=args)

        fleet_state_reader = FleetStateReader()

        rclpy.spin(fleet_state_reader)

        fleet_state_reader.destroy_node()
        rclpy.shutdown()
    except:
        fleet_state_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
