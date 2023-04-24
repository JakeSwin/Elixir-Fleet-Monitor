import sys
import time
import rclpy
import cv2
import base64
import time

from rclpy.node import Node
from rclpy.time import Time
from rmf_task_msgs.msg import Tasks

from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE, SIG_DFL)

class OngoingTaskReader(Node):

    def __init__(self):
        super().__init__('elixir_ongoing_task_reader')

        self.create_subscription(
            Tasks,
            "/dispatcher_ongoing_tasks",
            self.listener_callback,
            1
        )
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.task_data = None

    def listener_callback(self, msg):
        data = [{
            "fleet_name": task.fleet_name,
            "task_id": task.task_id,
            "state": task.state,
            "status": task.status,
            "submission_time": task.submission_time.sec,
            "start_time": task.start_time.sec,
            "current_time": self.get_clock().now().seconds_nanoseconds()[0],
            "end_time": task.end_time.sec,
            "robot_name": task.robot_name
        } for task in msg.tasks]
        self.task_data = data

    def timer_callback(self):
        if self.task_data != None:
            sys.stdout.write(f"{self.task_data}")
            sys.stdout.flush()

def main(args=None):
    try: 
        rclpy.init(args=args)

        ongoing_task_reader = OngoingTaskReader()
        
        rclpy.spin(ongoing_task_reader)

        ongoing_task_reader.destroy_node()
        rclpy.shutdown()
    except:
        ongoing_task_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
