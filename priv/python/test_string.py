import sys
import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE,SIG_DFL)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'test',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        sys.stdout.write(f"I heard: {msg.data}")
        sys.stdout.flush()

def main(args=None):
    try:
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    except:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
