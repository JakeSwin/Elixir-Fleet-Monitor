import sys
import time
import rclpy
import cv2
import base64
import time

from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE, SIG_DFL)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        # '/robot1/camera/rgb/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_color',
            self.listener_callback,
            1
        )
        self.subscription

    def listener_callback(self, msg):
        start = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            sys.stderr.write(f"{e}")
        jpg_img = cv2.imencode(".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        b64_string = base64.b64encode(jpg_img[1]).decode("utf-8")
        end = time.time()
        # sys.stdout.write(f"height {msg.height} width {msg.width} time {end - start} data {b64_string}")
        # sys.stdout.write(f"height {msg.height} width {msg.width}")
        # sys.stdout.flush()
        # sys.stdout.write(f"time {end - start}")
        # sys.stdout.flush()
        sys.stdout.write(f"data {b64_string}")
        sys.stdout.flush()

    def test_callback(self, msg):
        # start = time.time()
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # except CvBridgeError as e:
        #     sys.stderr.write(f"{e}")
        # jpg_img = cv2.imencode(".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        # b64_string = base64.b64encode(jpg_img[1]).decode("utf-8")
        # end = time.time()
        # sys.stdout.write(f"height {msg.height} width {msg.width} time {end - start} data {b64_string}")
        sys.stdout.write(f"Got Image")
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
