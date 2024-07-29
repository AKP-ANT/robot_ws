import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.timer import Timer

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.linear_speed = 0.5
        self.angular_speed = 0.0
        self.publish_timer = self.create_timer(0.2, self.publish_cmd_vel)


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.follow_line(frame)

    def follow_line(self, frame):
        height, width = frame.shape[:2]
        cropped_frame = frame[int(height * 0.5):, :]
        
        gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        M = cv2.moments(binary, False)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(cropped_frame, (cx, cy), 5, (255, 0, 0), -1)
            
            error = cx - width / 2
            self.angular_speed = round(-error * 0.002, 3)
            # self.publish_cmd_vel()

        cv2.imshow("binary", binary)
        cv2.imshow("Frame", cropped_frame)
        cv2.waitKey(1)

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
