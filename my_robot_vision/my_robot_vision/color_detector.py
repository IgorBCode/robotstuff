import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from nav_msgs.msg import Odometry

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.active_movement = False

    def image_callback(self, msg):
        if self.active_movement:
            return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red color range in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        

        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # detection
        if cv2.countNonZero(red_mask) > 5000:
            self.get_logger().info('Red detected')
            # self.move_forward()
            # self.turn_right()
            self.start_movement()
        elif cv2.countNonZero(blue_mask) > 5000:
            self.get_logger().info('Blue detected')
            self.start_movement()
            # self.move_forward()
            # self.turn_right()
        # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # mask = cv2.bitwise_or(mask1, mask2)


    # movement functions
    def start_movement(self):
        self.active_movement = True

        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        self.cmd_pub.publish(twist_msg)
        self.create_timer(2.0, self.stop_movement)
    
    def stop_movement(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.cmd_pub.publish(twist_msg)

        # turn
        twist_msg.angular.z = -1.57
        self.cmd_pub.publish(twist_msg)
        self.create_timer(2.0, self.finish_movement)
    
    def finish_movement(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        self.cmd_pub.publish(twist_msg)
        self.active_movement = False

    # def move_forward(self):
    #     twist_msg = Twist()
    #     twist_msg.linear.x = 0.5
    #     self.cmd_pub.publish(twist_msg)
    #     # movement timer
    #     time.sleep(2)
    #     twist_msg.linear.x = 0.0
    #     self.cmd_pub.publish(twist_msg)

    # def turn_right(self):
    #     twist_msg = Twist()
    #     twist_msg.angular.z = -0.785 # turn 90 degrees right
    #     self.cmd_pub.publish(twist_msg)
    #     twist_msg.angular.z = 0.0
    #     self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
