import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

FORWARD_DISTANCE = 3.0
LINEAR_SPEED = 1.0
ANGULAR_SPEED = 0.4
TURN_ANGLE = math.pi / 2
ANGLE_TOLERANCE = 0.005
DIST_TOLERANCE = 0.05

class ColorMove(Node):
    def __init__(self):
        super().__init__('color_move_node')
        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pose = None
        self.yaw = None

        self.state = 'WAITING'
        self.color = None
        self.start_pos = None
        self.target_yaw = None

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Node initialized. Waiting for color...")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.yaw = self.quaternion_to_yaw(self.pose.orientation)

    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def image_callback(self, msg):
        if self.state != 'WAITING':
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) | \
                   cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
        blue_mask = cv2.inRange(hsv, (100, 150, 0), (140, 255, 255))
        green_mask = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255)) 

        red = cv2.countNonZero(red_mask) > 0
        blue = cv2.countNonZero(blue_mask) > 0
        green = cv2.countNonZero(green_mask) > 0

        if green:
            if self.state != 'FOLLOWING_GREEN':
                self.get_logger().info("Green detected moving forward")
            self.state = 'FOLLOWING_GREEN'

        # stop robot if it sees a different color
        if self.state == 'FOLLOWING_GREEN' and not green:
            self.get_logger().info("Green lost — stopping")
            self.state = 'WAITING'
            self.cmd_pub.publish(Twist())
            return

        if red or blue:
            if red:
                self.color = 'red'
                self.get_logger().info("Red detected. Will move and turn RIGHT.")
            elif blue:
                self.color = 'blue'
                self.get_logger().info("Blue detected. Will move and turn LEFT.")

            self.state = 'WAITING_FOR_ODOM'

    def control_loop(self):
        if self.state == 'FOLLOWING_GREEN':
            twist = Twist()
            twist.linear.x = LINEAR_SPEED * 1.5
            self.cmd_pub.publish(twist)
            return

        if self.state == 'WAITING_FOR_ODOM':
            if self.pose is None or self.yaw is None:
                if not hasattr(self, '_printed_odom_waiting'):
                    self.get_logger().info("Waiting for odometry...")
                    self._printed_odom_waiting = True
                return
            self.start_pos = self.pose.position
            self.state = 'MOVING'
            self.get_logger().info("Starting forward movement.")

        elif self.state == 'MOVING':
            dist = self.distance(self.pose.position, self.start_pos)
            if dist < FORWARD_DISTANCE - DIST_TOLERANCE:
                twist = Twist()
                twist.linear.x = LINEAR_SPEED
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Forward movement complete.")
                dir_mult = -1 if self.color == 'red' else 1
                self.target_yaw = self.normalize_angle(self.yaw + dir_mult * TURN_ANGLE)
                self.state = 'TURNING'

        elif self.state == 'TURNING':
            angle_error = self.normalize_angle(self.target_yaw - self.yaw)
            if abs(angle_error) > ANGLE_TOLERANCE:
                twist = Twist()
                twist.angular.z = ANGULAR_SPEED if angle_error > 0 else -ANGULAR_SPEED
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Turn complete. Robot done.")
                self.state = 'WAITING'

        elif self.state == 'DONE':
            self.cmd_pub.publish(Twist())

    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = ColorMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
