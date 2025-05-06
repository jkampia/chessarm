import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String

from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped

import cv2
from cv_bridge import CvBridge 

from .kinematic_helper import ARM_5DOF

import struct

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .user_input import command_list


class MainArmNode(Node):


    def __init__(self):    

        super().__init__('main_arm_node')
        self.timer = self.create_timer(0.1, self.timerCallback)
        self.get_logger().info("Main control node initialized")

        self.robot = ARM_5DOF([100, 400, 400, 100, 100], [0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_pub = self.create_publisher(PointCloud2, '/chessarm/out/joint_coordinates', 10)
        self.link_pub = self.create_publisher(Path, '/chessarm/out/link_coordinates', 10)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publishStaticTF()


    def timerCallback(self):

        
        self.parseUserInput(self.getUserInput())

        self.robot.solveFK()
        self.publishJointsToRviz()


    def publishJointsToRviz(self):
        
        link_msg = Path()
        joint_msg = PointCloud2()

        link_msg.header.frame_id = "map"
        link_msg.header.stamp = self.get_clock().now().to_msg()
        link_msg.poses = [self.toPoseStamped(joint) for joint in self.robot.joint_coordinates_m]

        joint_msg = self.toPC2(self.robot.joint_coordinates_m, self.robot.joint_colors)

        self.joint_pub.publish(joint_msg)
        self.link_pub.publish(link_msg)


    def packRGB(self, r, g, b):
        return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]


    def toPC2(self, points, colors):

        # restructure points to be x,y,z,rgb (packed)
        for i in range(len(points)):
            print(colors[i]) # [color name, (r,g,b)]
            r, g, b = colors[i]
            points[i] = (points[i][0], points[i][1], points[i][2], self.packRGB(r, g, b))

        header = Header()
        header.stamp = self.get_clock().now().to_msg() 
        header.frame_id = 'map'

        cloud = pc2.create_cloud(
            header,
            fields=[
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            points=points,
        )

        return cloud
    

    def toPoseStamped(self, tuple):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tuple[0]
        pose.pose.position.y = tuple[1]
        pose.pose.position.z = tuple[2]
        return pose
    

    def publishStaticTF(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform: map -> map')

    def getUserInput(self): 
        return input("Enter a command: ")

    def parseUserInput(self, user_input):
        if user_input.startswith("help"):
            for command in command_list:
                print(command)
        elif user_input.startswith("ik"):
            pass
        elif user_input.startswith("fk"):
            pass



def main(args=None):
    rclpy.init(args=args)
    node = MainArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
