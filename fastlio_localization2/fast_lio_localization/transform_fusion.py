#!/usr/bin/env python3

import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import rclpy.timer
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.declare_parameter("odom_topic", "/nav_odom")
        self.declare_parameter("map_to_odom_topic", "/map_to_odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.cb_save_cur_odom,
            1,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("map_to_odom_topic").value,
            self.cb_save_map_to_odom,
            1,
        )

        self.freq_pub_localization = 50
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)
        # threading.Thread(target=self.transform_fusion, daemon=True).start()

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        trans[:3, :3] = Rotation.from_quat(quat).as_matrix()
        return trans

    def mat_to_quat(self, transform):
        return Rotation.from_matrix(np.array(transform[:3, :3], dtype=np.float64, copy=True)).as_quat()

    def transform_fusion(self):
        if self.cur_odom_to_baselink is None:
            return

        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        transform_msg = Transform()
        transform_msg.translation.x = T_map_to_odom[0, 3]
        transform_msg.translation.y = T_map_to_odom[1, 3]
        transform_msg.translation.z = T_map_to_odom[2, 3]
        
        quat = self.mat_to_quat(T_map_to_odom)

        transform_msg.rotation.x = quat[0]
        transform_msg.rotation.y = quat[1]
        transform_msg.rotation.z = quat[2]
        transform_msg.rotation.w = quat[3]
        
        header = Header()
        header.stamp = self.cur_odom_to_baselink.header.stamp
        header.frame_id = self.get_parameter("map_frame").value
        
        transform_stamped_msg = TransformStamped(
                header = header,
                child_frame_id = self.get_parameter("odom_frame").value,
                transform = transform_msg
            )
        if self.get_parameter("publish_tf").value:
            self.tf_broadcaster.sendTransform(transform_stamped_msg)

        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

            xyz = T_map_to_base_link[:3, 3]
            quat = self.mat_to_quat(T_map_to_base_link)

            localization = Odometry()
            localization.pose.pose = Pose(
                position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
                orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
            )
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = self.get_parameter("map_frame").value
            localization.child_frame_id = self.get_parameter("base_frame").value
            self.pub_localization.publish(localization)


    def cb_save_cur_odom(self, msg):
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
