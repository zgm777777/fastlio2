#!/usr/bin/env python3

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.initialized = False

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("scan_topic", "/cloud_registered_aligned"),
                ("odom_topic", "/nav_odom"),
                ("initialpose_topic", "/initialpose"),
                ("map_frame", "map"),
                ("odom_frame", "odom"),
                ("base_frame", "base_link"),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        self.get_logger().info("Waiting for global map...")
        # global_map_msg = wait_for_message(msg_type = PointCloud2, node = self, topic = "/cloud_pcd")[1]
        # self.initialize_global_map(global_map_msg)
        
        self.initialize_global_map()
        self.get_logger().info("Global map received.")
        
        self.create_subscription(
            PointCloud2,
            self.get_parameter("scan_topic").value,
            self.cb_save_cur_scan,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.cb_save_cur_odom,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initialpose_topic").value,
            self.cb_initialpose,
            10,
        )

        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)

    def global_map_callback(self):
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("map_frame").value
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        
    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = Rotation.from_quat(quat).as_matrix()
        return trans

    def mat_to_quat(self, transform):
        return Rotation.from_matrix(np.array(transform[:3, :3], dtype=np.float64, copy=True)).as_quat()
    
    def msg_to_array(self, pc_msg):
        points = point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
        if points.dtype.names:
            return np.column_stack([points["x"], points["y"], points["z"]]).astype(np.float32)
        return np.asarray(points, dtype=np.float32).reshape(-1, 3)
    
    def registration_at_scale(self, scan, map, initial, scale, use_point_to_plane=False):
        scan_down = self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale)
        map_down = self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale)

        # 精配准阶段可选 point-to-plane（仅在初始对齐时使用，持续跟踪用 point-to-point 省CPU）
        if use_point_to_plane and len(scan_down.points) > 100:
            try:
                map_down.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=1.0 * scale, max_nn=30)
                )
                result_icp = o3d.pipelines.registration.registration_icp(
                    scan_down, map_down,
                    1.0 * scale,
                    initial,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
                )
                return result_icp.transformation, result_icp.fitness
            except Exception:
                pass  # fall through to point-to-point

        result_icp = o3d.pipelines.registration.registration_icp(
            scan_down, map_down,
            1.0 * scale,
            initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc):
        pc = np.asarray(pc, dtype=np.float32)
        if pc.size == 0:
            pc = pc.reshape(0, 3)

        if pc.shape[1] >= 4:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg = point_cloud2.create_cloud(header, fields, pc[:, :4])
        else:
            msg = point_cloud2.create_cloud_xyz32(header, pc[:, :3])
        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, pose_estimation, margin_factor=1.5):
        """根据 map->odom 估计位姿裁剪全局地图的FOV区域。
        
        Args:
            pose_estimation: map->odom 变换矩阵
            margin_factor: FOV扩展系数，>1表示扩大裁剪范围以容忍位姿误差
        """
        if self.cur_odom is None:
            # 没有里程计时，直接用 map->odom 估计裁剪地图
            T_map_to_base_link = pose_estimation
        else:
            T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        fov = self.get_parameter("fov").value
        fov_far = self.get_parameter("fov_far").value * margin_factor

        # 扩大FOV范围以容忍初始位姿误差
        half_fov = fov / 2.0 * margin_factor

        if fov > 3.14:
            # 360度FOV：按距离裁剪
            indices = np.where(
                (global_map_in_base_link[:, 0] < fov_far)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < half_fov)
            )
        else:
            # 前向FOV：同时用距离+距离兜底（不留空）
            dist = np.sqrt(
                global_map_in_base_link[:, 0]**2
                + global_map_in_base_link[:, 1]**2
                + global_map_in_base_link[:, 2]**2
            )
            indices = np.where(
                (global_map_in_base_link[:, 0] > -1.0)  # 允许后方1m（容忍误差）
                & (dist < fov_far)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < half_fov)
            )
        # 最小点数保护：点太少就用距离裁剪兜底
        if len(indices[0]) < 200:
            dist = np.sqrt(
                global_map_in_base_link[:, 0]**2
                + global_map_in_base_link[:, 1]**2
                + global_map_in_base_link[:, 2]**2
            )
            indices = np.where(dist < fov_far)

        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("map_frame").value
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def generate_initial_hypotheses(self, base_pose, num_samples=8, trans_std=1.0, yaw_std=0.3):
        """在初始位姿附近采样多个假设，提高ICP收敛成功率"""
        hypotheses = [base_pose]  # 始终包含原始位姿
        np.random.seed(42)  # 固定种子保证可重复
        for _ in range(num_samples):
            T = base_pose.copy()
            # 随机平移扰动
            T[0, 3] += np.random.normal(0, trans_std)
            T[1, 3] += np.random.normal(0, trans_std)
            T[2, 3] += np.random.normal(0, trans_std * 0.3)  # z轴扰动小一些
            # 随机偏航角扰动
            yaw = np.random.normal(0, yaw_std)
            cos_y, sin_y = np.cos(yaw), np.sin(yaw)
            R_yaw = np.array([
                [cos_y, -sin_y, 0],
                [sin_y,  cos_y, 0],
                [0,       0,     1]
            ])
            T[:3, :3] = R_yaw @ base_pose[:3, :3]
            hypotheses.append(T)
        return hypotheses

    def initial_alignment(self, initial_pose):
        """首次初始对齐：多假设搜索 + 3级级联ICP + point-to-plane，一次性的高CPU开销"""
        self.get_logger().info("Starting initial alignment with multi-hypothesis search...")

        # 合并最近几帧扫描构建子图（比单帧更鲁棒）
        if hasattr(self, 'scan_buffer') and len(self.scan_buffer) > 1:
            merged_scan = o3d.geometry.PointCloud()
            for s in self.scan_buffer:
                merged_scan += s
            scan = merged_scan
            self.get_logger().info(f"Merged {len(self.scan_buffer)} scans: {len(merged_scan.points)} pts")
        else:
            scan = copy.copy(self.cur_scan)

        # 多假设：原始位姿 + 4 个扰动（平移±1m，偏航±15°）
        hypotheses = self.generate_initial_hypotheses(initial_pose, num_samples=4)

        best_transformation = initial_pose
        best_fitness = 0.0

        for idx, hypothesis in enumerate(hypotheses):
            global_map_in_FOV = self.crop_global_map_in_FOV(hypothesis)
            if len(global_map_in_FOV.points) < 100:
                continue

            # Stage 1: 粗配准 (scale=5, point-to-point)
            T_coarse, _ = self.registration_at_scale(
                scan, global_map_in_FOV, initial=hypothesis, scale=5
            )
            # Stage 2: 中配准 (scale=2, point-to-point, 用粗结果初始化)
            T_medium, _ = self.registration_at_scale(
                scan, global_map_in_FOV, initial=T_coarse, scale=2
            )
            # Stage 3: 精配准 (scale=1, point-to-plane, 用中结果初始化)
            T_fine, fitness = self.registration_at_scale(
                scan, global_map_in_FOV, initial=T_medium,
                scale=1, use_point_to_plane=True
            )

            self.get_logger().info(f"  hypothesis {idx}: fitness={fitness:.4f}")

            if fitness > best_fitness:
                best_fitness = fitness
                best_transformation = T_fine

            if best_fitness > 0.95:  # 足够好，提前结束
                break

        self.get_logger().info(f"Initial alignment done. Best fitness: {best_fitness:.4f}")

        threshold = self.get_parameter("localization_threshold").value
        if best_fitness > threshold:
            self.T_map_to_odom = best_transformation
            self.publish_odom(best_transformation)
            self.initialized = True
            self.get_logger().info("Initial alignment SUCCESS.")
        else:
            self.get_logger().warn(
                f"Initial alignment FAILED: best fitness {best_fitness:.4f} < threshold {threshold}"
            )

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)
        
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        
        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg
        
    def cb_save_cur_scan(self, msg):
        pc = self.msg_to_array(msg)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)
        self.publish_point_cloud(self.pub_pc_in_map, msg.header, pc)
        
        # 累积局部子图：保留最近3帧用于初始对齐（仅首次对齐时使用）
        if not hasattr(self, 'scan_buffer'):
            self.scan_buffer = []
        self.scan_buffer.append(copy.copy(self.cur_scan))
        if len(self.scan_buffer) > 3:
            self.scan_buffer.pop(0)
        
    def initialize_global_map(self): #, pc_msg):
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        self.global_map = o3d.io.read_point_cloud(self.get_parameter("pcd_map_path").value)
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        # o3d.io.write_point_cloud("/home/wheelchair2/laksh_ws/pcds/lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd", self.global_map)
        self.get_logger().info("Global map received.")

    def cb_initialpose(self, msg):
        T_map_to_base_link = self.pose_to_mat(msg.pose.pose)
        self.get_logger().info("Initial pose received, starting alignment...")

        if self.cur_odom is None:
            self.get_logger().warn("No odometry received yet, cannot compute initial map->odom")
            return

        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        initial_pose = np.matmul(T_map_to_base_link, self.inverse_se3(T_odom_to_base_link))
        
        if self.cur_scan is not None:
            self.initial_alignment(initial_pose)
        else:
            self.get_logger().warn("No scan received yet, cannot align")
            
    def publish_odom(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat = self.mat_to_quat(transform)
        odom_msg.pose.pose = Pose(
            position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
            orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        )
        if self.cur_odom is not None:
            odom_msg.header.stamp = self.cur_odom.header.stamp
        else:
            odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.get_parameter("map_frame").value
        odom_msg.child_frame_id = self.get_parameter("odom_frame").value
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self):
        """持续跟踪：轻量级 ICP，低 CPU 开销"""
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...", throttle_duration_sec=5.0)
            return
        
        if self.cur_scan is None:
            return

        # 用当前 scan（不做多帧合并，省 CPU）
        scan = copy.copy(self.cur_scan)

        # 2 级级联 ICP：coarse(scale=3) → fine(scale=1)，均 point-to-point
        try:
            global_map_in_FOV = self.crop_global_map_in_FOV(self.T_map_to_odom)

            T_coarse, _ = self.registration_at_scale(
                scan, global_map_in_FOV, initial=self.T_map_to_odom, scale=3
            )
            T_fine, fitness = self.registration_at_scale(
                scan, global_map_in_FOV, initial=T_coarse, scale=1
            )

            if fitness > self.get_parameter("localization_threshold").value:
                self.T_map_to_odom = T_fine
                self.publish_odom(T_fine)
        except Exception as e:
            self.get_logger().warn(f"Tracking ICP failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
