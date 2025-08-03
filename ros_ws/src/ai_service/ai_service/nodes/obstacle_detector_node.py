#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from sensor_msgs.msg import LaserScan
# ⭐️ [수정] custom message의 import 경로를 'libo_interfaces'로 변경합니다.
from libo_interfaces.msg import DetectionStatus 
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # --- 1. 파라미터 선언 및 초기화 ---
        self.declare_parameter('scan_topic', '/scan')
        
        # 중앙 ROI 파라미터
        self.declare_parameter('center_roi_min_dist_m', 0.15)
        self.declare_parameter('center_roi_max_dist_m', 0.3)
        
        # 좌우(Side) ROI 파라미터
        self.declare_parameter('side_roi_x_min_m', -0.65)
        self.declare_parameter('side_roi_x_max_m', 0.3)
        self.declare_parameter('side_roi_inner_edge_m', 0.22)
        self.declare_parameter('side_roi_outer_edge_m', 0.32)

        # 후방(Rear) ROI 파라미터
        self.declare_parameter('rear_roi_x_min_m', -0.65)
        self.declare_parameter('rear_roi_x_max_m', -0.52)
        self.declare_parameter('rear_roi_width_m', 0.4)

        # DBSCAN 클러스터링 파라미터
        self.declare_parameter('dbscan_eps', 0.15)
        self.declare_parameter('dbscan_min_samples', 5)

        # 파라미터 값 가져오기
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.center_min_dist = self.get_parameter('center_roi_min_dist_m').get_parameter_value().double_value
        self.center_max_dist = self.get_parameter('center_roi_max_dist_m').get_parameter_value().double_value
        self.side_x_min = self.get_parameter('side_roi_x_min_m').get_parameter_value().double_value
        self.side_x_max = self.get_parameter('side_roi_x_max_m').get_parameter_value().double_value
        self.side_inner_edge = self.get_parameter('side_roi_inner_edge_m').get_parameter_value().double_value
        self.side_outer_edge = self.get_parameter('side_roi_outer_edge_m').get_parameter_value().double_value
        self.rear_x_min = self.get_parameter('rear_roi_x_min_m').get_parameter_value().double_value
        self.rear_x_max = self.get_parameter('rear_roi_x_max_m').get_parameter_value().double_value
        self.rear_width = self.get_parameter('rear_roi_width_m').get_parameter_value().double_value
        self.dbscan_eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value

        # --- 2. 발행자(Publisher) 및 구독자(Subscriber) 생성 ---
        self.status_pub_ = self.create_publisher(DetectionStatus, '/detection_status', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.roi_scan_pub_ = self.create_publisher(LaserScan, '/scan_roi', 10)

        self.scan_sub_ = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)

        self.publish_roi_boundaries()

        self.get_logger().info("✅ 장애물 감지 및 시각화 노드가 시작되었습니다.")
        self.get_logger().info(f"   >> 중앙 감지 거리(x): {self.center_min_dist:.2f}m ~ {self.center_max_dist:.2f}m")
        self.get_logger().info(f"   >> 좌우 감지 거리(x): {self.side_x_min:.2f}m ~ {self.side_x_max:.2f}m")
        self.get_logger().info(f"   >> 후방 감지 거리(x): {self.rear_x_min:.2f}m ~ {self.rear_x_max:.2f}m")

    def scan_callback(self, scan_msg: LaserScan):
        roi_scan_msg = LaserScan()
        roi_scan_msg.header = scan_msg.header
        roi_scan_msg.angle_min = scan_msg.angle_min
        roi_scan_msg.angle_max = scan_msg.angle_max
        roi_scan_msg.angle_increment = scan_msg.angle_increment
        roi_scan_msg.time_increment = scan_msg.time_increment
        roi_scan_msg.scan_time = scan_msg.scan_time
        roi_scan_msg.range_min = scan_msg.range_min
        roi_scan_msg.range_max = scan_msg.range_max
        roi_ranges = [np.inf] * len(scan_msg.ranges)

        all_points_in_roi = []
        point_roi_labels = []

        for i, distance in enumerate(scan_msg.ranges):
            if np.isinf(distance) or np.isnan(distance):
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            raw_x = distance * np.cos(angle)
            raw_y = distance * np.sin(angle)
            x = -raw_x
            y = raw_y
            
            is_in_center = (self.center_min_dist < x < self.center_max_dist) and (abs(y) < self.side_inner_edge)
            is_in_left = (self.side_x_min < x < self.side_x_max) and (-self.side_outer_edge <= y < -self.side_inner_edge)
            is_in_right = (self.side_x_min < x < self.side_x_max) and (self.side_inner_edge < y <= self.side_outer_edge)
            is_in_rear = (self.rear_x_min < x < self.rear_x_max) and (abs(y) < self.rear_width / 2.0)
            
            if is_in_center or is_in_left or is_in_right or is_in_rear:
                roi_ranges[i] = distance
                all_points_in_roi.append([x, y])
                if is_in_center: point_roi_labels.append("center")
                elif is_in_left: point_roi_labels.append("left")
                elif is_in_right: point_roi_labels.append("right")
                elif is_in_rear: point_roi_labels.append("rear")

        roi_scan_msg.ranges = roi_ranges
        self.roi_scan_pub_.publish(roi_scan_msg)

        status_msg = DetectionStatus()
        status_msg.center_detected = False
        status_msg.left_detected = False
        status_msg.right_detected = False
        status_msg.rear_detected = False

        if not all_points_in_roi:
            self.status_pub_.publish(status_msg)
            self.clear_markers()
            return

        points_np = np.array(all_points_in_roi)
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points_np)
        labels = db.labels_

        if not np.any(labels > -1):
            self.status_pub_.publish(status_msg)
            self.clear_markers()
            return

        detected_rois = set()
        marker_array = MarkerArray()
        
        for label in set(labels):
            if label == -1:
                continue
            class_member_mask = (labels == label)
            for i, mask_val in enumerate(class_member_mask):
                if mask_val:
                    detected_rois.add(point_roi_labels[i])
            center_x = np.mean(points_np[class_member_mask, 0])
            center_y = np.mean(points_np[class_member_mask, 1])
            marker = self.create_cluster_marker(label, center_x, center_y)
            marker_array.markers.append(marker)

        if "center" in detected_rois: status_msg.center_detected = True
        if "left" in detected_rois: status_msg.left_detected = True
        if "right" in detected_rois: status_msg.right_detected = True
        if "rear" in detected_rois: status_msg.rear_detected = True
        
        self.status_pub_.publish(status_msg)
        self.marker_pub_.publish(marker_array)

    def create_cluster_marker(self, marker_id, x, y):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "clusters"
        marker.id = int(marker_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        marker.lifetime = Duration(sec=1)
        return marker

    def publish_roi_boundaries(self):
        marker_array = MarkerArray()
        
        marker_array.markers.append(self.create_roi_marker(
            0, (self.center_min_dist, self.center_max_dist),
            (-self.side_inner_edge, self.side_inner_edge), 
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.3)))
        marker_array.markers.append(self.create_roi_marker(
            1, (self.side_x_min, self.side_x_max),
            (-self.side_outer_edge, -self.side_inner_edge), 
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)))
        marker_array.markers.append(self.create_roi_marker(
            2, (self.side_x_min, self.side_x_max),
            (self.side_inner_edge, self.side_outer_edge), 
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)))
        marker_array.markers.append(self.create_roi_marker(
            3, (self.rear_x_min, self.rear_x_max),
            (-self.rear_width / 2.0, self.rear_width / 2.0), 
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.3)))
            
        self.marker_pub_.publish(marker_array)

    def create_roi_marker(self, marker_id, x_bounds, y_bounds, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "roi_boundaries"
        marker.id = int(marker_id)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.color = color
        marker.lifetime = Duration()
        
        x_min, x_max = x_bounds
        y_min, y_max = y_bounds

        points = [
            (x_min, y_min, 0.0), (x_max, y_min, 0.0),
            (x_max, y_max, 0.0), (x_min, y_max, 0.0),
            (x_min, y_min, 0.0)
        ]
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        return marker

    def clear_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.ns = "clusters"
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_pub_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트로 노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()