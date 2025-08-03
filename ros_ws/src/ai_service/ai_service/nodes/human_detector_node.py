#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

# ROS 2 표준 이미지 메시지 타입을 임포트합니다.
from sensor_msgs.msg import Image

# 직접 정의한 커스텀 메시지 타입을 임포트합니다.
# 이 메시지는 탐지된 사람의 정보를 담고 있습니다.
from libo_interfaces.msg import HumanInfo

# 사람 탐지 정보를 구조화하여 저장하기 위한 클래스입니다. (노드 내부용)
# ROS 메시지와는 별개로, 코드 내에서 데이터를 편리하게 다루기 위해 사용됩니다.
class PersonDetection:
    def __init__(self):
        self.track_id = None            # YOLOv8이 부여하는 추적 ID
        self.x_center = 0.0             # 바운딩 박스의 x축 중심 좌표
        self.y_center = 0.0             # 바운딩 박스의 y축 중심 좌표
        self.width = 0.0                # 바운딩 박스의 너비
        self.height = 0.0               # 바운딩 박스의 높이
        self.confidence = 0.0           # 탐지 신뢰도 (0.0 ~ 1.0)
        self.distance = 0.0             # 로봇으로부터의 거리 (미터 단위)
        self.horizontal_offset = 0.0    # 이미지 중심으로부터의 수평 오프셋 (비율)

class HumanDetectorNode(Node):
    """
    RGB-D 카메라를 사용하여 사람을 탐지하고 추적하는 ROS 2 노드.
    탐지된 사람 중 가장 가까운 사람을 타겟으로 지정하고, 
    해당 타겟의 정보를 커스텀 메시지('/human_info')로 발행합니다.
    """
    def __init__(self):
        # 노드 이름을 'human_detector_node'로 초기화합니다.
        super().__init__('human_detector_node')
        
        # --- 1. 의존성 및 모델 초기화 ---
        self.bridge = CvBridge()  # ROS 이미지 메시지와 OpenCV 이미지 간의 변환을 위한 브릿지
        self.model = YOLO('yolov8n.pt')  # YOLOv8 객체 탐지 모델 로드 (가장 작은 'nano' 버전)
        
        # --- 2. 내부 변수 초기화 ---
        self.rgb_image = None  # RGB 카메라 이미지를 저장할 변수
        self.depth_image = None  # Depth 카메라 이미지를 저장할 변수
        
        # --- 3. 타겟 관리 변수 ---
        self.target_track_id = None  # 현재 추적 중인 타겟의 ID
        self.target_lost_count = 0  # 타겟을 놓친 횟수를 카운트하는 변수
        self.target_lost_threshold = 50  # 타겟을 완전히 잃었다고 판단하기 위한 임계값 (50 프레임, 약 5초)
        self.min_detection_distance = 0.3  # 타겟으로 인식할 최소 거리 (0.3m)
        self.max_detection_distance = 5.0  # 타겟으로 인식할 최대 거리 (5.0m)
        
        # --- 4. 파라미터 설정 ---
        self.confidence_threshold = 0.5  # 사람으로 판단할 최소 신뢰도
        self.person_class_id = 0  # COCO 데이터셋에서 'person' 클래스의 ID는 0입니다.
        self.show_window = True  # 디버깅용 시각화 창을 켤지 여부

        # --- 5. ROS 2 통신 설정 ---
        
        # [입력] RGB 이미지 토픽을 구독합니다.
        self.rgb_subscriber = self.create_subscription(
            Image, '/ascamera_nuwa/camera_publisher/rgb0/image', self.rgb_callback, 10)
        
        # [입력] Depth 이미지 토픽을 구독합니다.
        self.depth_subscriber = self.create_subscription(
            Image, '/ascamera_nuwa/camera_publisher/depth0/image_raw', self.depth_callback, 10)
        
        # [출력] 탐지된 사람의 정보를 발행할 Publisher를 생성합니다.
        self.info_publisher = self.create_publisher(HumanInfo, 'human_info', 10)
        
        # 0.1초마다 (10Hz) 메인 처리 함수인 'process_detection'을 실행할 타이머를 생성합니다.
        self.timer = self.create_timer(0.1, self.process_detection)
        
        self.get_logger().info('🤖 Human Detector Node 시작! (Custom Message Publisher)')
        self.get_logger().info("📢 '/human_info' 토픽으로 탐지된 사람의 데이터를 발행합니다.")
        self.get_logger().info("🖥️ 'Human Detection Debug' 창으로 시각화 결과를 표시합니다.")

    def rgb_callback(self, msg):
        """RGB 이미지 메시지를 수신하면 호출되는 콜백 함수."""
        try:
            # ROS 이미지 메시지를 OpenCV의 BGR8 포맷으로 변환합니다.
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'RGB 이미지 변환 실패: {e}')
    
    def depth_callback(self, msg):
        """Depth 이미지 메시지를 수신하면 호출되는 콜백 함수."""
        try:
            # ROS 이미지 메시지를 OpenCV의 16비트 단일 채널(16UC1) 포맷으로 변환합니다.
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'Depth 이미지 변환 실패: {e}')

    def find_valid_detections(self, all_detections):
        """탐지된 객체들 중 설정된 거리 범위 내에 있는 유효한 객체만 필터링합니다."""
        return [d for d in all_detections if self.min_detection_distance <= d.distance <= self.max_detection_distance]

    def find_closest_detection(self, detections):
        """주어진 객체 리스트에서 거리가 가장 가까운 객체를 찾습니다."""
        if not detections:
            return None
        return min(detections, key=lambda d: d.distance)
        
    def update_target_tracking(self, all_detections):
        """타겟 추적 로직을 업데이트하고 현재 상태와 타겟 정보를 반환합니다."""
        valid_detections = self.find_valid_detections(all_detections)
        
        # 1. 현재 추적 중인 타겟이 있는 경우
        if self.target_track_id is not None:
            # 유효한 탐지 목록에서 현재 타겟 ID를 찾아봅니다.
            target_detection = next((d for d in valid_detections if d.track_id == self.target_track_id), None)
            
            if target_detection:
                # 타겟을 계속 발견한 경우: lost 카운터를 리셋하고 타겟 정보를 반환합니다.
                self.target_lost_count = 0
                return "TARGET_TRACKED", target_detection
            else:
                # 타겟을 놓친 경우: lost 카운터를 증가시킵니다.
                self.target_lost_count += 1
                if self.target_lost_count >= self.target_lost_threshold:
                    # 임계값을 초과하면 타겟을 완전히 잃었다고 판단하고 타겟 정보를 초기화합니다.
                    self.target_track_id = None
                    self.target_lost_count = 0
                    return "TARGET_LOST", None
                else:
                    # 아직 임계값 미만이면, 타겟이 다시 나타나길 기다립니다.
                    return "TARGET_WAITING", None
        
        # 2. 현재 추적 중인 타겟이 없는 경우
        if valid_detections:
            # 유효한 탐지 목록에서 가장 가까운 객체를 새로운 타겟으로 지정합니다.
            new_target = self.find_closest_detection(valid_detections)
            if new_target and new_target.track_id is not None:
                self.target_track_id = new_target.track_id
                self.target_lost_count = 0
                return "TARGET_ACQUIRED", new_target
                
        # 3. 유효한 탐지 객체가 아무도 없는 경우
        return "NO_TARGET", None
        
    def calculate_distance(self, x, y):
        """주어진 좌표 (x, y)를 중심으로 한 5x5 영역의 평균 깊이 값을 계산하여 거리(m)를 반환합니다."""
        if self.depth_image is None:
            return 0.0
        h, w = self.depth_image.shape
        # 좌표가 이미지 범위 내에 있는지 확인합니다.
        if 0 <= y < h and 0 <= x < w:
            # 중심점 주변의 5x5 영역을 설정합니다. (이미지 경계를 넘지 않도록)
            x_start, x_end = max(0, x - 2), min(w, x + 3)
            y_start, y_end = max(0, y - 2), min(h, y + 3)
            
            # 해당 영역의 깊이 값을 추출합니다.
            depth_region = self.depth_image[y_start:y_end, x_start:x_end]
            
            # 깊이 값이 0인 경우(측정 실패)를 제외하고 유효한 값만 필터링합니다.
            valid_depths = depth_region[depth_region > 0]
            
            if len(valid_depths) > 0:
                # 유효한 깊이 값들의 평균을 계산하고, mm 단위를 m 단위로 변환하여 반환합니다.
                return np.mean(valid_depths) / 1000.0
        return 0.0
    
    def draw_detection(self, image, detection, is_target=False):
        """하나의 탐지 결과를 이미지에 시각적으로 그립니다 (바운딩 박스, 정보 텍스트)."""
        if is_target:
            # 타겟인 경우: 자홍색, 더 두꺼운 선으로 표시
            base_color = (255, 0, 255) 
            thickness = 3
            text_prefix = f"TARGET[{detection.track_id}]: "
        else:
            # 일반 탐지인 경우: 녹색, 일반 두께로 표시
            base_color = (0, 255, 0)
            thickness = 2
            text_prefix = f"ID[{detection.track_id}]: "

        # 바운딩 박스의 좌상단(x1, y1)과 우하단(x2, y2) 좌표를 계산합니다.
        x1 = int(detection.x_center - detection.width / 2)
        y1 = int(detection.y_center - detection.height / 2)
        x2 = int(detection.x_center + detection.width / 2)
        y2 = int(detection.y_center + detection.height / 2)
        
        # 사각형을 그립니다.
        cv2.rectangle(image, (x1, y1), (x2, y2), base_color, thickness)
        
        # 표시할 텍스트를 구성합니다 (ID, 거리, 수평 위치).
        text = f'{text_prefix}{detection.distance:.2f}m | Pos:{detection.horizontal_offset:+.2f}'
            
        # 텍스트를 위한 배경 사각형을 그립니다.
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(image, (x1, y1 - h - 10), (x1 + w, y1), base_color, -1)
        # 텍스트를 씁니다.
        cv2.putText(image, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def draw_status_info(self, image, tracking_status, target_detection):
        """이미지 좌측 상단에 현재 노드의 전반적인 상태 정보를 그립니다."""
        status_text = f'Status: {tracking_status}'
        if target_detection:
            status_text += f' (Target ID: {target_detection.track_id})'
        
        cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    def process_detection(self):
        """메인 처리 함수: 10Hz로 실행되며, 탐지, 추적, 발행, 시각화의 전체 과정을 담당합니다."""
        if self.rgb_image is None:
            # 아직 이미지를 수신하지 못했다면 함수를 종료합니다.
            return

        # 시각화를 위해 원본 이미지를 복사합니다.
        debug_image = self.rgb_image.copy()
            
        # --- 1. 사람 탐지 및 정보 추출 ---
        # YOLOv8 모델로 이미지에서 객체를 추적합니다. persist=True는 프레임 간 추적 정보를 유지합니다.
        results = self.model.track(self.rgb_image, persist=True, verbose=False)
        h, w, _ = self.rgb_image.shape
        image_center_x = w // 2
        
        all_detections = [] # 이번 프레임에서 탐지된 모든 사람 정보를 저장할 리스트
        if results and results[0].boxes:
            for box in results[0].boxes:
                # 'person' 클래스이고 신뢰도가 임계값 이상인 경우에만 처리
                if int(box.cls[0]) == self.person_class_id and float(box.conf[0]) >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # PersonDetection 객체를 생성하고 탐지 정보를 채웁니다.
                    detection = PersonDetection()
                    detection.track_id = int(box.id[0]) if box.id is not None else -1
                    detection.confidence = float(box.conf[0])
                    detection.x_center = float(center_x)
                    detection.y_center = float(center_y)
                    detection.width = float(x2 - x1)
                    detection.height = float(y2 - y1)
                    detection.distance = self.calculate_distance(center_x, center_y)
                    detection.horizontal_offset = (center_x - image_center_x) / image_center_x if image_center_x > 0 else 0.0
                    all_detections.append(detection)

        # --- 2. 타겟 추적 및 발행할 데이터 결정 ---
        tracking_status, target_detection = self.update_target_tracking(all_detections)
        
        # 발행할 HumanInfo 메시지 객체를 생성합니다.
        human_info_msg = HumanInfo()
        
        if target_detection:
            # 추적 중인 타겟이 있으면, 타겟의 정보로 메시지를 채웁니다.
            human_info_msg.is_detected = True
            human_info_msg.track_id = target_detection.track_id
            human_info_msg.confidence = target_detection.confidence
            human_info_msg.distance = target_detection.distance
            human_info_msg.horizontal_offset = target_detection.horizontal_offset
        else:
            # 타겟이 없으면, '탐지 안됨' 상태로 메시지를 채웁니다.
            human_info_msg.is_detected = False
            human_info_msg.track_id = -1
            human_info_msg.confidence = 0.0
            human_info_msg.distance = 0.0
            human_info_msg.horizontal_offset = 0.0

        # --- 3. HumanInfo 메시지 발행 ---
        self.info_publisher.publish(human_info_msg)
        
        # --- 4. 시각적 디버깅 ---
        if self.show_window:
            # 모든 탐지된 사람을 시각화합니다.
            for det in all_detections:
                is_target = (target_detection is not None and det.track_id == target_detection.track_id)
                self.draw_detection(debug_image, det, is_target)
            
            # 현재 추적 상태 정보를 시각화합니다.
            self.draw_status_info(debug_image, tracking_status, target_detection)

            # 결과 이미지를 창에 표시합니다.
            cv2.imshow('Human Detection Debug', debug_image)
            key = cv2.waitKey(1) & 0xFF
            # 'q' 키를 누르면 종료합니다.
            if key == ord('q'):
                self.get_logger().info('👋 Q키로 종료 요청됨')
                rclpy.shutdown()

def main(args=None):
    # ROS 2 시스템을 초기화합니다.
    rclpy.init(args=args)
    # HumanDetectorNode 인스턴스를 생성합니다.
    node = HumanDetectorNode()
    try:
        # 노드를 실행하고 콜백 함수들이 호출되도록 대기합니다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 입력 시, 루프를 종료합니다.
        pass
    finally:
        # 노드가 종료될 때 모든 OpenCV 창을 닫습니다.
        cv2.destroyAllWindows()
        # rclpy를 안전하게 종료합니다.
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    # 이 스크립트가 직접 실행될 때 main 함수를 호출합니다.
    main()