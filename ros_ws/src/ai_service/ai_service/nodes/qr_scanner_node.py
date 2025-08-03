#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# [수정] 명세서에 따라 필요한 서비스 타입만 import
from libo_interfaces.srv import ActivateQRScanner, DeactivateQRScanner # QR 스캐너 제어용
from libo_interfaces.srv import RobotQRCheck # 이름 유효성 검사용

class QRCodeValidator(Node):
    """
    [수정] 명세서 아키텍처를 따르는 QR 코드 검증 노드.
    외부 요청에 의해 활성화되고, 인식된 QR 정보의 유효성을 다른 서비스에 요청하여 검증합니다.
    """
    def __init__(self):
        super().__init__('qr_code_validator')

        # --- 1. 상태 변수 및 ROS 인터페이스 초기화 ---
        self.is_active = False          # 노드의 활성화 상태. True여야만 QR 인식을 수행
        self.processing_qr = False      # 현재 QR 코드를 처리 중인지 여부 (중복 요청 방지)
        self.robot_id = 'libo_a'        # 이 노드가 동작하는 로봇의 ID
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        # 이미지 구독 (카메라로부터 영상 수신)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera_nuwa/camera_publisher/rgb0/image', # 사용하는 카메라 토픽
            self.image_callback,
            10)
        
        # [수정] 서비스 클라이언트: Libo Service에 이름 유효성을 '문의'하기 위한 클라이언트
        self.robot_check_client = self.create_client(RobotQRCheck, '/robot_qr_check')

        # [신규] 서비스 서버: 외부(Libo Service)에서 이 노드의 스캔 기능을 제어하기 위한 서버
        self.activate_qr_srv = self.create_service(
            ActivateQRScanner, '/activate_qr_scanner', self.handle_activate_qr_scanner)
        self.deactivate_qr_srv = self.create_service(
            DeactivateQRScanner, '/deactivate_qr_scanner', self.handle_deactivate_qr_scanner)

        # 디버깅용 결과 퍼블리셔
        self.result_publisher = self.create_publisher(String, '/qr_auth_result', 10)
        
        self.get_logger().info('✅ QR Code Validator Node started (간소화 버전). 활성화 대기 중...')

    # --- 2. 서비스 서버 콜백 함수 (외부 제어) ---
    def handle_activate_qr_scanner(self, request, response):
        """'/activate_qr_scanner' 서비스 요청을 처리하여 스캐너를 활성화합니다."""
        self.get_logger().info('🟢 QR 스캐너 활성화 요청 수신. 스캔을 시작합니다.')
        self.is_active = True
        self.processing_qr = False # 새로운 활성화 시, 처리 상태 초기화
        response.success = True
        response.message = "QR scanner activated."
        return response

    def handle_deactivate_qr_scanner(self, request, response):
        """'/deactivate_qr_scanner' 서비스 요청을 처리하여 스캐너를 비활성화합니다."""
        self.get_logger().info('🔴 QR 스캐너 비활성화 요청 수신. 스캔을 중지합니다.')
        self.is_active = False
        response.success = True
        response.message = "QR scanner deactivated."
        return response

    # --- 3. 메인 로직 (이미지 처리) ---
    def image_callback(self, msg):
        """카메라 이미지를 수신하여 QR 인식 및 처리 로직을 수행합니다."""
        # 비활성화 상태이거나, 이미 다른 QR을 처리 중이면 아무것도 하지 않음
        if not self.is_active or self.processing_qr:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            data, bbox, _ = self.detector.detectAndDecode(cv_image)
        except Exception as e:
            self.get_logger().error(f'❌ 이미지 처리 실패: {e}')
            return

        # QR 코드가 감지되었고, 내부에 데이터가 있는 경우
        if bbox is not None and data:
            self.get_logger().info(f'🔎 QR 코드 감지: "{data}". 유효성 검사를 요청합니다.')
            self.processing_qr = True # 중복 처리를 막기 위해 플래그 설정
            self.validate_name_with_service(data.strip()) # 외부 서비스에 유효성 검사 요청
            
    # --- 4. 서비스 클라이언트 호출 및 응답 처리 ---
    def validate_name_with_service(self, name: str):
        """
        인식된 이름을 'RobotQRCheck.srv'를 통해 Libo Service에 보내 유효성을 검증받습니다.
        """
        if not self.robot_check_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('❌ RobotQRCheck 서비스 연결 실패')
            self.processing_qr = False # 처리 실패 시, 다시 시도할 수 있도록 플래그 해제
            return

        request = RobotQRCheck.Request()
        request.robot_id = self.robot_id
        request.admin_name = name

        future = self.robot_check_client.call_async(request)
        # 서비스 응답이 오면 'handle_validation_response' 콜백 함수가 실행되도록 설정
        future.add_done_callback(
            lambda fut: self.handle_validation_response(fut, name))

    def handle_validation_response(self, future, name: str):
        """
        [수정] 'RobotQRCheck' 서비스의 응답을 처리하고, 모든 인증 절차를 마무리합니다.
        """
        try:
            response = future.result()
            if response.success:
                # 인증 성공 시
                self.get_logger().info(f'🟢 QR 인증 성공: {name}')
                self.result_publisher.publish(String(data=f'success:{name}'))
            else:
                # 인증 실패 시
                self.get_logger().warn(f'🔴 인증 실패 - 미등록 이름 또는 서버 거부: "{name}"')
                self.result_publisher.publish(String(data=f'fail:{name}'))
        except Exception as e:
            self.get_logger().error(f'📤 유효성 검사 요청 중 예외 발생: {e}')
        finally:
            # 성공/실패 여부와 관계없이, 한 번의 인증 절차가 끝나면 스캐너를 비활성화하여 자원 낭비를 막음
            self.get_logger().info('🔒 한 번의 인증 절차 완료. 스캐너를 자동으로 비활성화합니다.')
            self.is_active = False
            self.processing_qr = False

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()