import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# libo_interfaces 패키지에서 서비스 import
from libo_interfaces.srv import QRScanResult, ActivateTalker, ActivateTracker

class QRCodeValidator(Node):
    def __init__(self):
        super().__init__('qr_code_validator')

        # ✅ 1. 이미지 구독 (스테레오 카메라)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera_nuwa/camera_publisher/rgb0/image',
            self.image_callback,
            10)

        # ✅ 2. 서비스 클라이언트 초기화
        self.qr_result_client = self.create_client(QRScanResult, '/qr_scan_result')
        self.activate_talker_client = self.create_client(ActivateTalker, '/activate_talker')
        self.activate_tracker_client = self.create_client(ActivateTracker, '/activate_tracker')

        # ✅ 3. 디버깅용 결과 퍼블리셔
        self.result_publisher = self.create_publisher(String, '/qr_auth_result', 10)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        # ✅ 4. 등록된 QR 이름 목록
        self.valid_names = ['김민수', '박지현', '이서준', '정예린', '최도현']
        self.robot_id = 'libo_a'

        self.get_logger().info('✅ QR Code Validator Node started with libo_interfaces.')

    def image_callback(self, msg):
        """이미지 수신 → QR 인식 → 인증 → 서비스 호출"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'❌ 이미지 변환 실패: {e}')
            return

        data, bbox, _ = self.detector.detectAndDecode(cv_image)

        if bbox is not None:
            name = data.strip()
            points = bbox[0].astype(int)

            cv2.polylines(cv_image, [points], True, (0, 255, 0), 2)
            cv2.putText(cv_image, name, tuple(points[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if name:
                if name in self.valid_names:
                    self.get_logger().info(f'🟢 QR 인증 성공: {name}')
                    self.result_publisher.publish(String(data=f'success:{name}'))
                    self.send_qr_result(name, success=True)
                    self.activate_talker()
                    self.activate_tracker()
                else:
                    self.get_logger().warn(f'🔴 인증 실패 - 미등록 이름: "{name}"')
                    self.result_publisher.publish(String(data=f'fail:{name}'))
                    self.send_qr_result(name, success=False)

        cv2.imshow("QR Scan Result", cv_image)
        cv2.waitKey(1)

    def send_qr_result(self, name: str, success: bool):
        """QRScanResult.srv 결과 전송"""
        if not self.qr_result_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('❌ QRScanResult 서비스 연결 실패')
            return

        request = QRScanResult.Request()
        request.robot_id = self.robot_id
        request.result = 'success' if success else 'fail'

        future = self.qr_result_client.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info(f'📤 QR 결과 전송 성공: {request.result}')
                else:
                    self.get_logger().warn(f'📤 QR 결과 실패: {response.message}')
            except Exception as e:
                self.get_logger().error(f'📤 QR 결과 예외: {e}')

        future.add_done_callback(callback)

    def activate_talker(self):
        """ActivateTalker.srv 호출"""
        if not self.activate_talker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('❌ ActivateTalker 서비스 연결 실패')
            return

        request = ActivateTalker.Request()
        request.robot_id = self.robot_id

        future = self.activate_talker_client.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info('🗣️ Talker 활성화 성공')
                else:
                    self.get_logger().warn(f'🗣️ Talker 실패: {response.message}')
            except Exception as e:
                self.get_logger().error(f'🗣️ Talker 예외: {e}')

        future.add_done_callback(callback)

    def activate_tracker(self):
        """ActivateTracker.srv 호출"""
        if not self.activate_tracker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('❌ ActivateTracker 서비스 연결 실패')
            return

        request = ActivateTracker.Request()
        request.robot_id = self.robot_id

        future = self.activate_tracker_client.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info('🛰️ Tracker 활성화 성공')
                else:
                    self.get_logger().warn(f'🛰️ Tracker 실패: {response.message}')
            except Exception as e:
                self.get_logger().error(f'🛰️ Tracker 예외: {e}')

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeValidator()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
