#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import KioskQRCheck
from PyQt5.QtCore import QObject, pyqtSignal
import cv2
import numpy as np
from pyzbar import pyzbar
import time

class KioskQRCheckClient(QObject):
    """KioskQRCheck 서비스 클라이언트"""
    
    # 시그널 정의
    qr_check_completed = pyqtSignal(bool, str)  # 성공 여부, 메시지
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.qr_check_client = None
        self.camera = None
        self.is_scanning = False
        
        # ROS2 클라이언트 초기화
        self.init_ros_client()
        
        print("✅ KioskQRCheckClient 초기화 완료")
    
    def init_ros_client(self):
        """ROS2 클라이언트 초기화"""
        try:
            self.qr_check_client = self.ros_node.create_client(KioskQRCheck, 'kiosk_qr_check')
            
            # 서비스가 준비될 때까지 대기
            while not self.qr_check_client.wait_for_service(timeout_sec=1.0):
                print("⏳ KioskQRCheck 서비스 대기 중...")
            
            print("✅ KioskQRCheck 서비스 연결 완료")
            
        except Exception as e:
            print(f"❌ KioskQRCheck 클라이언트 초기화 실패: {e}")
    
    def start_qr_scan(self):
        """QR 스캔 시작"""
        if self.is_scanning:
            print("⚠️ 이미 스캔 중입니다.")
            return
        
        self.is_scanning = True
        print("🔍 QR 스캔 시작...")
        
        # 카메라 초기화
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("❌ 카메라를 열 수 없습니다.")
            self.is_scanning = False
            self.qr_check_completed.emit(False, "카메라를 열 수 없습니다.")
            return
        
        # QR 스캔 루프
        self.scan_qr_loop()
    
    def scan_qr_loop(self):
        """QR 스캔 루프"""
        try:
            while self.is_scanning:
                ret, frame = self.camera.read()
                if not ret:
                    print("❌ 카메라에서 프레임을 읽을 수 없습니다.")
                    break
                
                # QR 코드 검출
                qr_codes = pyzbar.decode(frame)
                
                for qr_code in qr_codes:
                    qr_data = qr_code.data.decode('utf-8')
                    print(f"📱 QR 코드 감지: {qr_data}")
                    
                    # QR 데이터 파싱 (관리자 이름 추출)
                    admin_name = self.parse_qr_data(qr_data)
                    if admin_name:
                        # KioskQRCheck 서비스 호출
                        self.send_qr_check_request(admin_name)
                        return
                
                # 프레임 표시 (디버깅용)
                cv2.imshow('QR Scanner', frame)
                
                # ESC 키로 종료
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                
                # 100ms 대기
                time.sleep(0.1)
                
        except Exception as e:
            print(f"❌ QR 스캔 중 오류: {e}")
            self.qr_check_completed.emit(False, f"QR 스캔 중 오류: {str(e)}")
        finally:
            self.stop_qr_scan()
    
    def parse_qr_data(self, qr_data):
        """QR 데이터에서 관리자 이름 파싱"""
        try:
            # QR 데이터 형식: "admin:김민수" 또는 "김민수"
            if "admin:" in qr_data:
                admin_name = qr_data.split("admin:")[1]
            else:
                admin_name = qr_data
            
            # 관리자 이름 기본 검증 (DB에서 최종 검증)
            if admin_name and len(admin_name) > 0:
                return admin_name
            else:
                print(f"⚠️ 빈 관리자 이름: {admin_name}")
                return None
                
        except Exception as e:
            print(f"❌ QR 데이터 파싱 오류: {e}")
            return None
    
    def send_qr_check_request(self, admin_name):
        """KioskQRCheck 서비스 요청 전송"""
        try:
            if not self.qr_check_client:
                print("❌ KioskQRCheck 클라이언트가 초기화되지 않았습니다.")
                self.qr_check_completed.emit(False, "서비스 클라이언트가 초기화되지 않았습니다.")
                return
            
            # 서비스 요청 생성
            request = KioskQRCheck.Request()
            request.kiosk_id = "kiosk_1"
            request.admin_name = admin_name
            
            print(f"📤 KioskQRCheck 요청: kiosk_id={request.kiosk_id}, admin_name={request.admin_name}")
            
            # 비동기 서비스 호출
            future = self.qr_check_client.call_async(request)
            future.add_done_callback(self.qr_check_response_callback)
            
        except Exception as e:
            print(f"❌ KioskQRCheck 요청 전송 실패: {e}")
            self.qr_check_completed.emit(False, f"서비스 요청 실패: {str(e)}")
    
    def qr_check_response_callback(self, future):
        """KioskQRCheck 서비스 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                print(f"✅ QR 인증 성공: {response.message}")
                self.qr_check_completed.emit(True, response.message)
            else:
                print(f"❌ QR 인증 실패: {response.message}")
                self.qr_check_completed.emit(False, response.message)
                
        except Exception as e:
            print(f"❌ KioskQRCheck 응답 처리 중 오류: {e}")
            self.qr_check_completed.emit(False, f"응답 처리 오류: {str(e)}")
    
    def stop_qr_scan(self):
        """QR 스캔 중지"""
        self.is_scanning = False
        
        if self.camera:
            self.camera.release()
        
        cv2.destroyAllWindows()
        print("🛑 QR 스캔 중지")
    
    def cleanup(self):
        """리소스 정리"""
        self.stop_qr_scan()
        print("🧹 KioskQRCheckClient 정리 완료") 