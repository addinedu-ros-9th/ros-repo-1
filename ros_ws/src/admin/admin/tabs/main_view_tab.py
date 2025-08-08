#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time
import socket
import json
import cv2
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGraphicsScene, QGraphicsPixmapItem, QGraphicsView, QGraphicsRectItem, QGraphicsEllipseItem, QGraphicsItem
from PyQt5.QtCore import Qt, QRectF, QTimer, QThread, pyqtSignal, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QPixmap, QPainter, QImage, QPen, QBrush, QColor
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

# TaskStatus 메시지 import
from libo_interfaces.msg import TaskStatus, OverallStatus
# AddGoalLocation 서비스 import
from libo_interfaces.srv import AddGoalLocation

class VideoReceiverThread(QThread):
    """UDP 영상 수신 스레드"""
    frame_received = pyqtSignal(np.ndarray)  # 프레임 수신 시그널
    
    def __init__(self, port=7021):
        super().__init__()
        self.port = port
        self.running = False
        self.sock = None
        
    def run(self):
        """스레드 실행"""
        try:
            # UDP 소켓 생성
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.settimeout(1.0)  # 1초 타임아웃
            
            print(f"🎥 UDP 영상 수신 시작 (포트: {self.port})")
            self.running = True
            
            frame_count = 0
            
            while self.running:
                try:
                    # 데이터 수신
                    data, addr = self.sock.recvfrom(65536)  # 64KB 버퍼
                    
                    # 메시지 파싱 (header|image 형식)
                    if b'|' in data:
                        parts = data.split(b'|', 1)
                        if len(parts) == 2:
                            header_str = parts[0].decode('utf-8')
                            image_data = parts[1]
                            
                            # 헤더 파싱
                            try:
                                header = json.loads(header_str)
                                direction = header.get('direction', 'unknown')
                                frame_id = header.get('frame_id', 0)
                                
                                # JPEG 디코딩
                                nparr = np.frombuffer(image_data, np.uint8)
                                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                                
                                if frame is not None:
                                    # BGR to RGB 변환 (OpenCV는 BGR, Qt는 RGB)
                                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    self.frame_received.emit(frame_rgb)
                                    frame_count += 1
                                    
                                    # 100프레임마다 메모리 정리
                                    if frame_count % 100 == 0:
                                        import gc
                                        gc.collect()
                                else:
                                    pass
                                    
                            except (json.JSONDecodeError, cv2.error):
                                pass
                                
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"❌ UDP 수신 오류: {e}")
                    break
                    
        except Exception as e:
            print(f"❌ UDP 소켓 생성 실패: {e}")
        finally:
            if self.sock:
                self.sock.close()
            print("🛑 UDP 영상 수신 종료")
    
    def stop(self):
        """스레드 중지"""
        self.running = False
        self.wait()

class MapButton(QGraphicsItem):
    """맵 위의 클릭 가능한 버튼 아이템"""
    
    def __init__(self, button_id, x, y, width=40, height=40, parent=None, service_client=None):
        super().__init__(parent)
        self.button_id = button_id  # 버튼 ID 저장
        self.service_client = service_client  # 서비스 클라이언트 저장
        # 중심점 기준으로 좌상단 좌표 계산
        self.button_rect = QRectF(x - width/2, y - height/2, width, height)
        # 마우스 이벤트 허용
        self.setAcceptHoverEvents(True)
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setFlag(QGraphicsItem.ItemIsFocusable)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)
        
        # 애니메이션 관련 변수들
        self.animation_timer = None
        self.animation_circle = None
    
    def boundingRect(self):
        """바운딩 박스 반환"""
        return self.button_rect
    
    def paint(self, painter, option, widget):
        """그리기 (투명하게)"""
        # 투명하게 그리기 (시각적으로는 보이지 않음)
        painter.setPen(QPen(Qt.transparent))
        painter.setBrush(QBrush(Qt.transparent))
        painter.drawRect(self.button_rect)
    
    def mousePressEvent(self, event):
        """버튼 클릭 이벤트 처리"""
        print(f"🗺️ 맵 버튼 클릭: {self.button_id} (좌표: {self.button_rect.center().x():.1f}, {self.button_rect.center().y():.1f})")
        
        # AddGoalLocation 서비스 호출
        self.call_add_goal_service()
        
        # 빨간색 동그라미 애니메이션 생성
        self.create_click_animation()
        
        # 이벤트 처리 완료 (전파 방지)
        event.accept()
    
    def call_add_goal_service(self):
        """AddGoalLocation 서비스 호출"""
        try:
            if self.service_client:
                # 서비스 서버가 사용 가능한지 확인
                if not self.service_client.service_is_ready():
                    print(f"⚠️ 서비스 서버가 준비되지 않음: add_goal_location")
                    return
                
                # 서비스 요청 생성
                request = AddGoalLocation.Request()
                request.robot_id = "libo_a"  # 기본 로봇 ID (나중에 선택 가능하게 변경)
                
                # 중복된 버튼 ID들을 원래 이름으로 변환
                goal_location = self.button_id
                if self.button_id == 'D52':
                    goal_location = 'D5'  # D52 → D5로 변환
                elif self.button_id == 'D72':
                    goal_location = 'D7'  # D72 → D7로 변환
                
                request.goal_location = goal_location
                
                # 서비스 호출
                future = self.service_client.call_async(request)
                print(f"🎯 AddGoalLocation 서비스 호출: 로봇={request.robot_id}, 목표={request.goal_location} (원본 버튼: {self.button_id})")
                
                # 비동기 응답 처리 (간단한 로그만)
                future.add_done_callback(self.service_callback)
            else:
                print("❌ 서비스 클라이언트가 초기화되지 않음")
                print(f"🔍 디버그: service_client = {self.service_client}")
                
        except Exception as e:
            print(f"❌ 서비스 호출 실패: {e}")
    
    def service_callback(self, future):
        """서비스 응답 처리"""
        try:
            response = future.result()
            if response.success:
                print(f"✅ 서비스 성공: {response.message}")
            else:
                print(f"❌ 서비스 실패: {response.message}")
        except Exception as e:
            print(f"❌ 서비스 응답 처리 실패: {e}")
    
    def mouseDoubleClickEvent(self, event):
        """더블클릭 이벤트 무시 (한번 클릭만 작동하도록)"""
        event.accept()
    
    def create_click_animation(self):
        """클릭 시 빨간색 동그라미 애니메이션 생성"""
        try:
            # 기존 애니메이션 정리
            if self.animation_circle:
                self.scene().removeItem(self.animation_circle)
                self.animation_circle = None
            
            if self.animation_timer:
                self.animation_timer.stop()
                self.animation_timer = None
            
            # 빨간색 동그라미 생성 (중심점 기준)
            center_x = self.button_rect.center().x()
            center_y = self.button_rect.center().y()
            circle_size = 30  # 동그라미 크기
            
            self.animation_circle = QGraphicsEllipseItem(center_x - circle_size/2, center_y - circle_size/2, circle_size, circle_size)
            self.animation_circle.setPen(QPen(QColor(255, 0, 0), 3))  # 빨간색 테두리
            self.animation_circle.setBrush(QBrush(QColor(255, 0, 0, 100)))  # 반투명 빨간색
            
            # 씬에 추가
            if self.scene():
                self.scene().addItem(self.animation_circle)
            
            # 1초 후 애니메이션 제거
            self.animation_timer = QTimer()
            self.animation_timer.timeout.connect(self.remove_animation)
            self.animation_timer.start(1000)  # 1초
            
        except Exception as e:
            print(f"❌ 애니메이션 생성 실패: {e}")
    
    def remove_animation(self):
        """애니메이션 제거"""
        try:
            if self.animation_circle and self.scene():
                self.scene().removeItem(self.animation_circle)
                self.animation_circle = None
            
            if self.animation_timer:
                self.animation_timer.stop()
                self.animation_timer = None
                
        except Exception as e:
            print(f"❌ 애니메이션 제거 실패: {e}")

class MainViewTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node  # ROS 노드 저장
        
        # 맵 버튼 좌표 상수 정의 (중심점 기준)
        self.MAP_BUTTON_POSITIONS = {
            'D3': (637.0, 294.0),
            'C4': (685.0, 225.0),
            'E4': (685.0, 360.0),
            'D5': (737.0, 296.0),
            'D52': (798.0, 295.0),  # 두 번째 D5
            'C6': (846.0, 224.0),
            'E6': (844.0, 362.0),
            'D7': (891.0, 298.0),   # 첫 번째 D7
            'D72': (958.0, 312.0),  # 두 번째 D7
            'C8': (1027.0, 262.0),
            'E8': (1028.0, 352.0),
            'D9': (1100.0, 314.0),
            'kiosk1': (1115.0, 430.0),
            'kiosk2': (553.0, 200.0),
            'base': (572.0, 392.0),
            'admin': (222.0, 244.0)
        }
        
        # TaskStatus 관련 변수들
        self.task_status_data = {}  # 작업 상태 데이터 저장
        
        # RobotStatus 관련 변수들
        self.robot_status_dict = {}  # 로봇 상태 데이터 저장
        
        # 영상 수신 관련 변수들
        self.video_receiver = None  # 영상 수신 스레드
        self.current_frame = None  # 현재 프레임
        
        # Back camera 수신 관련 변수들
        self.video_receiver_back = None  # Back camera 수신 스레드
        self.current_frame_back = None  # Back camera 현재 프레임
        
        # 로봇 아이콘 관련 변수들
        self.robot_item = None  # 로봇 아이콘 아이템
        self.robot_speed = 2  # 로봇 이동 속도 (픽셀) - 더 부드럽게 하기 위해 줄임
        self.robot_rotation_speed = 2  # 로봇 회전 속도 (도)
        self.keys_pressed = set()  # 현재 눌린 키들 저장
        self.animation_timer = None  # 애니메이션 타이머
        
        # 실제 로봇 좌표 저장 변수들
        self.real_robot_x = 0.0  # 실제 로봇 X 좌표
        self.real_robot_y = 0.0  # 실제 로봇 Y 좌표
        self.real_robot_yaw = 0.0  # 실제 로봇 Yaw 각도
        self.real_robot_received = False  # 실제 로봇 좌표 수신 여부
        
        # 캘리브레이션을 위한 오프셋 변수들 (로봇 아이콘 중앙 기준)
        self.offset_x = 569.24 - 5 - 10  # X축 오프셋 (실제 0,0과 UI 0,0의 차이) - 5만큼 왼쪽 - 20픽셀 중앙 조정
        self.offset_y = 385.48 - 10 - 10  # Y축 오프셋 - 10만큼 위로 - 20픽셀 중앙 조정
        
        # 부드러운 움직임을 위한 보간 변수들
        self.target_ui_x = 0.0  # 목표 UI X 좌표
        self.target_ui_y = 0.0  # 목표 UI Y 좌표
        self.target_ui_rotation = 0.0  # 목표 UI 회전
        self.interpolation_factor = 0.8  # 보간 계수 (0.8로 높여서 더 빠르게)
        
        # AddGoalLocation 서비스 클라이언트
        self.add_goal_client = None
        
        self.init_ui()  # UI 초기화
        self.init_ros_connections()  # ROS 연결 초기화
        self.init_timers()  # 타이머 초기화
        self.init_video_receiver()  # 영상 수신 초기화
        
        # 맵 뷰에 배경 이미지 로드 (ROS 연결 초기화 후에 실행)
        self.load_map_background()
        
        # 키보드 이벤트 활성화
        self.setFocusPolicy(Qt.StrongFocus)
        self.map_view.setFocusPolicy(Qt.StrongFocus)
    
    def init_ui(self):
        """UI 초기화"""
        try:
            # main_view_tab.ui 파일 로드
            ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'main_view_tab.ui')
            uic.loadUi(ui_file_path, self)
            self.get_logger().info("✅ Main View Tab UI 로드 완료")
            
            # current_task_text 위젯 확인
            if hasattr(self, 'current_task_text'):
                self.get_logger().info("✅ current_task_text 위젯 확인됨")
                # 초기 텍스트 설정
                self.current_task_text.setPlainText("활성 작업 없음")
            else:
                self.get_logger().error("❌ current_task_text 위젯을 찾을 수 없음")
            
            # robot_list_text 위젯 확인
            if hasattr(self, 'robot_list_text'):
                self.get_logger().info("✅ robot_list_text 위젯 확인됨")
                # 초기 텍스트 설정
                self.robot_list_text.setPlainText("활성 로봇 없음")
            else:
                self.get_logger().error("❌ robot_list_text 위젯을 찾을 수 없음")
            
            # video_front 위젯 확인
            if hasattr(self, 'video_front'):
                self.get_logger().info("✅ video_front 위젯 확인됨")
                # 초기 텍스트 설정
                self.video_front.setText("영상 대기 중...")
            else:
                self.get_logger().error("❌ video_front 위젯을 찾을 수 없음")
            
            # video_back 위젯 확인
            if hasattr(self, 'video_back'):
                self.get_logger().info("✅ video_back 위젯 확인됨")
                # 초기 텍스트 설정
                self.video_back.setText("영상 대기 중...")
            else:
                self.get_logger().error("❌ video_back 위젯을 찾을 수 없음")
            
            # map_view에 마우스 클릭 이벤트 연결
            if hasattr(self, 'map_view'):
                self.map_view.mousePressEvent = self.map_view_mouse_press_event
                self.get_logger().info("✅ map_view 마우스 클릭 이벤트 연결 완료")
            else:
                self.get_logger().error("❌ map_view 위젯을 찾을 수 없음")
            
        except Exception as e:
            # UI 파일이 없을 경우 기본 레이아웃 설정
            self.get_logger().warning(f"⚠️ UI 파일 로드 실패: {e}, 기본 레이아웃 사용")
            self.layout = QVBoxLayout()
            self.setLayout(self.layout)
            
            # 임시 라벨 추가
            temp_label = QLabel("Main View Tab - 기능 구현 예정")
            temp_label.setAlignment(Qt.AlignCenter)
            self.layout.addWidget(temp_label)
    
    def init_ros_connections(self):
        """ROS 연결 초기화"""
        try:
            # TaskStatus 구독자
            self.task_status_subscription = self.ros_node.create_subscription(
                TaskStatus, 'task_status', self.task_status_callback, 10
            )
            self.get_logger().info("✅ TaskStatus 구독자 초기화 완료")
            
            # OverallStatus 구독자
            self.robot_status_subscription = self.ros_node.create_subscription(
                OverallStatus, 'robot_status', self.robot_status_callback, 10
            )
            self.get_logger().info("✅ OverallStatus 구독자 초기화 완료")

            # AMCL pose 구독자 추가
            self.amcl_pose_subscription = self.ros_node.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10
            )
            self.get_logger().info("✅ AMCL pose 구독자 초기화 완료")
            
            # AddGoalLocation 서비스 클라이언트
            self.get_logger().info("🔍 AddGoalLocation 서비스 클라이언트 생성 시도...")
            self.add_goal_client = self.ros_node.create_client(AddGoalLocation, 'add_goal_location')
            
            if self.add_goal_client:
                self.get_logger().info("✅ AddGoalLocation 서비스 클라이언트 초기화 완료")
                self.get_logger().info(f"🔍 서비스 클라이언트 생성됨: {self.add_goal_client}")
            else:
                self.get_logger().error("❌ 서비스 클라이언트 생성 실패 - None 반환")
                
        except Exception as e:
            self.get_logger().error(f"❌ ROS 연결 초기화 중 오류: {e}")
            import traceback
            self.get_logger().error(f"🔍 상세 오류: {traceback.format_exc()}")
        
        # 구독 확인을 위한 디버그 로그 추가
        self.get_logger().info("🔍 AMCL pose 구독 시작 - /amcl_pose 토픽 대기 중...")
    
    def init_timers(self):
        """타이머 초기화"""
        # 작업 상태 업데이트 타이머
        self.task_status_timer = QTimer()
        self.task_status_timer.timeout.connect(self.update_task_status_display)
        self.task_status_timer.start(1000)  # 1초마다
        self.get_logger().info("✅ 작업 상태 업데이트 타이머 시작됨")
        
        # 로봇 상태 업데이트 타이머
        self.robot_status_timer = QTimer()
        self.robot_status_timer.timeout.connect(self.update_robot_status_display)
        self.robot_status_timer.start(1000)  # 1초마다
        self.get_logger().info("✅ 로봇 상태 업데이트 타이머 시작됨")
        
        # 로봇 애니메이션 타이머 (부드러운 움직임용)
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update_robot_animation)
        self.animation_timer.start(16)  # 약 60 FPS (1000ms / 60 ≈ 16ms)
        self.get_logger().info("✅ 로봇 애니메이션 타이머 시작됨 (60 FPS)")
    
    def init_video_receiver(self):
        """영상 수신 초기화"""
        try:
            print("🎥 영상 수신 초기화 시작...")
            
            # Front camera 수신 스레드 생성 (포트 7021)
            self.video_receiver = VideoReceiverThread(port=7021)
            self.video_receiver.frame_received.connect(self.on_frame_received)
            self.video_receiver.start()
            
            # Back camera 수신 스레드 생성 (포트 7101)
            self.video_receiver_back = VideoReceiverThread(port=7101)
            self.video_receiver_back.frame_received.connect(self.on_frame_received_back)
            self.video_receiver_back.start()
            
            print("✅ Front camera 수신 스레드 시작됨 (포트: 7021)")
            print("✅ Back camera 수신 스레드 시작됨 (포트: 7101)")
            self.get_logger().info("✅ Front camera 수신 스레드 시작됨 (포트: 7021)")
            self.get_logger().info("✅ Back camera 수신 스레드 시작됨 (포트: 7101)")
            
        except Exception as e:
            print(f"❌ 영상 수신 초기화 실패: {e}")
            self.get_logger().error(f"❌ 영상 수신 초기화 실패: {e}")
    
    def on_frame_received(self, frame):
        """프레임 수신 처리"""
        try:
            # 이전 프레임 메모리 해제
            if hasattr(self, 'current_qimage'):
                del self.current_qimage
            if hasattr(self, 'current_pixmap'):
                del self.current_pixmap
            
            self.current_frame = frame
            
            # QImage로 변환
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            self.current_qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # QPixmap으로 변환
            self.current_pixmap = QPixmap.fromImage(self.current_qimage)
            
            # video_front 위젯에 표시
            if hasattr(self, 'video_front'):
                # 위젯 크기에 맞게 스케일링
                scaled_pixmap = self.current_pixmap.scaled(
                    self.video_front.size(), 
                    Qt.KeepAspectRatio, 
                    Qt.SmoothTransformation
                )
                self.video_front.setPixmap(scaled_pixmap)
            else:
                print("❌ video_front 위젯을 찾을 수 없음")
                
        except Exception as e:
            print(f"❌ 프레임 처리 중 오류: {e}")
            self.get_logger().error(f"❌ 프레임 처리 중 오류: {e}")
        finally:
            # 메모리 정리
            import gc
            gc.collect()
    
    def on_frame_received_back(self, frame):
        """Back camera 프레임 수신 처리"""
        try:
            # 이전 프레임 메모리 해제
            if hasattr(self, 'current_qimage_back'):
                del self.current_qimage_back
            if hasattr(self, 'current_pixmap_back'):
                del self.current_pixmap_back
            
            self.current_frame_back = frame
            
            # QImage로 변환
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            self.current_qimage_back = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # QPixmap으로 변환
            self.current_pixmap_back = QPixmap.fromImage(self.current_qimage_back)
            
            # video_back 위젯에 표시
            if hasattr(self, 'video_back'):
                # 위젯 크기에 맞게 스케일링
                scaled_pixmap = self.current_pixmap_back.scaled(
                    self.video_back.size(), 
                    Qt.KeepAspectRatio, 
                    Qt.SmoothTransformation
                )
                self.video_back.setPixmap(scaled_pixmap)
            else:
                print("❌ video_back 위젯을 찾을 수 없음")
                
        except Exception as e:
            print(f"❌ Back camera 프레임 처리 중 오류: {e}")
            self.get_logger().error(f"❌ Back camera 프레임 처리 중 오류: {e}")
        finally:
            # 메모리 정리
            import gc
            gc.collect()
    
    def load_map_background(self):
        """맵 뷰에 배경 이미지 로드"""
        try:
            # 이미지 파일 경로
            image_path = os.path.join(get_package_share_directory('admin'), 'resource', 'map_background_landscape_1170_white.png')
            
            if os.path.exists(image_path):
                # QGraphicsScene 생성
                scene = QGraphicsScene()
                
                # 이미지 로드
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    # QGraphicsPixmapItem 생성 및 씬에 추가
                    pixmap_item = QGraphicsPixmapItem(pixmap)
                    scene.addItem(pixmap_item)
                    
                    # 씬 크기를 이미지 크기에 맞춤 (QRect를 QRectF로 변환)
                    rect = pixmap.rect()
                    scene.setSceneRect(QRectF(rect))
                    
                    # 지도 크기와 위치 정보 로그 출력
                    self.get_logger().info(f"🗺️ 지도 정보: 크기={pixmap.width()}x{pixmap.height()}, 씬크기={scene.sceneRect().width()}x{scene.sceneRect().height()}")
                    
                    # 로봇 아이콘 추가 (지도 한가운데)
                    robot_icon_path = os.path.join(get_package_share_directory('admin'), 'resource', 'libo_full.png')
                    if os.path.exists(robot_icon_path):
                        robot_pixmap = QPixmap(robot_icon_path)
                        if not robot_pixmap.isNull():
                            # 로봇 아이콘 크기 조정 (너무 크지 않게)
                            robot_pixmap = robot_pixmap.scaled(40, 40, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                            
                            # 지도 한가운데 위치 계산
                            center_x = pixmap.width() / 2 - robot_pixmap.width() / 2
                            center_y = pixmap.height() / 2 - robot_pixmap.height() / 2
                            
                            # 로봇 아이콘 생성 및 위치 설정
                            self.robot_item = QGraphicsPixmapItem(robot_pixmap)
                            self.robot_item.setPos(center_x, center_y)
                            
                            # 로봇 아이콘의 중심점 설정 (회전 기준점)
                            # 아이콘 크기가 40x40이므로 중심점은 (20, 20)
                            self.robot_item.setTransformOriginPoint(20, 20)
                            
                            scene.addItem(self.robot_item)
                            
                            self.get_logger().info(f"✅ 로봇 아이콘 추가 완료 (지도 중앙: {center_x:.1f}, {center_y:.1f})")
                        else:
                            self.get_logger().error("❌ 로봇 아이콘 파일 로드 실패")
                    else:
                        self.get_logger().warning(f"⚠️ 로봇 아이콘 파일을 찾을 수 없음: {robot_icon_path}")
                    
                    # map_view에 씬 설정
                    self.map_view.setScene(scene)
                    
                    # 이미지가 뷰에 맞게 표시되도록 설정
                    self.map_view.setRenderHint(QPainter.Antialiasing)
                    self.map_view.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
                    
                    # 원본 사이즈로 표시 (fitInView 제거)
                    self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
                    self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
                    
                    # 맵 버튼들 배치
                    self.add_map_buttons(scene)
                    
                    self.get_logger().info("✅ 맵 배경 이미지 로드 완료")
                else:
                    self.get_logger().error("❌ 이미지 파일 로드 실패")
            else:
                self.get_logger().warning(f"⚠️ 이미지 파일을 찾을 수 없음: {image_path}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 맵 배경 로드 중 오류: {e}")
    
    def add_map_buttons(self, scene):
        """맵에 버튼들을 배치"""
        try:
            # 서비스 클라이언트 상태 확인
            if not self.add_goal_client:
                self.get_logger().warning("⚠️ 서비스 클라이언트가 None입니다. 버튼은 생성되지만 서비스 호출이 불가능합니다.")
            
            # 각 버튼 생성 및 씬에 추가
            for button_id, (x, y) in self.MAP_BUTTON_POSITIONS.items():
                button = MapButton(button_id, x, y, 40, 40, service_client=self.add_goal_client)
                scene.addItem(button)
                self.get_logger().debug(f"✅ 맵 버튼 추가: {button_id} ({x:.1f}, {y:.1f})")
            
            self.get_logger().info(f"✅ 맵 버튼 {len(self.MAP_BUTTON_POSITIONS)}개 배치 완료")
            
        except Exception as e:
            self.get_logger().error(f"❌ 맵 버튼 배치 중 오류: {e}")
    
    def robot_status_callback(self, msg):
        """OverallStatus 메시지 수신"""
        robot_id = msg.robot_id
        self.robot_status_dict[robot_id] = {
            'state': msg.robot_state,
            'is_available': msg.is_available,
            'battery': msg.battery,
            'book_weight': msg.book_weight,
            'position_x': msg.position_x,
            'position_y': msg.position_y,
            'position_yaw': msg.position_yaw,
            'timestamp': time.time()
        }
    
    def update_robot_status_display(self):
        """로봇 상태 표시 업데이트"""
        try:
            # robot_list_text가 존재하는지 확인
            if not hasattr(self, 'robot_list_text'):
                self.get_logger().error("❌ robot_list_text 위젯을 찾을 수 없음")
                return
            
            current_time = time.time()
            active_robots = []
            
            for robot_id, status in self.robot_status_dict.items():
                # 5초 이상 업데이트가 없으면 제거
                if current_time - status['timestamp'] > 5.0:
                    continue
                
                # 로봇 상태에 따라 사용 가능/불가 결정
                robot_state = status['state']
                if robot_state in ['INIT', 'CHARGING']:
                    available_text = "🔴 사용 불가"
                elif status['is_available']:
                    available_text = "🟢 사용가능"
                else:
                    available_text = "🔴 사용중"
                
                robot_info = f"🤖 {robot_id}\n"
                robot_info += f"   상태: {status['state']}\n"
                robot_info += f"   {available_text}\n"
                robot_info += f"   배터리: {status['battery']}%\n"
                weight_g = int(round(status.get('book_weight', 0.0) * 1000))
                robot_info += f"   무게: {weight_g}g\n"
                if weight_g > 3000:
                    robot_info += f"   ⚠️ 무게 한계 초과: {weight_g}g > 3000g\n"
                robot_info += f"   위치: ({status.get('position_x', 0.0):.1f}, {status.get('position_y', 0.0):.1f})\n"
                robot_info += f"   방향: {status.get('position_yaw', 0.0):.1f}°\n"
                robot_info += "─" * 20 + "\n"
                active_robots.append(robot_info)
            
            if active_robots:
                self.robot_list_text.setPlainText("".join(active_robots))
                self.get_logger().debug(f"✅ 로봇 상태 업데이트: {len(active_robots)}개 로봇 표시")
            else:
                self.robot_list_text.setPlainText("활성 로봇 없음")
                self.get_logger().debug("✅ 로봇 상태 업데이트: 활성 로봇 없음")
                
        except Exception as e:
            self.get_logger().error(f"❌ 로봇 상태 업데이트 중 오류: {e}")
    
    def task_status_callback(self, msg):
        """TaskStatus 메시지 수신"""
        task_id = msg.task_id
        self.task_status_data[task_id] = {
            'robot_id': msg.robot_id,
            'task_type': msg.task_type,
            'task_stage': msg.task_stage,
            'call_location': msg.call_location,
            'goal_location': msg.goal_location,
            'start_time': msg.start_time,
            'timestamp': time.time()
        }
    
    def update_task_status_display(self):
        """작업 상태 표시 업데이트"""
        try:
            # current_task_text가 존재하는지 확인
            if not hasattr(self, 'current_task_text'):
                self.get_logger().error("❌ current_task_text 위젯을 찾을 수 없음")
                return
            
            current_time = time.time()
            active_tasks = []
            
            for task_id, task in self.task_status_data.items():
                # 10초 이상 업데이트가 없으면 제거
                if current_time - task['timestamp'] > 10.0:
                    continue
                
                task_info = f"📋 Task[{task_id}]\n"
                task_info += f"   로봇: {task['robot_id']}\n"
                task_info += f"   타입: {task['task_type']}\n"
                task_info += f"   단계: {task['task_stage']}\n"
                task_info += f"   경로: {task['call_location']} → {task['goal_location']}\n"
                task_info += "─" * 20 + "\n"
                active_tasks.append(task_info)
            
            if active_tasks:
                self.current_task_text.setPlainText("".join(active_tasks))
                self.get_logger().debug(f"✅ 작업 상태 업데이트: {len(active_tasks)}개 작업 표시")
            else:
                self.current_task_text.setPlainText("활성 작업 없음")
                self.get_logger().debug("✅ 작업 상태 업데이트: 활성 작업 없음")
                
        except Exception as e:
            self.get_logger().error(f"❌ 작업 상태 업데이트 중 오류: {e}")
    
    def cleanup(self):
        """탭 정리 작업"""
        self.get_logger().info("🛑 Main View Tab 정리 중...")
        
        # 타이머 정리
        if hasattr(self, 'task_status_timer'):
            self.task_status_timer.stop()
            self.get_logger().info("✅ 작업 상태 업데이트 타이머 정지됨")
        
        if hasattr(self, 'robot_status_timer'):
            self.robot_status_timer.stop()
            self.get_logger().info("✅ 로봇 상태 업데이트 타이머 정지됨")
        
        # 로봇 애니메이션 타이머 정리
        if hasattr(self, 'animation_timer') and self.animation_timer:
            self.animation_timer.stop()
            self.get_logger().info("✅ 로봇 애니메이션 타이머 정지됨")
        
        # 영상 수신 스레드 정리
        if hasattr(self, 'video_receiver') and self.video_receiver:
            self.video_receiver.stop()
            self.get_logger().info("✅ Front camera 수신 스레드 정지됨")
        
        if hasattr(self, 'video_receiver_back') and self.video_receiver_back:
            self.video_receiver_back.stop()
            self.get_logger().info("✅ Back camera 수신 스레드 정지됨")
        
        # 메모리 정리
        if hasattr(self, 'current_frame'):
            del self.current_frame
        if hasattr(self, 'current_frame_back'):
            del self.current_frame_back
        if hasattr(self, 'current_qimage'):
            del self.current_qimage
        if hasattr(self, 'current_qimage_back'):
            del self.current_qimage_back
        if hasattr(self, 'current_pixmap'):
            del self.current_pixmap
        if hasattr(self, 'current_pixmap_back'):
            del self.current_pixmap_back
        
        # 가비지 컬렉션 실행
        import gc
        gc.collect()
        self.get_logger().info("✅ 메모리 정리 완료")
    
    def get_logger(self):
        """ROS 로거 반환"""
        return self.ros_node.get_logger() 
    
    def keyPressEvent(self, event):
        """키보드 이벤트 처리 - 키를 누를 때"""
        # WASD 키를 keys_pressed에 추가
        if event.key() == Qt.Key_W:
            self.keys_pressed.add('W')
            self.get_logger().info("⌨️ W 키 누름")
        elif event.key() == Qt.Key_S:
            self.keys_pressed.add('S')
            self.get_logger().info("⌨️ S 키 누름")
        elif event.key() == Qt.Key_A:
            self.keys_pressed.add('A')
            self.get_logger().info("⌨️ A 키 누름")
        elif event.key() == Qt.Key_D:
            self.keys_pressed.add('D')
            self.get_logger().info("⌨️ D 키 누름")
        elif event.key() == Qt.Key_Q:  # 왼쪽 회전
            self.keys_pressed.add('Q')
            self.get_logger().info("⌨️ Q 키 누름")
        elif event.key() == Qt.Key_E:  # 오른쪽 회전
            self.keys_pressed.add('E')
            self.get_logger().info("⌨️ E 키 누름")
        else:
            return
            
        # 이벤트 처리 완료
        event.accept()
    
    def keyReleaseEvent(self, event):
        """키보드 이벤트 처리 - 키를 뗄 때"""
        # WASD 키를 keys_pressed에서 제거
        if event.key() == Qt.Key_W:
            self.keys_pressed.discard('W')
        elif event.key() == Qt.Key_S:
            self.keys_pressed.discard('S')
        elif event.key() == Qt.Key_A:
            self.keys_pressed.discard('A')
        elif event.key() == Qt.Key_D:
            self.keys_pressed.discard('D')
        elif event.key() == Qt.Key_Q:  # 왼쪽 회전
            self.keys_pressed.discard('Q')
        elif event.key() == Qt.Key_E:  # 오른쪽 회전
            self.keys_pressed.discard('E')
        else:
            return
            
        # 이벤트 처리 완료
        event.accept()
    
    def update_robot_animation(self):
        """로봇 애니메이션 업데이트 (60 FPS) - 실제 로봇 좌표 우선"""
        if self.robot_item is None:
            return
            
        # 키보드 입력이 있으면 키보드 우선 (장난용)
        if self.keys_pressed:
            current_pos = self.robot_item.pos()
            new_x = current_pos.x()
            new_y = current_pos.y()
            moved = False
            rotated = False
            
            # 로봇의 현재 회전 각도 (라디안)
            current_rotation_rad = math.radians(self.robot_item.rotation())
            
            # 눌린 키에 따라 로봇 이동 (회전 방향 고려)
            if 'W' in self.keys_pressed:  # 앞으로 이동 (회전 방향 기준)
                # 회전된 방향으로 앞으로 이동
                new_x += self.robot_speed * math.sin(current_rotation_rad)
                new_y -= self.robot_speed * math.cos(current_rotation_rad)
                moved = True
            if 'S' in self.keys_pressed:  # 뒤로 이동 (회전 방향 기준)
                # 회전된 방향으로 뒤로 이동
                new_x -= self.robot_speed * math.sin(current_rotation_rad)
                new_y += self.robot_speed * math.cos(current_rotation_rad)
                moved = True
            if 'A' in self.keys_pressed:  # 왼쪽으로 이동 (항상 수평)
                new_x -= self.robot_speed
                moved = True
            if 'D' in self.keys_pressed:  # 오른쪽으로 이동 (항상 수평)
                new_x += self.robot_speed
                moved = True
            
            # 눌린 키에 따라 로봇 회전
            if 'Q' in self.keys_pressed:  # 왼쪽 회전
                current_rotation = self.robot_item.rotation()
                new_rotation = current_rotation - self.robot_rotation_speed
                self.robot_item.setRotation(new_rotation)
                rotated = True
            if 'E' in self.keys_pressed:  # 오른쪽 회전
                current_rotation = self.robot_item.rotation()
                new_rotation = current_rotation + self.robot_rotation_speed
                self.robot_item.setRotation(new_rotation)
                rotated = True
            
            # 새로운 위치 설정
            if moved:
                self.robot_item.setPos(new_x, new_y)
                
                # 키보드 이동 시에도 위치 로그 출력
                current_ui_pos = self.robot_item.pos()
                robot_center_x = current_ui_pos.x() + 20
                robot_center_y = current_ui_pos.y() + 20
                self.get_logger().info(f"⌨️ 키보드 이동: 모서리({current_ui_pos.x():.1f}, {current_ui_pos.y():.1f}), 중심점({robot_center_x:.1f}, {robot_center_y:.1f})")
            
            return
        
        # 실제 로봇 좌표가 수신되었으면 실제 좌표 사용
        if self.real_robot_received:
            # 스케일 팩터 (미터 → 픽셀 변환)
            # 실제 맵 크기: 19.69m x 7.95m
            # UI 맵 크기: 1170px x 480px
            # 비율: 가로 59.42, 세로 60.38 → 평균 60 픽셀/미터 사용
            scale_factor = 60  # 1미터 = 60픽셀 (정확한 비율)
            
            # 실제 로봇 좌표를 UI 좌표로 변환 (단순 테스트: 실제 X → UI X, 실제 Y → UI Y, X는 정방향, Y는 반대 방향)
            ui_x = self.real_robot_x * scale_factor + self.offset_x  # 실제 X를 UI X로 (양수로 정방향)
            ui_y = -self.real_robot_y * scale_factor + self.offset_y  # 실제 Y를 UI Y로 (음수로 반대 방향)
            
            # 실제 로봇 회전을 UI 회전으로 변환 (90도 회전, 방향 반대)
            ui_rotation = -math.degrees(self.real_robot_yaw) + 90
            
            # 목표 위치 업데이트 (부드러운 보간을 위해)
            self.target_ui_x = ui_x
            self.target_ui_y = ui_y
            self.target_ui_rotation = ui_rotation
            
            # 로봇 아이콘 위치와 회전 업데이트 (보간 적용)
            current_pos = self.robot_item.pos()
            current_rotation = self.robot_item.rotation()
            
            # 부드러운 보간 적용
            new_x = current_pos.x() + (self.target_ui_x - current_pos.x()) * self.interpolation_factor
            new_y = current_pos.y() + (self.target_ui_y - current_pos.y()) * self.interpolation_factor
            new_rotation = current_rotation + (self.target_ui_rotation - current_rotation) * self.interpolation_factor
            
            self.robot_item.setPos(new_x, new_y)
            self.robot_item.setRotation(new_rotation)
            
            # 현재 UI 좌표 확인 (캘리브레이션용)
            current_ui_pos = self.robot_item.pos()
            
            # 로봇 아이콘의 중심점 계산 (아이콘 크기: 40x40)
            robot_center_x = current_ui_pos.x() + 20  # 왼쪽 모서리 + 아이콘 반지름
            robot_center_y = current_ui_pos.y() + 20  # 위쪽 모서리 + 아이콘 반지름
            
            self.get_logger().debug(f"📍 현재 UI 좌표: 모서리({current_ui_pos.x():.1f}, {current_ui_pos.y():.1f}), 중심점({robot_center_x:.1f}, {robot_center_y:.1f})")
            
            return
    
    def amcl_pose_callback(self, msg):
        """AMCL로부터 로봇의 실제 위치를 받아서 UI 업데이트"""
        try:
            # AMCL에서 받은 실제 로봇 좌표
            real_x = msg.pose.pose.position.x
            real_y = msg.pose.pose.position.y
            
            # 쿼터니언에서 Yaw 각도 추출 (회전 방향)
            orientation = msg.pose.pose.orientation
            yaw = self.quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
            
            # 실제 로봇 좌표 저장
            self.real_robot_x = real_x
            self.real_robot_y = real_y
            self.real_robot_yaw = yaw
            self.real_robot_received = True
            
            # 로그로 출력 (첫 번째 단계) - debug 레벨로 변경
            self.get_logger().debug(f"🤖 AMCL 좌표 수신: X={real_x:.2f}, Y={real_y:.2f}, Yaw={math.degrees(yaw):.1f}°")
            
        except Exception as e:
            self.get_logger().error(f"AMCL 좌표 처리 중 오류: {e}")
    
    def quaternion_to_yaw(self, x, y, z, w):
        """쿼터니언을 Yaw 각도(라디안)로 변환"""
        # Z축 회전 (Yaw) 계산
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    def map_view_mouse_press_event(self, event):
        """map_view 마우스 클릭 이벤트 처리"""
        # 스크롤 클릭(middle click)일 때만 좌표 출력
        if event.button() == Qt.MiddleButton:
            # 클릭한 위치를 씬 좌표로 변환
            scene_pos = self.map_view.mapToScene(event.pos())
            
            # 클릭한 좌표를 터미널 로그로 출력
            self.get_logger().info(f"🗺️ 맵 스크롤 클릭: X={scene_pos.x():.1f}, Y={scene_pos.y():.1f}")
        
        # 이벤트 처리 완료
        event.accept()