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
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGraphicsScene, QGraphicsPixmapItem, QGraphicsView
from PyQt5.QtCore import Qt, QRectF, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QPainter, QImage
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# TaskStatus 메시지 import
from libo_interfaces.msg import TaskStatus, OverallStatus

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
                                
                                print(f"📥 프레임 수신: {direction}, ID: {frame_id}, 크기: {len(image_data)} bytes")
                                
                                # JPEG 디코딩
                                nparr = np.frombuffer(image_data, np.uint8)
                                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                                
                                if frame is not None:
                                    # BGR to RGB 변환 (OpenCV는 BGR, Qt는 RGB)
                                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    self.frame_received.emit(frame_rgb)
                                    frame_count += 1
                                    
                                    if frame_count % 30 == 0:  # 30프레임마다 로그
                                        print(f"✅ 프레임 처리 완료: {frame_count}개")
                                    
                                    # 100프레임마다 메모리 정리
                                    if frame_count % 100 == 0:
                                        import gc
                                        gc.collect()
                                        print(f"🧹 메모리 정리 완료 (프레임: {frame_count})")
                                else:
                                    print("❌ 프레임 디코딩 실패")
                                    
                            except (json.JSONDecodeError, cv2.error) as e:
                                print(f"⚠️ 프레임 파싱 오류: {e}")
                                
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

class MainViewTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node  # ROS 노드 저장
        
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
        
        self.init_ui()  # UI 초기화
        self.init_ros_connections()  # ROS 연결 초기화
        self.init_timers()  # 타이머 초기화
        self.init_video_receiver()  # 영상 수신 초기화
        
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
            
            # 맵 뷰에 배경 이미지 로드
            self.load_map_background()
            
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
    
    def init_video_receiver(self):
        """영상 수신 초기화"""
        try:
            print("🎥 영상 수신 초기화 시작...")
            
            # Front camera 수신 스레드 생성 (포트 7021)
            self.video_receiver = VideoReceiverThread(port=7021)
            self.video_receiver.frame_received.connect(self.on_frame_received)
            self.video_receiver.start()
            
            # Back camera 수신 스레드 생성 (포트 7020)
            self.video_receiver_back = VideoReceiverThread(port=7020)
            self.video_receiver_back.frame_received.connect(self.on_frame_received_back)
            self.video_receiver_back.start()
            
            print("✅ Front camera 수신 스레드 시작됨 (포트: 7021)")
            print("✅ Back camera 수신 스레드 시작됨 (포트: 7020)")
            self.get_logger().info("✅ Front camera 수신 스레드 시작됨 (포트: 7021)")
            self.get_logger().info("✅ Back camera 수신 스레드 시작됨 (포트: 7020)")
            
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
                print(f"🎬 영상 표시 완료: {width}x{height} -> {scaled_pixmap.width()}x{scaled_pixmap.height()}")
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
                print(f"🎬 Back camera 영상 표시 완료: {width}x{height} -> {scaled_pixmap.width()}x{scaled_pixmap.height()}")
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
            image_path = os.path.join(get_package_share_directory('admin'), 'resource', 'map_background_landscape_1170.png')
            
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
                    
                    # map_view에 씬 설정
                    self.map_view.setScene(scene)
                    
                    # 이미지가 뷰에 맞게 표시되도록 설정
                    self.map_view.setRenderHint(QPainter.Antialiasing)
                    self.map_view.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
                    
                    # 원본 사이즈로 표시 (fitInView 제거)
                    self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
                    self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
                    
                    self.get_logger().info("✅ 맵 배경 이미지 로드 완료")
                else:
                    self.get_logger().error("❌ 이미지 파일 로드 실패")
            else:
                self.get_logger().warning(f"⚠️ 이미지 파일을 찾을 수 없음: {image_path}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 맵 배경 로드 중 오류: {e}")
    
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
                robot_info += f"   무게: {status.get('book_weight', 0.0):.1f}kg\n"
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