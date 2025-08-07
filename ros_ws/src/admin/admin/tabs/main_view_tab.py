#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGraphicsScene, QGraphicsPixmapItem, QGraphicsView
from PyQt5.QtCore import Qt, QRectF, QTimer
from PyQt5.QtGui import QPixmap, QPainter
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# TaskStatus 메시지 import
from libo_interfaces.msg import TaskStatus

class MainViewTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node  # ROS 노드 저장
        
        # TaskStatus 관련 변수들
        self.task_status_data = {}  # 작업 상태 데이터 저장
        
        self.init_ui()  # UI 초기화
        self.init_ros_connections()  # ROS 연결 초기화
        self.init_timers()  # 타이머 초기화
        
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
    
    def init_timers(self):
        """타이머 초기화"""
        # 작업 상태 업데이트 타이머
        self.task_status_timer = QTimer()
        self.task_status_timer.timeout.connect(self.update_task_status_display)
        self.task_status_timer.start(1000)  # 1초마다
        self.get_logger().info("✅ 작업 상태 업데이트 타이머 시작됨")
    
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
    
    def get_logger(self):
        """ROS 로거 반환"""
        return self.ros_node.get_logger() 