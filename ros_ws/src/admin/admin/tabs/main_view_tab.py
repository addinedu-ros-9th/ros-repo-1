#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGraphicsScene, QGraphicsPixmapItem, QGraphicsView
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPixmap, QPainter
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

class MainViewTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node  # ROS 노드 저장
        
        self.init_ui()  # UI 초기화
        
    def init_ui(self):
        """UI 초기화"""
        try:
            # main_view_tab.ui 파일 로드
            ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'main_view_tab.ui')
            uic.loadUi(ui_file_path, self)
            self.get_logger().info("✅ Main View Tab UI 로드 완료")
            
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
    
    def cleanup(self):
        """탭 정리 작업"""
        self.get_logger().info("🛑 Main View Tab 정리 중...")
    
    def get_logger(self):
        """ROS 로거 반환"""
        return self.ros_node.get_logger() 