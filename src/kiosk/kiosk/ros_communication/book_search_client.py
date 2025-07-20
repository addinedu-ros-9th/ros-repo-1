#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import BookSearch
from PyQt5.QtCore import QThread, pyqtSignal

class BookSearchClient(QThread):
    # 검색 완료 시그널
    search_completed = pyqtSignal(bool, str, list)  # success, message, books
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.client = None
        self.query = ""
        self.search_type = ""
        
    def init_ros(self):
        """ROS2 초기화"""
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node('kiosk_book_search_client')
        self.client = self.node.create_client(BookSearch, 'book_search')
        
        # 서비스 서버 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('📡 book_search 서비스 대기 중...')
    
    def search_books(self, query, search_type):
        """도서 검색 요청"""
        self.query = query
        self.search_type = search_type
        self.start()  # QThread 시작
    
    def run(self):
        """QThread 실행 (백그라운드에서 ROS2 서비스 호출)"""
        try:
            if self.node is None:
                self.init_ros()
            
            # 서비스 요청 생성
            request = BookSearch.Request()
            request.query = self.query
            request.search_type = self.search_type
            
            self.node.get_logger().info(f'🔍 검색 요청: "{self.query}" ({self.search_type})')
            
            # 서비스 호출
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                response = future.result()
                
                # 응답 데이터를 Python dict로 변환
                books = []
                for book_msg in response.books:
                    book = {
                        'id': book_msg.id,
                        'title': book_msg.title,
                        'author': book_msg.author,
                        'publisher': book_msg.publisher,
                        'category_name': book_msg.category_name,
                        'location': book_msg.location,
                        'price': book_msg.price,
                        'stock_quantity': book_msg.stock_quantity,
                        'isbn': book_msg.isbn,
                        'cover_image_url': book_msg.cover_image_url
                    }
                    books.append(book)
                
                # 결과 시그널 발생
                self.search_completed.emit(response.success, response.message, books)
                
            else:
                self.search_completed.emit(False, "서비스 호출 실패", [])
                
        except Exception as e:
            self.search_completed.emit(False, f"오류: {str(e)}", [])
    
    def cleanup(self):
        """리소스 정리"""
        if self.node:
            self.node.destroy_node()