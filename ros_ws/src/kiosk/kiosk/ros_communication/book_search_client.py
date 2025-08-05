#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import BookSearch
from PyQt5.QtCore import QThread, pyqtSignal
import threading
import time

class BookSearchClient(QThread):
    # 검색 완료 시그널
    search_completed = pyqtSignal(bool, str, list)  # success, message, books
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.client = None
        self.query = ""
        self.search_type = ""
        self._lock = threading.Lock()  # 스레드 안전성을 위한 락
        self._is_cleaning_up = False
        self._node_initialized = False
        self.max_retries = 3  # 최대 재시도 횟수
        
    def init_ros(self):
        """ROS2 초기화"""
        try:
            with self._lock:
                if self._is_cleaning_up:
                    return False
                
                if not rclpy.ok():
                    rclpy.init()
                
                if self.node is None:
                    self.node = Node('kiosk_book_search_client')
                    self.client = self.node.create_client(BookSearch, 'book_search')
                
                # 서비스 서버 대기 (타임아웃 설정)
                timeout_count = 0
                max_timeout = 10  # 최대 10초 대기
                
                while not self.client.wait_for_service(timeout_sec=1.0):
                    if self._is_cleaning_up:
                        return False
                    
                    self.node.get_logger().info('📡 book_search 서비스 대기 중...')
                    timeout_count += 1
                    
                    if timeout_count >= max_timeout:
                        print("❌ book_search 서비스 서버를 찾을 수 없습니다.")
                        return False
                
                self._node_initialized = True
                print("✅ BookSearchClient ROS2 초기화 완료")
                return True
                
        except Exception as e:
            print(f"❌ BookSearchClient ROS2 초기화 실패: {e}")
            return False
    
    def search_books(self, query, search_type):
        """도서 검색 요청"""
        self.query = query
        self.search_type = search_type
        self.start()  # QThread 시작
    
    def call_service_with_retry(self):
        """재시도 메커니즘을 포함한 서비스 호출"""
        for attempt in range(self.max_retries):
            try:
                print(f"🔄 서비스 호출 시도 {attempt + 1}/{self.max_retries}")
                
                # 서비스 요청 생성
                request = BookSearch.Request()
                request.query = self.query
                request.search_type = self.search_type
                
                # 서비스 호출
                future = self.client.call_async(request)
                
                # 안전한 타임아웃 처리
                timeout_seconds = 15.0  # 각 시도마다 15초 타임아웃
                start_time = time.time()
                
                while not future.done():
                    if time.time() - start_time > timeout_seconds:
                        print(f"❌ 시도 {attempt + 1} 타임아웃")
                        break
                    
                    # 짧은 간격으로 스핀
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception as spin_error:
                        print(f"⚠️ spin_once 오류 (무시): {spin_error}")
                        time.sleep(0.1)  # 잠시 대기
                        continue
                
                # 결과 확인
                if future.done():
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
                        
                        # 성공 시 결과 반환
                        return True, response.success, response.message, books
                    else:
                        print(f"❌ 시도 {attempt + 1} 실패: 서비스 응답 없음")
                else:
                    print(f"❌ 시도 {attempt + 1} 실패: 타임아웃")
                
                # 재시도 전 잠시 대기 (점진적 백오프)
                if attempt < self.max_retries - 1:
                    wait_time = (attempt + 1) * 2  # 2초, 4초, 6초
                    print(f"⏳ {wait_time}초 후 재시도...")
                    time.sleep(wait_time)
                    
            except Exception as e:
                print(f"❌ 시도 {attempt + 1} 중 오류: {e}")
                if attempt < self.max_retries - 1:
                    wait_time = (attempt + 1) * 2
                    print(f"⏳ {wait_time}초 후 재시도...")
                    time.sleep(wait_time)
        
        # 모든 시도 실패
        return False, False, "모든 재시도 실패", []
    
    def run(self):
        """QThread 실행 (백그라운드에서 ROS2 서비스 호출)"""
        try:
            # ROS2 초기화
            if not self._node_initialized:
                if not self.init_ros():
                    self.search_completed.emit(False, "ROS2 초기화 실패", [])
                    return
            
            if self.node is None or self.client is None:
                self.search_completed.emit(False, "ROS2 클라이언트 초기화 실패", [])
                return
            
            print(f'🔍 검색 요청: "{self.query}" ({self.search_type})')
            
            # 재시도 메커니즘을 포함한 서비스 호출
            success, service_success, message, books = self.call_service_with_retry()
            
            if success:
                # 결과 시그널 발생
                self.search_completed.emit(service_success, message, books)
            else:
                self.search_completed.emit(False, message, [])
                
        except Exception as e:
            print(f"❌ 검색 중 오류: {e}")
            self.search_completed.emit(False, f"오류: {str(e)}", [])
    
    def cleanup(self):
        """리소스 정리"""
        with self._lock:
            self._is_cleaning_up = True
        
        # 스레드가 실행 중이면 종료 대기
        if self.isRunning():
            self.quit()
            self.wait(3000)  # 3초 대기
        
        try:
            if self.client:
                self.client.destroy()
                self.client = None
        except Exception as e:
            print(f"⚠️ search_client client 정리 중 오류: {e}")
        
        try:
            if self.node:
                # 노드가 유효한지 확인
                try:
                    if hasattr(self.node, 'get_name') and self.node.get_name():
                        self.node.destroy_node()
                except Exception as e:
                    print(f"⚠️ search_client node 정리 중 오류: {e}")
                finally:
                    self.node = None
                    self._node_initialized = False
        except Exception as e:
            print(f"⚠️ search_client cleanup 중 오류: {e}")
        
        print("✅ BookSearchClient 리소스 정리 완료")