#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import BookSearch
from libo_interfaces.msg import BookInfo

# ROS2 패키지 경로로 import
from main_server.database.db_manager import DatabaseManager

class BookSearchService(Node):
    def __init__(self):
        super().__init__('book_search_service')
        
        # 데이터베이스 매니저 초기화
        self.db_manager = DatabaseManager()
        
        # 서비스 서버 생성
        self.service = self.create_service(
            BookSearch,
            'book_search',
            self.book_search_callback
        )
        
        self.get_logger().info('📚 Book Search Service 시작됨')
    
    def book_search_callback(self, request, response):
        """도서 검색 서비스 콜백"""
        self.get_logger().info(f'🔍 검색 요청: "{request.query}" ({request.search_type})')
        
        try:
            # 데이터베이스 연결 확인
            if not self.db_manager.connection:
                self.get_logger().error('❌ 데이터베이스 연결이 없습니다.')
                response.success = False
                response.message = '데이터베이스 연결 오류'
                response.books = []
                return response
            
            # 데이터베이스에서 검색
            search_results = self.db_manager.search_books(
                request.query, 
                request.search_type
            )
            
            # 검색 결과를 ROS2 메시지로 변환
            book_list = []
            for book_data in search_results:
                try:
                    book_info = BookInfo()
                    book_info.id = book_data['id']
                    book_info.title = book_data['title']
                    book_info.author = book_data['author'] or ''
                    book_info.publisher = book_data['publisher'] or ''
                    book_info.category_name = book_data['category_name']
                    book_info.location = book_data['location']
                    book_info.price = float(book_data['price']) if book_data['price'] else 0.0
                    book_info.stock_quantity = book_data['stock_quantity']
                    book_info.isbn = book_data['isbn'] or ''
                    book_info.cover_image_url = book_data['cover_image_url'] or ''
                    
                    book_list.append(book_info)
                except Exception as book_error:
                    self.get_logger().warning(f'⚠️ 책 데이터 변환 오류: {book_error}')
                    continue
            
            # 응답 설정
            response.success = True
            response.message = f'{len(book_list)}권의 도서를 찾았습니다.'
            response.books = book_list
            
            self.get_logger().info(f'✅ 검색 완료: {len(book_list)}권 발견')
            
        except Exception as e:
            # 오류 처리
            self.get_logger().error(f'❌ 검색 오류: {str(e)}')
            response.success = False
            response.message = f'오류: {str(e)}'
            response.books = []
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    book_search_service = BookSearchService()
    
    try:
        rclpy.spin(book_search_service)
    except KeyboardInterrupt:
        pass
    finally:
        book_search_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()