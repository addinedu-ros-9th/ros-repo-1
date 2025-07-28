#!/usr/bin/env python3
# 도서 등록 서비스 (aladin_book_register.py)
# ROS2 노드로 알라딘 API → DB 저장 통합
# 연결 테스트 및 오류 처리
# 1단계 특정 도서 등록 기능
import rclpy
from rclpy.node import Node
import sys
import os

# 상위 디렉토리 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'database'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))

from aladin_api_client import AladinAPIClient
from book_data_parser import BookDataParser
from db_manager import DatabaseManager

class AladinBookRegisterService(Node):
    def __init__(self):
        super().__init__('aladin_book_register_service')
        
        # 컴포넌트 초기화
        self.aladin_client = AladinAPIClient("ttbleeshun08062356001")
        self.parser = BookDataParser()
        self.db_manager = DatabaseManager()
        
        self.get_logger().info('📚 Aladin Book Register Service 시작됨')
        
        # 연결 테스트
        self._test_connections()
    
    def _test_connections(self):
        """API 및 DB 연결 테스트"""
        # 알라딘 API 연결 테스트
        if not self.aladin_client.test_connection():
            self.get_logger().error('❌ 알라딘 API 연결 실패')
            return
        
        # DB 연결 테스트
        if not self.db_manager.test_connection():
            self.get_logger().error('❌ 데이터베이스 연결 실패')
            return
        
        self.get_logger().info('✅ 모든 연결 테스트 통과')
    
    def register_book_from_aladin(self, book_title: str) -> bool:
        """
        알라딘 API를 통해 도서 등록
        
        Args:
            book_title: 등록할 도서 제목
            
        Returns:
            bool: 등록 성공 여부
        """
        try:
            self.get_logger().info(f'🔍 도서 검색 시작: {book_title}')
            
            # 1. 알라딘 API에서 도서 검색
            book_info = self.aladin_client.search_specific_book(book_title)
            
            if not book_info:
                self.get_logger().error(f'❌ 도서를 찾을 수 없습니다: {book_title}')
                return False
            
            # 2. 응답 데이터 파싱
            self.get_logger().info('📋 도서 정보 파싱 중...')
            parsed_book = self.parser.parse_aladin_response(book_info)
            
            if not parsed_book:
                self.get_logger().error('❌ 도서 정보 파싱 실패')
                return False
            
            # 3. 데이터 유효성 검증
            if not self.parser.validate_parsed_data(parsed_book):
                self.get_logger().error('❌ 파싱된 데이터가 유효하지 않습니다')
                return False
            
            # 4. 데이터베이스에 등록
            self.get_logger().info('💾 데이터베이스에 도서 등록 중...')
            success = self.db_manager.register_book(parsed_book)
            
            if success:
                self.get_logger().info(f'✅ 도서 등록 완료: {parsed_book["title"]}')
                return True
            else:
                self.get_logger().error('❌ 데이터베이스 등록 실패')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ 도서 등록 중 오류 발생: {str(e)}')
            return False
    
    def register_book_from_aladin_with_author(self, book_title: str, author: str = None) -> bool:
        """
        알라딘 API를 통해 도서 등록 (저자명 포함)
        
        Args:
            book_title: 등록할 도서 제목
            author: 저자명 (선택사항)
            
        Returns:
            bool: 등록 성공 여부
        """
        try:
            self.get_logger().info(f'🔍 도서 검색 시작: {book_title}')
            if author:
                self.get_logger().info(f'   저자: {author}')
            
            # 1. 알라딘 API에서 도서 검색 (저자명 포함)
            book_info = self.aladin_client.search_specific_book_with_author(book_title, author)
            
            if not book_info:
                self.get_logger().error(f'❌ 도서를 찾을 수 없습니다: {book_title}')
                return False
            
            # 2. 응답 데이터 파싱
            self.get_logger().info('📋 도서 정보 파싱 중...')
            parsed_book = self.parser.parse_aladin_response(book_info)
            
            if not parsed_book:
                self.get_logger().error('❌ 도서 정보 파싱 실패')
                return False
            
            # 3. 데이터 유효성 검증
            if not self.parser.validate_parsed_data(parsed_book):
                self.get_logger().error('❌ 파싱된 데이터가 유효하지 않습니다')
                return False
            
            # 4. 데이터베이스에 등록
            self.get_logger().info('�� 데이터베이스에 도서 등록 중...')
            success = self.db_manager.register_book(parsed_book)
            
            if success:
                self.get_logger().info(f'✅ 도서 등록 완료: {parsed_book["title"]}')
                return True
            else:
                self.get_logger().error('❌ 데이터베이스 등록 실패')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ 도서 등록 중 오류 발생: {str(e)}')
            return False

    def register_specific_book(self):
        """1단계용: 특정 도서 등록"""
        book_title = "밑바닥부터 시작하는 딥러닝 1"
        author = "사이토 고키"  # 저자명 추가
        
        self.get_logger().info(f'📚 1단계 도서 등록 시작: {book_title}')
        self.get_logger().info(f'   저자: {author}')
        
        success = self.register_book_from_aladin_with_author(book_title, author)
        
        if success:
            self.get_logger().info('🎉 1단계 도서 등록 완료!')
        else:
            self.get_logger().error('❌ 1단계 도서 등록 실패')



def main(args=None):
    rclpy.init(args=args)
    
    register_service = AladinBookRegisterService()
    
    try:
        # 1단계 도서 등록 실행
        register_service.register_specific_book()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"❌ 서비스 실행 중 오류: {e}")
    finally:
        if 'register_service' in locals():
            register_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 