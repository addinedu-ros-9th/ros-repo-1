#!/usr/bin/env python3

# 데이터 파서 (book_data_parser.py)
# 알라딘 API 응답을 DB 스키마에 맞게 변환
# 저자명 정리, 가격 추출, 카테고리 결정
# 데이터 유효성 검증
from typing import Dict, Optional, List
import re

class BookDataParser:
    """알라딘 API 응답을 DB 스키마에 맞게 변환하는 파서"""
    
    def __init__(self):
        """파서 초기화"""
        # 카테고리 매핑 (알라딘 카테고리 → 우리 DB 카테고리)
        self.category_mapping = {
            '컴퓨터/IT': '컴퓨터',
            '컴퓨터': '컴퓨터',
            '프로그래밍': '컴퓨터',
            '소프트웨어': '컴퓨터',
            '언어': '언어',
            '외국어': '언어',
            '영어': '언어',
            '소설': '소설',
            '문학': '소설',
            '시/에세이': '소설'
        }
        
        # 위치 매핑 (카테고리별 기본 위치)
        self.location_mapping = {
            '컴퓨터': 'D3',  # 컴퓨터 서적은 D3 또는 D5
            '언어': 'B',
            '소설': 'C'
        }
    
    def parse_aladin_response(self, book_info: Dict) -> Optional[Dict]:
        """
        알라딘 API 응답을 DB 스키마에 맞게 파싱
        
        Args:
            book_info (dict): 알라딘 API 응답의 book 정보
            
        Returns:
            dict: DB에 저장할 도서 정보 또는 None
        """
        if not book_info:
            print("❌ 파싱할 도서 정보가 없습니다.")
            return None
        
        try:
            # 기본 정보 추출
            title = book_info.get('title', '').strip()
            author = self._extract_author(book_info.get('author', ''))
            publisher = book_info.get('publisher', '').strip()
            isbn = book_info.get('isbn13', book_info.get('isbn', ''))
            price = self._extract_price(book_info.get('priceSales', 0))
            cover_url = book_info.get('cover', '')
            description = book_info.get('description', '')
            
            # 카테고리 결정
            category = self._determine_category(book_info)
            
            # 위치 결정
            location = self._determine_location(category)
            
            # 재고 수량 (기본값: 1)
            stock_quantity = 1
            
            # 파싱된 데이터 구성
            parsed_book = {
                'title': title,
                'author': author,
                'publisher': publisher,
                'category_name': category,
                'location': location,
                'price': price,
                'stock_quantity': stock_quantity,
                'isbn': isbn,
                'cover_image_url': cover_url,
                'description': description
            }
            
            print(f"✅ 도서 정보 파싱 완료:")
            print(f"   제목: {title}")
            print(f"   저자: {author}")
            print(f"   출판사: {publisher}")
            print(f"   카테고리: {category}")
            print(f"   위치: {location}")
            print(f"   가격: {price:,}원")
            print(f"   ISBN: {isbn}")
            
            return parsed_book
            
        except Exception as e:
            print(f"❌ 도서 정보 파싱 중 오류 발생: {e}")
            return None
    
    def _extract_author(self, author_str: str) -> str:
        """
        저자 정보에서 실제 저자명만 추출
        
        Args:
            author_str (str): 알라딘 API의 저자 문자열
            
        Returns:
            str: 정리된 저자명
        """
        if not author_str:
            return ''
        
        # 괄호와 설명 제거 (예: "사이토 고키 (지은이), 개앞맵시(이복연) (옮긴이)" → "사이토 고키")
        author = re.sub(r'\s*\([^)]*\)', '', author_str)
        
        # 쉼표로 구분된 경우 첫 번째 저자만 사용
        if ',' in author:
            author = author.split(',')[0].strip()
        
        return author.strip()
    
    def _extract_price(self, price_value) -> float:
        """
        가격 정보 정리
        
        Args:
            price_value: 알라딘 API의 가격 값
            
        Returns:
            float: 정리된 가격
        """
        if not price_value:
            return 0.0
        
        # 문자열인 경우 숫자만 추출
        if isinstance(price_value, str):
            price_str = re.sub(r'[^\d]', '', price_value)
            return float(price_str) if price_str else 0.0
        
        # 숫자인 경우 그대로 반환
        return float(price_value)
    
    def _determine_category(self, book_info: Dict) -> str:
        """
        도서 카테고리 결정
        
        Args:
            book_info (dict): 도서 정보
            
        Returns:
            str: 결정된 카테고리
        """
        # 1. 제목 기반 카테고리 추정
        title = book_info.get('title', '').lower()
        
        # 컴퓨터 관련 키워드
        computer_keywords = ['딥러닝', '머신러닝', '파이썬', '프로그래밍', '코딩', '알고리즘', 
                           '데이터', '인공지능', 'ai', '개발', '소프트웨어', 'it', '컴퓨터']
        
        # 언어 관련 키워드
        language_keywords = ['영어', '토익', '토플', '회화', '문법', '단어', '외국어', '중국어', '일본어']
        
        # 소설 관련 키워드
        novel_keywords = ['소설', '문학', '시', '에세이', '시집', '수필']
        
        # 제목 기반 분류
        for keyword in computer_keywords:
            if keyword in title:
                return '컴퓨터'
        
        for keyword in language_keywords:
            if keyword in title:
                return '언어'
        
        for keyword in novel_keywords:
            if keyword in title:
                return '소설'
        
        # 2. 기본값 (컴퓨터로 설정)
        return '컴퓨터'
    
    def _determine_location(self, category: str) -> str:
        """
        카테고리별 위치 결정
        
        Args:
            category (str): 도서 카테고리
            
        Returns:
            str: 위치 코드
        """
        return self.location_mapping.get(category, 'D3')
    
    def validate_parsed_data(self, parsed_book: Dict) -> bool:
        """
        파싱된 데이터 유효성 검증
        
        Args:
            parsed_book (dict): 파싱된 도서 정보
            
        Returns:
            bool: 유효성 여부
        """
        required_fields = ['title', 'author', 'publisher', 'isbn']
        
        for field in required_fields:
            if not parsed_book.get(field):
                print(f"❌ 필수 필드 누락: {field}")
                return False
        
        # ISBN 형식 검증
        isbn = parsed_book.get('isbn', '')
        if not re.match(r'^\d{10,13}$', isbn):
            print(f"❌ 잘못된 ISBN 형식: {isbn}")
            return False
        
        # 가격 검증
        price = parsed_book.get('price', 0)
        if price <= 0:
            print(f"❌ 잘못된 가격: {price}")
            return False
        
        print("✅ 파싱된 데이터 유효성 검증 통과")
        return True

# 사용 예시
def main():
    """테스트용 메인 함수"""
    import sys
    import os
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'database'))
    
    from aladin_api_client import AladinAPIClient
    
    # API 클라이언트 생성
    aladin = AladinAPIClient("ttbleeshun08062356001")
    
    # 도서 검색
    book_info = aladin.search_specific_book("밑바닥부터 시작하는 딥러닝 1")
    
    if book_info:
        # 파서 생성
        parser = BookDataParser()
        
        # 데이터 파싱
        parsed_book = parser.parse_aladin_response(book_info)
        
        if parsed_book:
            # 유효성 검증
            if parser.validate_parsed_data(parsed_book):
                print("\n📋 최종 파싱 결과:")
                for key, value in parsed_book.items():
                    print(f"  {key}: {value}")
            else:
                print("❌ 파싱된 데이터가 유효하지 않습니다.")

if __name__ == "__main__":
    main() 