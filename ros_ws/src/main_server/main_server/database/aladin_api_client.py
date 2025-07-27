#!/usr/bin/env python3

# 알라딘 API 호출 및 응답 처리
# TTBKey 인증 및 연결 테스트
# 특정 도서 검색 기능
import requests
import json
from typing import Dict, List, Optional, Any
from urllib.parse import quote
import time

class AladinAPIClient:
    def __init__(self, ttb_key: str):
        """
        알라딘 API 클라이언트 초기화
        
        Args:
            ttb_key (str): 알라딘에서 발급받은 TTBKey
        """
        self.ttb_key = ttb_key
        self.base_url = "http://www.aladin.co.kr/ttb/api"
        
    def search_books(self, query: str, query_type: str = "Title", 
                    search_target: str = "Book", max_results: int = 10, 
                    start: int = 1, sort: str = "Accuracy", 
                    cover: str = "Big", output: str = "JS") -> Optional[Dict]:
        """
        도서 검색 API
        
        Args:
            query (str): 검색어
            query_type (str): 검색 타입 (Keyword, Title, Author, Publisher)
            search_target (str): 검색 대상 (Book, Foreign, Music, DVD, Used, eBook, All)
            max_results (int): 페이지당 최대 결과 수 (1-50)
            start (int): 시작 페이지
            sort (str): 정렬 방식 (Accuracy, PublishTime, Title, SalesPoint, CustomerRating)
            cover (str): 표지 크기 (Big, MidBig, Mid, Small, Mini, None)
            output (str): 출력 형식 (XML, JS)
        
        Returns:
            dict: API 응답 결과 또는 None (오류 시)
        """
        url = f"{self.base_url}/ItemSearch.aspx"
        
        params = {
            'ttbkey': self.ttb_key,
            'Query': query,
            'QueryType': query_type,
            'SearchTarget': search_target,
            'MaxResults': min(max_results, 50),  # 최대 50개로 제한
            'start': start,
            'Sort': sort,
            'Cover': cover,
            'output': output,
            'Version': '20131101'
        }
        
        try:
            response = requests.get(url, params=params, timeout=10)
            response.raise_for_status()
            
            if output == "JS":
                return response.json()
            else:
                return response.text
                
        except requests.exceptions.RequestException as e:
            print(f"❌ API 요청 중 오류 발생: {e}")
            return None
    
    def lookup_book(self, item_id: str, item_id_type: str = "ISBN13", 
                   cover: str = "Big", opt_result: Optional[List[str]] = None, 
                   output: str = "JS") -> Optional[Dict]:
        """
        도서 상세 조회 API
        
        Args:
            item_id (str): 상품 ID (ISBN 또는 알라딘 상품 ID)
            item_id_type (str): ID 타입 (ISBN, ISBN13, ItemId)
            cover (str): 표지 크기
            opt_result (list): 부가 정보 요청 (ebookList, usedList, reviewList 등)
            output (str): 출력 형식
        
        Returns:
            dict: API 응답 결과 또는 None (오류 시)
        """
        url = f"{self.base_url}/ItemLookUp.aspx"
        
        params = {
            'ttbkey': self.ttb_key,
            'ItemId': item_id,
            'ItemIdType': item_id_type,
            'Cover': cover,
            'output': output,
            'Version': '20131101'
        }
        
        if opt_result:
            params['OptResult'] = ','.join(opt_result)
        
        try:
            response = requests.get(url, params=params, timeout=10)
            response.raise_for_status()
            
            if output == "JS":
                return response.json()
            else:
                return response.text
                
        except requests.exceptions.RequestException as e:
            print(f"❌ API 요청 중 오류 발생: {e}")
            return None
    
    def search_specific_book(self, book_title: str) -> Optional[Dict]:
        """
        특정 도서 검색 (1단계용)
        
        Args:
            book_title (str): 검색할 도서 제목
            
        Returns:
            dict: 도서 정보 또는 None
        """
        print(f"🔍 알라딘 API에서 도서 검색: {book_title}")
        
        # 제목으로 정확한 검색
        result = self.search_books(
            query=book_title,
            query_type="Title",
            search_target="Book",
            max_results=5,
            cover="Big",
            output="JS"
        )
        
        if not result or 'item' not in result:
            print(f"❌ '{book_title}' 검색 결과가 없습니다.")
            return None
        
        # 첫 번째 결과 반환 (가장 정확한 매치)
        book_info = result['item'][0] if result['item'] else None
        
        if book_info:
            print(f"✅ 도서 발견: {book_info.get('title', 'N/A')}")
            print(f"   저자: {book_info.get('author', 'N/A')}")
            print(f"   출판사: {book_info.get('publisher', 'N/A')}")
            print(f"   ISBN13: {book_info.get('isbn13', 'N/A')}")
            print(f"   가격: {book_info.get('priceSales', 'N/A')}원")
        
        return book_info
    
    def get_book_details(self, isbn: str) -> Optional[Dict]:
        """
        ISBN으로 도서 상세 정보 조회
        
        Args:
            isbn (str): 도서 ISBN
            
        Returns:
            dict: 상세 도서 정보 또는 None
        """
        print(f"📖 ISBN으로 도서 상세 정보 조회: {isbn}")
        
        result = self.lookup_book(
            item_id=isbn,
            item_id_type="ISBN13",
            cover="Big",
            opt_result=["ebookList", "usedList"],
            output="JS"
        )
        
        if not result or 'item' not in result:
            print(f"❌ ISBN '{isbn}' 조회 결과가 없습니다.")
            return None
        
        book_info = result['item'][0] if result['item'] else None
        
        if book_info:
            print(f"✅ 상세 정보 조회 완료: {book_info.get('title', 'N/A')}")
        
        return book_info
    
    def test_connection(self) -> bool:
        """
        API 연결 테스트
        
        Returns:
            bool: 연결 성공 여부
        """
        try:
            # 간단한 검색으로 연결 테스트
            result = self.search_books("파이썬", max_results=1)
            if result and 'item' in result:
                print("✅ 알라딘 API 연결 성공")
                return True
            else:
                print("❌ 알라딘 API 연결 실패")
                return False
        except Exception as e:
            print(f"❌ API 연결 테스트 실패: {e}")
            return False

# 사용 예시
def main():
    """테스트용 메인 함수"""
    # API 키 설정 (실제 사용시 환경변수나 설정파일에서 로드)
    TTB_KEY = "ttbleeshun08062356001"
    
    # API 클라이언트 생성
    aladin = AladinAPIClient(TTB_KEY)
    
    # 연결 테스트
    if not aladin.test_connection():
        print("❌ API 연결 실패. TTBKey를 확인해주세요.")
        return
    
    # 특정 도서 검색 테스트
    book_info = aladin.search_specific_book("밑바닥부터 시작하는 딥러닝 1")
    
    if book_info:
        print("\n📚 검색된 도서 정보:")
        print(f"제목: {book_info.get('title', 'N/A')}")
        print(f"저자: {book_info.get('author', 'N/A')}")
        print(f"출판사: {book_info.get('publisher', 'N/A')}")
        print(f"ISBN13: {book_info.get('isbn13', 'N/A')}")
        print(f"가격: {book_info.get('priceSales', 'N/A')}원")
        print(f"표지 URL: {book_info.get('cover', 'N/A')}")
        print(f"설명: {book_info.get('description', 'N/A')[:100]}...")

if __name__ == "__main__":
    main() 