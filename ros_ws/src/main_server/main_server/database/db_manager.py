# DB 매니저 확장 (db_manager.py)
# 도서 등록 메서드 추가 (register_book)
# ISBN 중복 검사
# ISBN으로 도서 조회 기능

import pymysql
from typing import List, Dict, Optional

class DatabaseManager:
    def __init__(self):
        """데이터베이스 매니저 초기화"""
        self.connection = None
        self._connect()
    
    def _connect(self):
        """GCP MySQL 데이터베이스 연결"""
        try:
            self.connection = pymysql.connect(
                host='34.47.96.177',           # GCP DB 공개 IP 주소
                port=3306,
                user='root',                   # 또는 생성한 사용자명
                password='qwer1234!@#$',       # DB 비밀번호
                database='libo_db',
                charset='utf8mb4',
                autocommit=True
            )
            print("✅ 데이터베이스 연결 성공")
        except Exception as e:
            print(f"❌ 데이터베이스 연결 실패: {e}")
            self.connection = None
    
    def search_books(self, query: str, search_type: str = 'title') -> List[Dict]:
        """
        도서 검색
        
        Args:
            query: 검색어
            search_type: 검색 타입 ('title', 'author', 'publisher')
            
        Returns:
            검색된 도서 정보 리스트
        """
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return []
        
        try:
            # 검색 타입에 따른 쿼리 작성
            if search_type == 'title':
                where_clause = "title LIKE %s"
            elif search_type == 'author':
                where_clause = "author LIKE %s"
            elif search_type == 'publisher':
                where_clause = "publisher LIKE %s"
            elif search_type == 'category_name':
                where_clause = "category_name LIKE %s"
            else:
                where_clause = "title LIKE %s"
            
            
            query_sql = f"""
            SELECT 
                id, title, author, publisher, category_name, location,
                price, stock_quantity, isbn, cover_image_url
            FROM books
            WHERE {where_clause}
            ORDER BY title ASC
            """
            
            search_param = f"%{query}%"
            
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                cursor.execute(query_sql, (search_param,))
                results = cursor.fetchall()
                
            print(f"🔍 검색 결과: {len(results)}권 발견")
            return results
            
        except Exception as e:
            print(f"❌ 검색 오류: {e}")
            return []
    
    def test_connection(self) -> bool:
        """데이터베이스 연결 테스트"""
        try:
            with self.connection.cursor() as cursor:
                cursor.execute("SELECT COUNT(*) as total FROM books")
                result = cursor.fetchone()
                print(f"📚 총 도서 수: {result[0]}권")
                return True
        except Exception as e:
            print(f"❌ 연결 테스트 실패: {e}")
            return False
    
    def register_book(self, book_data: Dict) -> bool:
        """
        도서 등록 (1단계용) - 중복 시 재고 증가
        
        Args:
            book_data: 등록할 도서 정보 딕셔너리
            
        Returns:
            bool: 등록 성공 여부
        """
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return False
        
        try:
            # 중복 검사 (ISBN 기준)
            isbn = book_data.get('isbn', '')
            if isbn:
                with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                    cursor.execute("SELECT id, stock_quantity FROM books WHERE isbn = %s", (isbn,))
                    existing_book = cursor.fetchone()
                    
                    if existing_book:
                        # 기존 도서가 있으면 재고 증가
                        current_stock = existing_book['stock_quantity']
                        new_stock = current_stock + book_data.get('stock_quantity', 1)
                        
                        update_sql = "UPDATE books SET stock_quantity = %s WHERE isbn = %s"
                        cursor.execute(update_sql, (new_stock, isbn))
                        
                        print(f"�� 기존 도서 재고 증가: {book_data.get('title', 'N/A')}")
                        print(f"   기존 재고: {current_stock}권 → 새로운 재고: {new_stock}권")
                        return True
            
            # 새로운 도서 등록
            insert_sql = """
            INSERT INTO books (
                title, author, publisher, category_name, location,
                price, stock_quantity, isbn, cover_image_url
            ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
            """
            
            values = (
                book_data.get('title', ''),
                book_data.get('author', ''),
                book_data.get('publisher', ''),
                book_data.get('category_name', ''),
                book_data.get('location', ''),
                book_data.get('price', 0),
                book_data.get('stock_quantity', 1),
                book_data.get('isbn', ''),
                book_data.get('cover_image_url', '')
            )
            
            with self.connection.cursor() as cursor:
                cursor.execute(insert_sql, values)
                
            print(f"✅ 새 도서 등록 성공: {book_data.get('title', 'N/A')}")
            return True
            
        except Exception as e:
            print(f"❌ 도서 등록 실패: {e}")
            return False
    
    def get_book_by_isbn(self, isbn: str) -> Optional[Dict]:
        """
        ISBN으로 도서 조회
        
        Args:
            isbn: 조회할 도서의 ISBN
            
        Returns:
            도서 정보 딕셔너리 또는 None
        """
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return None
        
        try:
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                cursor.execute("""
                    SELECT id, title, author, publisher, category_name, location,
                           price, stock_quantity, isbn, cover_image_url
                    FROM books WHERE isbn = %s
                """, (isbn,))
                result = cursor.fetchone()
                
            return result
            
        except Exception as e:
            print(f"❌ 도서 조회 실패: {e}")
            return None
    
    def close(self):
        """데이터베이스 연결 종료"""
        if self.connection:
            self.connection.close()
            print("📴 데이터베이스 연결 종료")