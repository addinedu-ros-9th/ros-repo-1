# DB 매니저 확장 (db_manager.py)
# 도서 등록 메서드 추가 (register_book)
# ISBN 중복 검사
# ISBN으로 도서 조회 기능
# ✅ DB 테이블 스키마와 완벽 일치하도록 수정

import pymysql
from typing import List, Dict, Optional  # 단순화를 위해 Set 제거
from datetime import datetime  # 날짜/시간 변환용

class DatabaseManager:
    def __init__(self):
        """데이터베이스 매니저 초기화"""
        self.connection = None  # DB 커넥션 핸들  # 초기값 None
        self._connect()  # DB 연결 시도  # 즉시 연결
        # 단순화: 컬럼 캐시/동적 매핑은 사용하지 않음  # 유지보수 용이성 위해 제거
    
    def _connect(self):  # DB 연결하는 내부 함수
        """GCP MySQL 데이터베이스 연결"""
        try:
            self.connection = pymysql.connect(
                host='34.47.118.222',          # GCP DB 공개 IP  # 환경 고정값
                port=3306,                     # MySQL 포트  # 기본 3306
                user='root',                   # 사용자명  # root로 접속
                password='qwer1234!@#$',       # 비밀번호  # 보안상 환경변수로 옮기면 더 좋음
                database='libo_db',            # DB 스키마명  # libo_db 사용
                charset='utf8mb4',             # 문자셋  # 이모지 포함
                autocommit=True                # 자동 커밋  # INSERT마다 즉시 반영
            )
            print("✅ 데이터베이스 연결 성공")  # 연결 성공 로그
        except Exception as e:
            print(f"❌ 데이터베이스 연결 실패: {e}")  # 실패 사유 출력
            self.connection = None  # 연결 실패 시 None 유지
    
    def _get_table_columns(self, table_name: str):  # 사용 안 함
        """단순화를 위해 컬럼 탐색 로직 비활성화 (항상 고정 스키마에 저장)."""
        return set()  # 더 이상 사용하지 않음
    
    def search_books(self, query: str, search_type: str = 'title') -> List[Dict]:  # 도서 검색하는 함수
        """
        도서 검색
        
        Args:
            query: 검색어
            search_type: 검색 타입 ('title', 'author', 'publisher')
            
        Returns:
            검색된 도서 정보 리스트
        """
        # 연결 재시도 로직
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다. 재연결 시도...")
            self._connect()
            if not self.connection:
                print("❌ 데이터베이스 재연결 실패")
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
            
            # ✅ 수정: location → location_id (DB 스키마 일치)
            query_sql = f"""
            SELECT 
                id, title, author, publisher, category_name, location_id,
                price, stock_quantity, isbn, cover_image_url
            FROM book
            WHERE {where_clause}
            ORDER BY title ASC
            """
            
            search_param = f"%{query}%"
            
            # 연결 상태 확인 및 재연결
            try:
                self.connection.ping(reconnect=True)
            except Exception as ping_error:
                print(f"⚠️ 데이터베이스 연결 확인 중 오류: {ping_error}")
                self._connect()
                if not self.connection:
                    print("❌ 데이터베이스 재연결 실패")
                    return []
            
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                cursor.execute(query_sql, (search_param,))
                results = cursor.fetchall()
                
                # ✅ 호환성 유지: location_id를 location으로도 제공
                for result in results:
                    if 'location_id' in result:
                        result['location'] = result['location_id']
                
            print(f"🔍 검색 결과: {len(results)}권 발견")
            return results
            
        except Exception as e:
            print(f"❌ 검색 오류: {e}")
            # 연결 오류인 경우 재연결 시도
            if "MySQL server has gone away" in str(e) or "Lost connection" in str(e):
                print("🔄 데이터베이스 연결 오류로 재연결 시도...")
                self._connect()
            return []
    
    def test_connection(self) -> bool:  # DB 연결이 잘 되는지 테스트하는 함수
        """데이터베이스 연결 테스트"""
        try:
            with self.connection.cursor() as cursor:
                cursor.execute("SELECT COUNT(*) as total FROM book")
                result = cursor.fetchone()
                print(f"📚 총 도서 수: {result[0]}권")
                return True
        except Exception as e:
            print(f"❌ 연결 테스트 실패: {e}")
            return False
    
    def register_book(self, book_data: Dict) -> bool:  # 새 도서 등록하거나 기존 도서 재고 늘리는 함수
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
                    cursor.execute("SELECT id, stock_quantity FROM book WHERE isbn = %s", (isbn,))
                    existing_book = cursor.fetchone()
                    
                    if existing_book:
                        # 기존 도서가 있으면 재고 증가
                        current_stock = existing_book['stock_quantity']
                        new_stock = current_stock + book_data.get('stock_quantity', 1)
                        
                        update_sql = "UPDATE book SET stock_quantity = %s WHERE isbn = %s"
                        cursor.execute(update_sql, (new_stock, isbn))
                        
                        print(f"✅ 기존 도서 재고 증가: {book_data.get('title', 'N/A')}")
                        print(f"   기존 재고: {current_stock}권 → 새로운 재고: {new_stock}권")
                        return True
            
            # ✅ 수정: location → location_id (DB 스키마 일치)
            insert_sql = """
            INSERT INTO book (
                title, author, publisher, category_name, location_id,
                price, stock_quantity, isbn, cover_image_url
            ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
            """
            
            values = (
                book_data.get('title', ''),
                book_data.get('author', ''),
                book_data.get('publisher', ''),
                book_data.get('category_name', ''),
                book_data.get('location', ''),  # 입력 데이터에서는 'location' 키 사용
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
    
    def verify_admin_qr(self, admin_name: str) -> bool:  # 관리자 QR 인증 함수
        """
        관리자 QR 인증
        
        Args:
            admin_name: 관리자 이름
            
        Returns:
            인증 성공 여부
        """
        # 연결 재시도 로직
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다. 재연결 시도...")
            self._connect()
            if not self.connection:
                print("❌ 데이터베이스 재연결 실패")
                return False
        
        try:
            # admin 테이블에서 해당 이름의 관리자 조회
            query_sql = """
            SELECT id, name, type
            FROM admin
            WHERE name = %s AND type = 'qr'
            """
            
            # 연결 상태 확인 및 재연결
            try:
                self.connection.ping(reconnect=True)
            except Exception as ping_error:
                print(f"⚠️ 데이터베이스 연결 확인 중 오류: {ping_error}")
                self._connect()
                if not self.connection:
                    print("❌ 데이터베이스 재연결 실패")
                    return False
            
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                cursor.execute(query_sql, (admin_name,))
                result = cursor.fetchone()
                
                if result:
                    print(f"✅ QR 인증 성공: {admin_name} (ID: {result['id']})")
                    return True
                else:
                    print(f"❌ QR 인증 실패: {admin_name} - DB에 등록되지 않은 관리자")
                    return False
                    
        except Exception as e:
            print(f"❌ QR 인증 오류: {e}")
            return False

    def get_book_by_isbn(self, isbn: str) -> Optional[Dict]:  # ISBN 번호로 도서 찾는 함수
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
                # ✅ 수정: location → location_id (DB 스키마 일치)
                cursor.execute("""
                    SELECT id, title, author, publisher, category_name, location_id,
                           price, stock_quantity, isbn, cover_image_url
                    FROM book WHERE isbn = %s
                """, (isbn,))
                result = cursor.fetchone()
                
                # ✅ 호환성 유지: location_id를 location으로도 제공
                if result and 'location_id' in result:
                    result['location'] = result['location_id']
                
            return result
            
        except Exception as e:
            print(f"❌ 도서 조회 실패: {e}")
            return None
    # DatabaseManager 클래스에 추가할 재고 감소 함수들

    def decrease_book_stock(self, isbn: str, quantity: int) -> bool:
        """
        도서 재고 감소
        
        Args:
            isbn: 도서 ISBN
            quantity: 감소할 수량
            
        Returns:
            재고 감소 성공 여부
        """
        # 연결 재시도 로직
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다. 재연결 시도...")
            self._connect()
            if not self.connection:
                print("❌ 데이터베이스 재연결 실패")
                return False
        
        try:
            # 연결 상태 확인 및 재연결
            try:
                self.connection.ping(reconnect=True)
            except Exception as ping_error:
                print(f"⚠️ 데이터베이스 연결 확인 중 오류: {ping_error}")
                self._connect()
                if not self.connection:
                    print("❌ 데이터베이스 재연결 실패")
                    return False
            
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                # 현재 재고 확인
                cursor.execute("SELECT id, stock_quantity FROM book WHERE isbn = %s", (isbn,))
                book = cursor.fetchone()
                
                if not book:
                    print(f"❌ 도서를 찾을 수 없습니다: ISBN {isbn}")
                    return False
                
                current_stock = book['stock_quantity']
                
                # 재고 부족 확인
                if current_stock < quantity:
                    print(f"❌ 재고 부족: 현재 {current_stock}권, 요청 {quantity}권")
                    return False
                
                # 재고 감소
                new_stock = current_stock - quantity
                update_sql = "UPDATE book SET stock_quantity = %s WHERE isbn = %s"
                cursor.execute(update_sql, (new_stock, isbn))
                
                print(f"✅ 재고 감소 성공: ISBN {isbn}")
                print(f"   기존 재고: {current_stock}권 → 새로운 재고: {new_stock}권")
                
                return True
                
        except Exception as e:
            print(f"❌ 재고 감소 실패: {e}")
            # 연결 오류인 경우 재연결 시도
            if "MySQL server has gone away" in str(e) or "Lost connection" in str(e):
                print("🔄 데이터베이스 연결 오류로 재연결 시도...")
                self._connect()
            return False

    def get_book_stock(self, isbn: str) -> int:
        """
        도서 재고 수량 조회
        
        Args:
            isbn: 도서 ISBN
            
        Returns:
            현재 재고 수량 (실패 시 -1)
        """
        # 연결 재시도 로직
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다. 재연결 시도...")
            self._connect()
            if not self.connection:
                print("❌ 데이터베이스 재연결 실패")
                return -1
        
        try:
            # 연결 상태 확인 및 재연결
            try:
                self.connection.ping(reconnect=True)
            except Exception as ping_error:
                print(f"⚠️ 데이터베이스 연결 확인 중 오류: {ping_error}")
                self._connect()
                if not self.connection:
                    print("❌ 데이터베이스 재연결 실패")
                    return -1
            
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                cursor.execute("SELECT stock_quantity FROM book WHERE isbn = %s", (isbn,))
                result = cursor.fetchone()
                
                if result:
                    return result['stock_quantity']
                else:
                    print(f"❌ 도서를 찾을 수 없습니다: ISBN {isbn}")
                    return -1
                    
        except Exception as e:
            print(f"❌ 재고 조회 실패: {e}")
            # 연결 오류인 경우 재연결 시도
            if "MySQL server has gone away" in str(e) or "Lost connection" in str(e):
                print("🔄 데이터베이스 연결 오류로 재연결 시도...")
                self._connect()
            return -1

    def increase_book_stock(self, isbn: str, quantity: int) -> bool:
        """
        도서 재고 증가 (입고용)
        
        Args:
            isbn: 도서 ISBN
            quantity: 증가할 수량
            
        Returns:
            재고 증가 성공 여부
        """
        # 연결 재시도 로직
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다. 재연결 시도...")
            self._connect()
            if not self.connection:
                print("❌ 데이터베이스 재연결 실패")
                return False
        
        try:
            # 연결 상태 확인 및 재연결
            try:
                self.connection.ping(reconnect=True)
            except Exception as ping_error:
                print(f"⚠️ 데이터베이스 연결 확인 중 오류: {ping_error}")
                self._connect()
                if not self.connection:
                    print("❌ 데이터베이스 재연결 실패")
                    return False
            
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                # 현재 재고 확인
                cursor.execute("SELECT id, stock_quantity FROM book WHERE isbn = %s", (isbn,))
                book = cursor.fetchone()
                
                if not book:
                    print(f"❌ 도서를 찾을 수 없습니다: ISBN {isbn}")
                    return False
                
                current_stock = book['stock_quantity']
                
                # 재고 증가
                new_stock = current_stock + quantity
                update_sql = "UPDATE book SET stock_quantity = %s WHERE isbn = %s"
                cursor.execute(update_sql, (new_stock, isbn))
                
                print(f"✅ 재고 증가 성공: ISBN {isbn}")
                print(f"   기존 재고: {current_stock}권 → 새로운 재고: {new_stock}권")
                
                return True
                
        except Exception as e:
            print(f"❌ 재고 증가 실패: {e}")
            # 연결 오류인 경우 재연결 시도
            if "MySQL server has gone away" in str(e) or "Lost connection" in str(e):
                print("🔄 데이터베이스 연결 오료로 재연결 시도...")
                self._connect()
            return False


    def log_robot_status(self, robot_data: Dict) -> bool:
        """로봇 상태 로그 저장 (10초마다)"""
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return False
        
        try:
            with self.connection.cursor() as cursor:
                cursor.execute("""
                    INSERT INTO robot_status_log 
                    (robot_id, robot_state, is_available, battery, book_weight, 
                        position_x, position_y, position_yaw)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                """, (
                    robot_data.get('robot_id'),
                    robot_data.get('robot_state'),
                    robot_data.get('is_available'),
                    robot_data.get('battery'),
                    robot_data.get('book_weight', 0.0),
                    robot_data.get('position_x', 0.0),
                    robot_data.get('position_y', 0.0),
                    robot_data.get('position_yaw', 0.0)
                ))
            
            return True
            
        except Exception as e:
            print(f"❌ 로봇 상태 로그 저장 실패: {e}")
            return False

    def log_task_start(self, task_data: Dict) -> bool:
        """Task 시작 로그 저장"""
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return False
        
        try:
            with self.connection.cursor() as cursor:
                cursor.execute("""
                    INSERT INTO task_log 
                    (task_id, robot_id, task_type, task_stage, call_location, 
                        goal_location, start_time, log_type)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, 'START')
                """, (
                    task_data.get('task_id'),
                    task_data.get('robot_id'),
                    task_data.get('task_type'),
                    task_data.get('task_stage'),
                    task_data.get('call_location'),
                    task_data.get('goal_location'),
                    task_data.get('start_time')
                ))
            
            return True
            
        except Exception as e:
            print(f"❌ Task 시작 로그 저장 실패: {e}")
            return False

    def log_task_complete(self, task_data: Dict) -> bool:
        """Task 완료 로그 저장"""
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return False
        
        try:
            with self.connection.cursor() as cursor:
                cursor.execute("""
                    INSERT INTO task_log 
                    (task_id, robot_id, task_type, task_stage, call_location, 
                        goal_location, end_time, log_type)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, 'COMPLETE')
                """, (
                    task_data.get('task_id'),
                    task_data.get('robot_id'),
                    task_data.get('task_type'),
                    task_data.get('task_stage'),
                    task_data.get('call_location'),
                    task_data.get('goal_location'),
                    task_data.get('end_time')
                ))
            
            return True
            
        except Exception as e:
            print(f"❌ Task 완료 로그 저장 실패: {e}")
            return False

    def get_robot_status_history(self, robot_id: str = None, hours: int = 24) -> List[Dict]:
        """로봇 상태 히스토리 조회"""
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return []
        
        try:
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                if robot_id:
                    cursor.execute("""
                        SELECT * FROM robot_status_log 
                        WHERE robot_id = %s AND timestamp >= NOW() - INTERVAL %s HOUR
                        ORDER BY timestamp DESC
                    """, (robot_id, hours))
                else:
                    cursor.execute("""
                        SELECT * FROM robot_status_log 
                        WHERE timestamp >= NOW() - INTERVAL %s HOUR
                        ORDER BY timestamp DESC
                    """, (hours,))
                
                results = cursor.fetchall()
                return results
                
        except Exception as e:
            print(f"❌ 로봇 상태 히스토리 조회 실패: {e}")
            return []

    def get_task_history(self, robot_id: str = None, hours: int = 24) -> List[Dict]:
        """Task 히스토리 조회"""
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return []
        
        try:
            with self.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                if robot_id:
                    cursor.execute("""
                        SELECT * FROM task_log 
                        WHERE robot_id = %s AND timestamp >= NOW() - INTERVAL %s HOUR
                        ORDER BY timestamp DESC
                    """, (robot_id, hours))
                else:
                    cursor.execute("""
                        SELECT * FROM task_log 
                        WHERE timestamp >= NOW() - INTERVAL %s HOUR
                        ORDER BY timestamp DESC
                    """, (hours,))
                
                results = cursor.fetchall()
                return results
                
        except Exception as e:
            print(f"❌ Task 히스토리 조회 실패: {e}")
            return []

    def save_overall_status(self, data: Dict) -> bool:
        """로봇 상태 스냅샷을 robot_status_log 테이블에 저장한다 (고정 스키마)."""
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다. 재연결 시도...")  # 안내 로그
            self._connect()  # 재연결 시도
            if not self.connection:
                print("❌ 데이터베이스 재연결 실패")  # 실패 안내
                return False  # 저장 불가
        try:
            try:
                self.connection.ping(reconnect=True)  # 연결 확인/복구  # 끊겼으면 재연결
            except Exception as ping_error:
                print(f"⚠️ 데이터베이스 연결 확인 중 오류: {ping_error}")  # 경고 로그
                self._connect()  # 재연결 재시도
                if not self.connection:
                    print("❌ 데이터베이스 재연결 실패")  # 실패 안내
                    return False  # 저장 불가

            # 고정 테이블 스키마에 맞춰 INSERT  # test_db.py의 robot_status_log 기준
            insert_sql = (
                """
                INSERT INTO robot_status_log (
                    robot_id, robot_state, is_available, battery,
                    book_weight, position_x, position_y, position_yaw
                ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                """
            )
            values = (
                data.get('robot_id'),                             # 로봇 ID
                data.get('robot_state'),                          # 로봇 상태 문자열
                1 if data.get('is_available') else 0,             # 사용 가능 여부(BOOL→INT)
                int(data.get('battery', 0)),                      # 배터리 (정수)
                float(data.get('book_weight', 0.0)),              # 책 무게(kg)
                float(data.get('position_x', 0.0)),               # 위치 X
                float(data.get('position_y', 0.0)),               # 위치 Y
                float(data.get('position_yaw', 0.0)),             # 요(각도)
            )
            with self.connection.cursor() as cursor:
                cursor.execute(insert_sql, values)  # INSERT 실행
            return True  # 성공
        except Exception as e:
            print(f"❌ robot_status_log 저장 실패: {e}")  # 실패 로그
            return False  # 실패 반환

    def save_task_status_event(self, data: Dict) -> bool:
        """Task 시작/완료 이벤트를 task_log 테이블에 저장한다 (고정 스키마)."""
        if not self.connection:
            print("❌ 데이터베이스 연결이 없습니다. 재연결 시도...")  # 안내 로그
            self._connect()  # 재연결 시도
            if not self.connection:
                print("❌ 데이터베이스 재연결 실패")  # 실패 안내
                return False  # 저장 불가
        try:
            try:
                self.connection.ping(reconnect=True)  # 연결 확인/복구
            except Exception as ping_error:
                print(f"⚠️ 데이터베이스 연결 확인 중 오류: {ping_error}")  # 경고 로그
                self._connect()  # 재연결 재시도
                if not self.connection:
                    print("❌ 데이터베이스 재연결 실패")  # 실패 안내
                    return False  # 저장 불가

            # 시간 변환 (TIMESTAMP 컬럼에 맞춤)  # NULL 허용
            start_dt = None  # 기본값 None
            end_dt = None  # 기본값 None
            try:
                if data.get('start_time') is not None:
                    start_dt = datetime.fromtimestamp(float(data['start_time']))  # epoch→datetime
                if data.get('end_time') is not None:
                    end_dt = datetime.fromtimestamp(float(data['end_time']))  # epoch→datetime
            except Exception:
                pass  # 변환 실패 시 None 유지

            # 이벤트 매핑 (ENUM('START','COMPLETE') 호환)  # 읽기 쉬운 값으로 통일
            ev = (data.get('event') or '').strip().upper()  # 공백 제거+대문자화
            log_type = 'START' if ev == 'STARTED' else ('COMPLETE' if ev == 'COMPLETED' else ev)  # 매핑

            insert_sql = (
                """
                INSERT INTO task_log (
                    task_id, robot_id, task_type, task_stage, call_location, goal_location,
                    start_time, end_time, log_type
                ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
                """
            )
            values = (
                data.get('task_id'),                # 태스크 ID
                data.get('robot_id'),               # 로봇 ID
                data.get('task_type'),              # 태스크 타입
                int(data.get('task_stage', 0)),     # 스테이지(정수)
                data.get('call_location'),          # 호출 위치
                data.get('goal_location'),          # 목적지 위치
                start_dt,                            # 시작 시간(datetime)
                end_dt,                              # 종료 시간(datetime)
                log_type,                            # 로그 타입(START/COMPLETE)
            )
            with self.connection.cursor() as cursor:
                cursor.execute(insert_sql, values)  # INSERT 실행
            return True  # 성공
        except Exception as e:
            print(f"❌ task_log 저장 실패: {e}")  # 실패 로그
            return False  # 실패 반환

    def close(self):  # DB 연결 끊는 함수
        """데이터베이스 연결 종료"""
        if self.connection:
            self.connection.close()
            print("📴 데이터베이스 연결 종료")