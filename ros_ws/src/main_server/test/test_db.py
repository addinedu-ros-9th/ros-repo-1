#!/usr/bin/env python3

import sys
import os

# 상위 디렉토리의 main_server 모듈을 찾을 수 있도록 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# 필요한 라이브러리 설치 확인
try:
    import pymysql
    print("✅ pymysql 라이브러리 설치됨")
except ImportError:
    print("❌ pymysql 라이브러리가 설치되지 않음")
    print("설치 명령어: pip install pymysql")
    exit(1)

# DB 매니저 테스트
from main_server.database.db_manager import DatabaseManager

def create_log_tables(db):
    """로그 테이블 생성"""
    print("\n 로그 테이블 생성 시작...")
    
    if not db.connection:
        print("❌ 데이터베이스 연결이 없습니다.")
        return False
    
    try:
        with db.connection.cursor() as cursor:
            # 로봇 상태 로그 테이블 생성
            print(" robot_status_log 테이블 생성 중...")
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS robot_status_log (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    robot_id VARCHAR(20) NOT NULL,
                    robot_state VARCHAR(20) NOT NULL,
                    is_available BOOLEAN NOT NULL,
                    battery INT NOT NULL,
                    book_weight DECIMAL(5,2) DEFAULT 0.0,
                    position_x DECIMAL(10,3) DEFAULT 0.0,
                    position_y DECIMAL(10,3) DEFAULT 0.0,
                    position_yaw DECIMAL(8,3) DEFAULT 0.0,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    INDEX idx_robot_id (robot_id),
                    INDEX idx_timestamp (timestamp),
                    INDEX idx_robot_state (robot_state)
                )
            """)
            print("✅ robot_status_log 테이블 생성 완료")
            
            # 태스크 로그 테이블 생성
            print("📋 task_log 테이블 생성 중...")
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS task_log (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    task_id VARCHAR(50) NOT NULL,
                    robot_id VARCHAR(20) NOT NULL,
                    task_type VARCHAR(20) NOT NULL,
                    task_stage INT NOT NULL,
                    call_location VARCHAR(20) NOT NULL,
                    goal_location VARCHAR(20) NOT NULL,
                    start_time TIMESTAMP NULL,
                    end_time TIMESTAMP NULL,
                    log_type ENUM('START', 'COMPLETE') NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    INDEX idx_task_id (task_id),
                    INDEX idx_robot_id (robot_id),
                    INDEX idx_timestamp (timestamp),
                    INDEX idx_task_type (task_type),
                    INDEX idx_log_type (log_type)
                )
            """)
            print("✅ task_log 테이블 생성 완료")
            
        print("✅ 로그 테이블 생성 완료")
        
        # 생성된 테이블 확인
        print("\n📊 생성된 테이블 확인...")
        with db.connection.cursor() as cursor:
            cursor.execute("SHOW TABLES LIKE '%log%'")
            tables = cursor.fetchall()
            for table in tables:
                if table:  # 테이블이 존재하면
                    print(f"  - {table[0]}")
        
        return True
        
    except Exception as e:
        print(f"❌ 로그 테이블 생성 실패: {e}")
        return False

def test_database():
    print("🔌 데이터베이스 연결 테스트 시작...")
    
    # DB 매니저 생성
    db = DatabaseManager()
    
    # 연결 테스트
    if db.test_connection():
        print("✅ 데이터베이스 연결 및 테이블 확인 완료")
        
        # 로그 테이블 생성
        create_log_tables(db)
        
        # 검색 테스트
        print("\n🔍 도서 검색 테스트...")
        results = db.search_books("파이썬", "title")
        
        if results:
            print(f"📚 검색 결과 ({len(results)}권):")
            for book in results:
                print(f"  - {book['title']} ({book['author']}) - {book['location']}구역")
        else:
            print("📚 검색 결과가 없습니다.")
            
        # 다른 검색어도 테스트
        print("\n🔍 '소설' 검색 테스트...")
        results2 = db.search_books("소설", "category_name")
        print(f"📚 소설 카테고리: {len(results2)}권")
    
    # 연결 종료
    db.close()

if __name__ == "__main__":
    test_database()