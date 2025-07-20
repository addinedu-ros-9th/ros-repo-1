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

def test_database():
    print("🔌 데이터베이스 연결 테스트 시작...")
    
    # DB 매니저 생성
    db = DatabaseManager()
    
    # 연결 테스트
    if db.test_connection():
        print("✅ 데이터베이스 연결 및 테이블 확인 완료")
        
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