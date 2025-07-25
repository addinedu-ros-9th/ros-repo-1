#!/usr/bin/env python3

import sys
sys.path.append('.')
from main_server.database.db_manager import DatabaseManager

def test_db_connection():
    print("🔌 데이터베이스 연결 테스트...")
    
    try:
        db = DatabaseManager()
        
        # 검색 테스트
        result = db.search_books('파이썬', 'title')
        print(f"🔍 검색 결과: {len(result)}권")
        
        for book in result:
            print(f"  - {book['title']} ({book['author']}) - {book['location']}구역")
            
        print("✅ 데이터베이스 연결 및 검색 성공!")
        
    except Exception as e:
        print(f"❌ 오류: {e}")

if __name__ == "__main__":
    test_db_connection()
