#!/usr/bin/env python3

import sys
import os

# ROS2 워크스페이스 환경 설정
ros_ws_path = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.join(ros_ws_path, 'install', 'lib', 'python3.10', 'site-packages'))

from PyQt5.QtWidgets import QApplication
from kiosk.ui.book_detail_popup import BookDetailPopup

def main():
    app = QApplication(sys.argv)
    
    # 테스트용 책 정보
    test_book = {
        'title': '밑바닥부터 시작하는 딥러닝 1 (리마스터판)',
        'author': '사이토 고키',
        'publisher': '한빛미디어',
        'category_name': '컴퓨터',
        'price': 32000,
        'stock_quantity': 2,
        'isbn': '9788968481475',
        'location': 'D',
        'cover_image_url': 'https://image.aladin.co.kr/product/2635/61/cover/8968481474_1.jpg'
    }
    
    print("📖 책 상세 정보 팝업창 테스트 시작")
    print(f"도서: {test_book['title']}")
    print(f"위치: {test_book['location']}구역")
    
    # 팝업창 생성 및 표시
    popup = BookDetailPopup(test_book)
    
    def on_escort_requested(escort_data):
        print(f"🚀 에스코팅 요청: {escort_data}")
        popup.close()
    
    popup.escort_requested.connect(on_escort_requested)
    popup.show()
    
    print("✅ 팝업창 표시 완료")
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 