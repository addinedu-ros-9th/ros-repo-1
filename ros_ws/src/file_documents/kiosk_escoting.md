지켜야 할 것
인터페이스 명세서 지키기
kiosk gui와 main_server는 ros2통신
코드 한번에 전부 다 하지 말고 질문을 하면 한단계씩만 설명하기
코드 수정은 내가 ok 하면 그때 수정
인터페이스 명세서가 뭔가 부족하다고 생각하면 추가 인터페이스 필요하다고 알려줘야함
그리고 이 작업에 대한 먼저 각 단계마다 데이터 흐름 과시퀀스 다이어그램 작성(plant_uml)해주기
일단은 gazebo시뮬레이션으로 연습할 예정
그리고 우리 환경은 우분투 24.04이고 ros2 jazzy를 사용하고 있고 gazebo harmonic을 사용하고 있다는 것을 명심해


## 해야할 것
## 1단계: 알라딘 api를 이용해서 원하는 책 db table에 저장
알라딘 api를 이용해서 원하는 책 db table에 저장할수 있도록 알라딘 api에서 책 정보를 가져와 해당 컬럼에 맞게 db table에 저장
```sql
mysql> select * from books;
+----+-------------------------------------+---------------------------+-----------------+---------------+----------+----------+----------------+---------------+--------------------------------------------------------------------+---------------------+
| id | title                               | author                    | publisher       | category_name | location | price    | stock_quantity | isbn          | cover_image_url                                                    | created_at          |
+----+-------------------------------------+---------------------------+-----------------+---------------+----------+----------+----------------+---------------+--------------------------------------------------------------------+---------------------+
|  1 | 클린 코드                           | 로버트 C. 마틴            | 인사이트        | 컴퓨터        | A        | 33000.00 |              2 | 9788966262281 | https://image.aladin.co.kr/product/2083/95/cover/8966262287_1.jpg  | 2025-07-20 15:29:56 |
|  2 | 파이썬 머신러닝                     | 권철민                    | 위키북스        | 컴퓨터        | A        | 35000.00 |              1 | 9791158391744 | https://image.aladin.co.kr/product/15887/64/cover/k682534545_1.jpg | 2025-07-20 15:29:56 |
|  3 | 자바스크립트 완벽 가이드            | 데이비드 플래너건         | 인사이트        | 컴퓨터        | A        | 45000.00 |              3 | 9788968481475 | https://image.aladin.co.kr/product/2635/61/cover/8968481474_1.jpg  | 2025-07-20 15:29:56 |
|  4 | 해커스 토익                         | 데이비드 조               | 해커스          | 언어          | B        | 18000.00 |              4 | 9788925567068 | https://image.aladin.co.kr/product/24441/93/cover/8925567067_1.jpg | 2025-07-20 15:29:56 |
|  5 | 영어회화 100일의 기적               | 문성현                    | 넥서스          | 언어          | B        | 15000.00 |              2 | 9788959135771 | https://image.aladin.co.kr/product/843/47/cover/8959135771_1.jpg   | 2025-07-20 15:29:56 |
|  6 | HSK 한 권으로 끝내기                | 시원스쿨                  | 시원스쿨        | 언어          | B        | 22000.00 |              1 | 9788934942467 | https://image.aladin.co.kr/product/18945/12/cover/8934942460_1.jpg | 2025-07-20 15:29:56 |
|  7 | 82년생 김지영                       | 조남주                    | 민음사          | 소설          | C        | 13800.00 |              5 | 9788954429788 | https://image.aladin.co.kr/product/8636/37/cover/8954429785_1.jpg  | 2025-07-20 15:29:56 |
|  8 | 미움받을 용기                       | 기시미 이치로             | 인플루엔셜      | 소설          | C        | 14900.00 |              3 | 9788936433598 | https://image.aladin.co.kr/product/5641/55/cover/8936433598_1.jpg  | 2025-07-20 15:29:56 |
|  9 | 사피엔스                            | 유발 하라리               | 김영사          | 소설          | C        | 19800.00 |              2 | 9788932917245 | https://image.aladin.co.kr/product/6635/94/cover/8932917248_1.jpg  | 2025-07-20 15:29:56 |
+----+-------------------------------------+---------------------------+-----------------+---------------+----------+----------+----------------+---------------+--------------------------------------------------------------------+---------------------+
9 rows in set (0.01 sec)


```
이 테이블에 있는책들은 예시임
일단은 밑바닥부터 시작하는 딥러닝 1(리마스터판) 이책 1권만 이 테이블 table에 등록해주셈 이 책 카테고리는 컴퓨터로 
참고로 책 카테고리는 컴퓨터, 언어, 소설 이렇게 3개만 사용할거임

예시 코드1
```python
import requests
import pandas as pd
import json
from urllib.parse import quote
import time

class AladinAPI:
    def __init__(self, ttb_key):
        """
        알라딘 API 클래스 초기화
        
        Args:
            ttb_key (str): 알라딘에서 발급받은 TTBKey
        """
        self.ttb_key = ttb_key
        self.base_url = "http://www.aladin.co.kr/ttb/api"
        
    def search_books(self, query, query_type="Keyword", search_target="Book", 
                    max_results=10, start=1, sort="Accuracy", cover="Mid", output="JS"):
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
            dict: API 응답 결과
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
            response = requests.get(url, params=params)
            response.raise_for_status()
            
            if output == "JS":
                return response.json()
            else:
                return response.text
                
        except requests.exceptions.RequestException as e:
            print(f"API 요청 중 오류 발생: {e}")
            return None
    
    def get_book_list(self, query_type="ItemNewAll", search_target="Book", 
                     max_results=10, start=1, cover="Mid", category_id=0, output="JS"):
        """
        도서 리스트 API (신간, 베스트셀러 등)
        
        Args:
            query_type (str): 리스트 종류 (ItemNewAll, ItemNewSpecial, ItemEditorChoice, Bestseller, BlogBest)
            search_target (str): 검색 대상
            max_results (int): 페이지당 최대 결과 수
            start (int): 시작 페이지
            cover (str): 표지 크기
            category_id (int): 카테고리 ID (0은 전체)
            output (str): 출력 형식
        
        Returns:
            dict: API 응답 결과
        """
        url = f"{self.base_url}/ItemList.aspx"
        
        params = {
            'ttbkey': self.ttb_key,
            'QueryType': query_type,
            'SearchTarget': search_target,
            'MaxResults': min(max_results, 50),
            'start': start,
            'Cover': cover,
            'CategoryId': category_id,
            'output': output,
            'Version': '20131101'
        }
        
        try:
            response = requests.get(url, params=params)
            response.raise_for_status()
            
            if output == "JS":
                return response.json()
            else:
                return response.text
                
        except requests.exceptions.RequestException as e:
            print(f"API 요청 중 오류 발생: {e}")
            return None
    
    def lookup_book(self, item_id, item_id_type="ISBN13", cover="Mid", 
                   opt_result=None, output="JS"):
        """
        도서 상세 조회 API
        
        Args:
            item_id (str): 상품 ID (ISBN 또는 알라딘 상품 ID)
            item_id_type (str): ID 타입 (ISBN, ISBN13, ItemId)
            cover (str): 표지 크기
            opt_result (list): 부가 정보 요청 (ebookList, usedList, reviewList 등)
            output (str): 출력 형식
        
        Returns:
            dict: API 응답 결과
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
            response = requests.get(url, params=params)
            response.raise_for_status()
            
            if output == "JS":
                return response.json()
            else:
                return response.text
                
        except requests.exceptions.RequestException as e:
            print(f"API 요청 중 오류 발생: {e}")
            return None
    
    def search_to_dataframe(self, query, **kwargs):
        """
        검색 결과를 pandas DataFrame으로 변환
        
        Args:
            query (str): 검색어
            **kwargs: search_books의 추가 파라미터
        
        Returns:
            pd.DataFrame: 검색 결과 데이터프레임
        """
        result = self.search_books(query, **kwargs)
        
        if not result or 'item' not in result:
            print("검색 결과가 없습니다.")
            return pd.DataFrame()
        
        books = []
        for item in result['item']:
            book_data = {
                'title': item.get('title', ''),
                'author': item.get('author', ''),
                'publisher': item.get('publisher', ''),
                'pubDate': item.get('pubDate', ''),
                'isbn': item.get('isbn', ''),
                'isbn13': item.get('isbn13', ''),
                'priceStandard': item.get('priceStandard', 0),
                'priceSales': item.get('priceSales', 0),
                'description': item.get('description', ''),
                'cover': item.get('cover', ''),
                'link': item.get('link', ''),
                'customerReviewRank': item.get('customerReviewRank', 0)
            }
            books.append(book_data)
        
        return pd.DataFrame(books)
    
    def display_search_results(self, query, max_results=10, show_cover=True):
        """
        검색 결과를 보기 좋게 출력
        
        Args:
            query (str): 검색어
            max_results (int): 최대 결과 수
            show_cover (bool): 표지 이미지 URL 표시 여부
        """
        df = self.search_to_dataframe(query, max_results=max_results, cover="Big")
        
        if df.empty:
            print("검색 결과가 없습니다.")
            return
        
        print(f"'{query}' 검색 결과 ({len(df)}건)")
        print("=" * 80)
        
        for idx, row in df.iterrows():
            print(f"{idx + 1}. {row['title']}")
            print(f"   저자: {row['author']}")
            print(f"   출판사: {row['publisher']} | 출간일: {row['pubDate']}")
            print(f"   정가: {row['priceStandard']:,}원 | 판매가: {row['priceSales']:,}원")
            print(f"   ISBN13: {row['isbn13']}")
            if show_cover and row['cover']:
                print(f"   표지 이미지: {row['cover']}")
            if row['description']:
                desc = row['description'][:100] + "..." if len(row['description']) > 100 else row['description']
                print(f"   설명: {desc}")
            print(f"   링크: {row['link']}")
            print("-" * 80)
    
    def download_cover_image(self, cover_url, filename=None):
        """
        표지 이미지 다운로드
        
        Args:
            cover_url (str): 표지 이미지 URL
            filename (str): 저장할 파일명 (None이면 자동 생성)
        
        Returns:
            str: 저장된 파일 경로
        """
        try:
            response = requests.get(cover_url)
            response.raise_for_status()
            
            if not filename:
                # URL에서 파일명 추출
                filename = cover_url.split('/')[-1]
                if '.' not in filename:
                    filename += '.jpg'
            
            with open(filename, 'wb') as f:
                f.write(response.content)
            
            print(f"표지 이미지 저장 완료: {filename}")
            return filename
            
        except requests.exceptions.RequestException as e:
            print(f"이미지 다운로드 중 오류 발생: {e}")
            return None
    
    def display_book_with_cover(self, query, index=0):
        """
        검색 결과 중 특정 책의 정보와 표지를 주피터 노트북에서 표시
        
        Args:
            query (str): 검색어
            index (int): 표시할 책의 인덱스 (0부터 시작)
        """
        try:
            from IPython.display import Image, display, HTML
            import requests
            from io import BytesIO
        except ImportError:
            print("주피터 노트북 환경이 아니거나 필요한 라이브러리가 없습니다.")
            return
        
        df = self.search_to_dataframe(query, max_results=10, cover="Big")
        
        if df.empty or index >= len(df):
            print("유효하지 않은 인덱스이거나 검색 결과가 없습니다.")
            return
        
        book = df.iloc[index]
        
        # 책 정보 HTML로 표시
        html_content = f"""
        <div style="display: flex; margin: 20px; padding: 20px; border: 1px solid #ddd; border-radius: 10px;">
            <div style="margin-right: 20px;">
                <img src="{book['cover']}" style="max-width: 200px; border-radius: 5px;" />
            </div>
            <div style="flex: 1;">
                <h2 style="margin-top: 0; color: #333;">{book['title']}</h2>
                <p><strong>저자:</strong> {book['author']}</p>
                <p><strong>출판사:</strong> {book['publisher']}</p>
                <p><strong>출간일:</strong> {book['pubDate']}</p>
                <p><strong>정가:</strong> {book['priceStandard']:,}원</p>
                <p><strong>판매가:</strong> {book['priceSales']:,}원</p>
                <p><strong>ISBN13:</strong> {book['isbn13']}</p>
                <p><strong>설명:</strong> {book['description'][:200]}{"..." if len(book['description']) > 200 else ""}</p>
                <p><a href="{book['link']}" target="_blank">알라딘에서 보기</a></p>
            </div>
        </div>
        """
        
        display(HTML(html_content))

# 사용 예시
def main():
    # API 키 설정
    TTB_KEY = "ttbleeshun08062356001"
    
    # API 객체 생성
    aladin = AladinAPI(TTB_KEY)
    
    # 1. 키워드로 도서 검색 (표지 이미지 포함)
    print("=== 키워드 검색 예시 (표지 이미지 URL 포함) ===")
    aladin.display_search_results("파이썬", max_results=3, show_cover=True)
    
    print("\n" + "="*100 + "\n")
    
    # 2. 큰 표지 이미지로 검색
    print("=== 큰 표지 이미지로 검색 ===")
    big_cover_df = aladin.search_to_dataframe("머신러닝", max_results=3, cover="Big")
    if not big_cover_df.empty:
        first_book = big_cover_df.iloc[0]
        print(f"첫 번째 책: {first_book['title']}")
        print(f"큰 표지 이미지 URL: {first_book['cover']}")
        
        # 표지 이미지 다운로드 (선택사항)
        # aladin.download_cover_image(first_book['cover'], f"{first_book['title'][:10]}_cover.jpg")
    
    print("\n" + "="*100 + "\n")
    
    # 3. 주피터 노트북에서 표지와 함께 표시 (주피터에서만 작동)
    print("=== 주피터 노트북용 표지 표시 기능 ===")
    print("주피터 노트북에서 다음 코드를 실행하세요:")
    print("aladin.display_book_with_cover('딥러닝', index=0)")

# 주피터 노트북 전용 사용 예시
def jupyter_examples():
    """
    주피터 노트북에서 사용할 수 있는 예시 함수들
    """
    TTB_KEY = "ttbleeshun08062356001"
    aladin = AladinAPI(TTB_KEY)
    
    # 1. 표지 이미지와 함께 책 정보 표시
    aladin.display_book_with_cover("파이썬", index=0)
    
    # 2. 여러 책의 표지 비교
    df = aladin.search_to_dataframe("머신러닝", max_results=5, cover="Big")
    
    from IPython.display import Image, display, HTML
    
    html = "<div style='display: flex; flex-wrap: wrap;'>"
    for idx, book in df.iterrows():
        html += f"""
        <div style='margin: 10px; text-align: center; width: 200px;'>
            <img src='{book["cover"]}' style='width: 150px; height: 200px; object-fit: cover;'/>
            <p style='font-size: 12px; margin: 5px 0;'>{book["title"][:20]}...</p>
            <p style='font-size: 10px; color: #666;'>{book["author"]}</p>
        </div>
        """
    html += "</div>"
    
    display(HTML(html))

# 실행
if __name__ == "__main__":
    main()
```

예시 코드2
```python
```python
# 1. 기본 설정
TTB_KEY = "ttbleeshun08062356001"
aladin = AladinAPI(TTB_KEY)

# 2. 표지 이미지 포함 검색
df = aladin.search_to_dataframe("파이썬 프로그래밍", max_results=10, cover="Big")

# 3. 첫 번째 책의 표지와 정보를 예쁘게 표시
aladin.display_book_with_cover("파이썬 프로그래밍", index=0)

# 4. 표지 이미지들을 갤러리로 표시
from IPython.display import Image, display, HTML

html = "<div style='display: flex; flex-wrap: wrap;'>"
for idx, book in df.head(6).iterrows():
    html += f"""
    <div style='margin: 10px; text-align: center; width: 180px;'>
        <img src='{book["cover"]}' style='width: 150px; height: 200px; object-fit: cover; border-radius: 5px;'/>
        <p style='font-size: 12px; margin: 5px 0; font-weight: bold;'>{book["title"][:25]}...</p>
        <p style='font-size: 10px; color: #666;'>{book["author"]}</p>
        <p style='font-size: 10px; color: #e74c3c;'>{book["priceSales"]:,}원</p>
    </div>
    """
html += "</div>"
display(HTML(html))
```


## 2단계: 이제 kiosk에서 도서 검색했을때 정보 조회
kiosk에서 도서 조회를 했을때 밑바닥부터 시작하는 딥러닝 1(리마스터판)이 검색 목록에 나오게 하기
검색목록에서 이 책을 클릭하면 팝업창이 나오고 팝업창 구조는 왼쪽에는 맵과 책이 있는 위치 빨간원으로 표시
오른쪽에는 책에 대한 세부정보 표시
오른쪽 밑에는 에스코팅 요청 버튼이 있을 수 있도록 

2단계 시퀀스 다이어 그램

@startuml 2단계_키오스크_도서검색_정보조회
!theme plain
skinparam backgroundColor #FFFFFF
skinparam sequenceArrowThickness 2
skinparam roundcorner 20
skinparam maxmessagesize 60

title 2단계: 키오스크에서 도서 검색 및 정보 조회

actor "사용자" as User
participant "Kiosk UI" as KioskUI
participant "BookSearchWidget" as BookWidget
participant "BookSearchClient" as SearchClient
participant "Main Server" as MainServer
database "MySQL DB" as DB

User -> KioskUI: 키오스크 메인 화면
activate KioskUI

User -> KioskUI: "Book Search" 버튼 클릭
KioskUI -> BookWidget: BookSearchWidget 생성 및 표시
activate BookWidget

User -> BookWidget: 검색어 입력\n("딥러닝")
User -> BookWidget: "검색" 버튼 클릭
activate BookWidget

BookWidget -> SearchClient: 도서 검색 요청\n(query: "딥러닝", search_type: "title")
activate SearchClient

SearchClient -> MainServer: /book_search 서비스 호출
activate MainServer

MainServer -> DB: SELECT * FROM books\nWHERE title LIKE '%딥러닝%'
activate DB
DB --> MainServer: 검색 결과 반환
deactivate DB

MainServer --> SearchClient: BookSearch.srv 응답\n(도서 목록)
deactivate MainServer

SearchClient --> BookWidget: 검색 결과 시그널
deactivate SearchClient

BookWidget -> BookWidget: 검색 결과 UI 업데이트\n(도서 목록 표시)
deactivate BookWidget

User -> BookWidget: "밑바닥부터 시작하는 딥러닝 1" 클릭
BookWidget -> BookWidget: 도서 상세 정보 팝업 표시
note right of BookWidget
  팝업 내용:
  - 왼쪽: 지도 + 책 위치 (빨간 원)
  - 오른쪽: 도서 상세 정보
  - 하단: 에스코팅 요청 버튼
end note

deactivate BookWidget
deactivate KioskUI
@enduml

## 3단계: 책 서적 위치까지 에스코팅
팝업창에서 에스코팅 요청 버튼을 누르면 키오스크는 libo를 호출했습니다라고 팝업창 띄움 그리고 5초뒤에 키오스크느 메인화면으로 이동
libo는 키오스크 위치로 와서 "서적 위치까지 에스코팅을 시작하겠습니다"라는 터미널에 메세지를 남기고 서적 위치로 에스코팅 하기
키오스크 위치 좌표: 'B1': (5.57, 4.90, 0.0),
컴퓨터 서적 좌표: 'D3': (0.03, 0.96, 0.0), 'D5': (2.92, 0.98, 0.0), 둘중 어는곳이든 상관없음
base 좌표: 'E3': (0.05, -0.34, 0.0),


3단계 시쿼스 다이어그램
@startuml 3단계_에스코팅요청_로봇호출
!theme plain
skinparam backgroundColor #FFFFFF
skinparam sequenceArrowThickness 2
skinparam roundcorner 20
skinparam maxmessagesize 60

title 3단계: 에스코팅 요청 및 로봇 호출

actor "사용자" as User
participant "BookSearchWidget" as BookWidget
participant "Kiosk UI" as KioskUI
participant "EscortRequestClient" as EscortClient
participant "Libo Service" as LiboService
participant "Libo Operator" as LiboOperator

User -> BookWidget: "에스코팅 요청" 버튼 클릭
activate BookWidget

BookWidget -> EscortClient: 에스코팅 요청\n(robot_id: "robot_01", book_title: "밑바닥부터 시작하는 딥러닝 1", book_location: "D3")
activate EscortClient

EscortClient -> LiboService: /escort_request 서비스 호출
activate LiboService

LiboService -> LiboService: 에스코팅 요청 검증
LiboService -> LiboService: escort_id 생성
LiboService --> EscortClient: EscortRequest.srv 응답\n(success: true, escort_id: "escort_001")
deactivate LiboService

EscortClient --> BookWidget: 에스코팅 요청 완료
deactivate EscortClient

BookWidget -> BookWidget: "리보를 호출했습니다" 팝업 표시
BookWidget -> BookWidget: 5초 후 메인 화면으로 이동
deactivate BookWidget

BookWidget -> KioskUI: 메인 화면 복귀
activate KioskUI

LiboService -> LiboService: 에스코팅 시작 처리
LiboService -> LiboService: "서적 위치까지 에스코팅을 시작하겠습니다" 터미널 메시지

LiboService -> LiboOperator: 키오스크 위치로 이동 명령\n(waypoint: "B1")
activate LiboOperator

LiboOperator -> LiboOperator: 키오스크 위치로 이동\n(B1: 5.57, 4.90, 0.0)
LiboOperator --> LiboService: 키오스크 도착 알림

LiboService -> LiboOperator: 서적 위치로 에스코팅\n(waypoint: "D3")
LiboOperator -> LiboOperator: 서적 위치로 이동\n(D3: 0.03, 0.96, 0.0)

deactivate LiboOperator
deactivate KioskUI
@enduml


## 4단계: 에스코팅 완료
해당 위치까지 도착했으면 에스코팅을 완료했습니다라고 터미널에 메세지를 남기고 1초후 또 이용해 주세요 라고 터미널에 메세지 남기고 3초뒤에 base로 복귀

4단계 시퀀스 다이어그램
@startuml 4단계_에스코팅완료_복귀
!theme plain
skinparam backgroundColor #FFFFFF
skinparam sequenceArrowThickness 2
skinparam roundcorner 20
skinparam maxmessagesize 60

title 4단계: 에스코팅 완료 및 복귀

participant "Libo Operator" as LiboOperator
participant "Libo Service" as LiboService
participant "Admin GUI" as AdminGUI

LiboOperator -> LiboService: 서적 위치 도착 알림\n(waypoint: "D3")
activate LiboService

LiboService -> LiboService: "에스코팅을 완료했습니다" 터미널 메시지
LiboService -> LiboService: 1초 대기
LiboService -> LiboService: "또 이용해 주세요" 터미널 메시지
LiboService -> LiboService: 3초 대기

LiboService -> LiboOperator: Base 위치로 복귀 명령\n(waypoint: "E3")
activate LiboOperator

LiboOperator -> LiboOperator: Base 위치로 이동\n(E3: 0.05, -0.34, 0.0)
LiboOperator --> LiboService: Base 도착 알림

LiboService -> LiboService: 에스코팅 세션 종료
LiboService -> AdminGUI: /escort_complete 토픽 발행\n(escort_id: "escort_001", final_location: "D3")

note right of LiboService
  에스코팅 완료 처리:
  1. 서적 위치 도착 확인
  2. 완료 메시지 출력
  3. 1초 대기 후 안내 메시지
  4. 3초 대기 후 Base 복귀
  5. 에스코팅 세션 종료
end note

deactivate LiboOperator
deactivate LiboService
@enduml