# 알라딘 API 문서

**최종업데이트: 2022-07-13**

## 목차

### 1. 요청 (Request)
1. [상품 검색 API](#1-상품-검색-api)
2. [상품 리스트 API](#2-상품-리스트-api)
3. [상품 조회 API](#3-상품-조회-api)
4. [중고상품 보유 매장 검색 API](#4-중고상품-보유-매장-검색-api)

### 2. 응답 (Response)
1. [상품 검색/상품 리스트/상품 조회 API](#1-상품-검색상품-리스트상품-조회-api-1)
2. [상품 조회 API : 부가정보](#2-상품-조회-api--부가정보)
3. [중고상품 보유 매장 검색 API](#3-중고상품-보유-매장-검색-api-1)

---

## 1. 요청 (Request)

### 1) 상품 검색 API

#### 요청 방법
- **요청 URL**: `http://www.aladin.co.kr/ttb/api/ItemSearch.aspx`
- **요청 URL 샘플**: 
  ```
  http://www.aladin.co.kr/ttb/api/ItemSearch.aspx?ttbkey=[TTBKey]&Query=aladdin&QueryType=Title&MaxResults=10&start=1&SearchTarget=Book&output=xml&Version=20131101
  ```
- **결과 샘플**:
  - XML형식: `http://www.aladin.co.kr/ttb/api/test/ItemSearch_20131101.xml`
  - JavaScript형식: `http://www.aladin.co.kr/ttb/api/test/ItemSearch_20131101.js`
- **제한사항**: 한 페이지에 최대 50개, 총 결과는 200개까지만 검색 가능

#### 상품 검색 API 요청 파라미터

| 구분 | 요청변수 | 변수종류 | 설명 |
|------|----------|----------|------|
| **필수** | TTBKey | 문자열 | 부여받은 TTBKey값 |
| | Query | 문자열 | 검색어 |
| **옵션** | QueryType | 문자열 | **Keyword (기본값)**: 제목+저자<br>**Title**: 제목검색<br>**Author**: 저자검색<br>**Publisher**: 출판사검색 |
| | SearchTarget | 문자열 | **Book(기본값)**: 도서<br>**Foreign**: 외국도서<br>**Music**: 음반<br>**DVD**: DVD<br>**Used**: 중고샵(도서/음반/DVD 등)<br>**eBook**: 전자책<br>**All**: 위의 모든 타겟(몰) |
| | Start | 정수 | 1이상, 양의 정수(기본값:1) - 검색결과 시작페이지 |
| | MaxResults | 정수 | 1이상 100이하, 양의 정수(기본값:10) - 검색결과 한 페이지당 최대 출력 개수 |
| | Sort | 문자열 | **Accuracy(기본값)**: 관련도<br>**PublishTime**: 출간일<br>**Title**: 제목<br>**SalesPoint**: 판매량<br>**CustomerRating**: 고객평점<br>**MyReviewCount**: 마이리뷰갯수 |
| | Cover | 문자열 | **Big**: 큰 크기 (너비 200px)<br>**MidBig**: 중간 큰 크기 (너비 150px)<br>**Mid(기본값)**: 중간 크기 (너비 85px)<br>**Small**: 작은 크기 (너비 75px)<br>**Mini**: 매우 작은 크기 (너비 65px)<br>**None**: 없음 |
| | CategoryId | 정수 | 양의정수 - 분야의 고유 번호(기본값:0, 전체) |
| | Output | 문자열 | **XML(기본값)**: REST XML형식<br>**JS**: JSON방식 |
| | Partner | 문자열 | 제휴와 관련한 파트너코드 |
| | includeKey | 정수 | 양의정수(기본값:0) - includeKey가 1인 경우 결과의 상품페이지 링크값에 TTBKey가 포함됨 |
| | InputEncoding | 문자열 | 인코딩의 영문이름 (기본값:utf-8) |
| | Version | 정수 | 날짜형식 (기본값: 20070901, 최신버젼: 20131101) |
| | outofStockfilter | 정수 | 양의정수(기본값:0) - 품절/절판 등 재고 없는 상품 필터링("1"이 제외 필터) |
| | RecentPublishFilter | 정수 | 0~60 사이 양의정수 (기본값:0) - 출간일(월단위) 제한 필터링 |
| | OptResult | 문자열 | 부가 정보 (Array 형태로 요청)<br>**ebookList**: 해당 종이책의 전자책 정보<br>**usedList**: 해당 상품에 등록된 중고상품 정보<br>**fileFormatList**: 전자책의 포맷 및 용량 |

### 2) 상품 리스트 API

#### 요청 방법
- **요청 URL**: `http://www.aladin.co.kr/ttb/api/ItemList.aspx`
- **요청 URL 샘플**: 
  ```
  http://www.aladin.co.kr/ttb/api/ItemList.aspx?ttbkey=[TTBKey]&QueryType=ItemNewAll&MaxResults=10&start=1&SearchTarget=Book&output=xml&Version=20131101
  ```
- **결과 샘플**:
  - XML형식: `http://www.aladin.co.kr/ttb/api/test/ItemList_20131101.xml`
  - JavaScript형식: `http://www.aladin.co.kr/ttb/api/test/ItemList_20131101.js`
- **제한사항**: 한 페이지에 최대 50개, 총 결과는 200개까지만 조회 가능

#### 상품 리스트 API 요청 파라미터

| 구분 | 요청변수 | 변수종류 | 설명 |
|------|----------|----------|------|
| **필수** | TTBKey | 문자열 | 부여받은 TTBKey값 |
| | QueryType | 문자열 | **ItemNewAll**: 신간 전체 리스트<br>**ItemNewSpecial**: 주목할 만한 신간 리스트<br>**ItemEditorChoice**: 편집자 추천 리스트<br>**Bestseller**: 베스트셀러<br>**BlogBest**: 블로거 베스트셀러 (국내도서만 조회 가능) |
| **옵션** | SearchTarget | 문자열 | **Book(기본값)**: 도서<br>**Foreign**: 외국도서<br>**Music**: 음반<br>**DVD**: DVD<br>**Used**: 중고샵<br>**eBook**: 전자책<br>**All**: 위의 모든 타겟(몰) |
| | SubSearchTarget | 문자열 | **Book**: 도서<br>**Music**: 음반<br>**DVD**: DVD<br>(SearchTarget이 중고(Used)인 경우, 서브 Mall 지정) |
| | Start | 정수 | 1이상, 양의 정수(기본값:1) - 검색결과 시작페이지 |
| | MaxResults | 정수 | 1이상 100이하, 양의 정수(기본값:10) - 검색결과 한 페이지당 최대 출력 개수 |
| | Cover | 문자열 | **Big**: 큰 크기 (너비 200px)<br>**MidBig**: 중간 큰 크기 (너비 150px)<br>**Mid(기본값)**: 중간 크기 (너비 85px)<br>**Small**: 작은 크기 (너비 75px)<br>**Mini**: 매우 작은 크기 (너비 65px)<br>**None**: 없음 |
| | CategoryId | 정수 | 양의정수 - 분야의 고유 번호(기본값:0, 전체) |
| | Output | 문자열 | **XML(기본값)**: REST XML형식<br>**JS**: JSON방식 |
| | Partner | 문자열 | 제휴와 관련한 파트너코드 |
| | includeKey | 정수 | 양의정수(기본값:0) - includeKey가 1인 경우 결과의 상품페이지 링크값에 TTBKey가 포함됨 |
| | InputEncoding | 문자열 | 인코딩의 영문이름 (기본값:utf-8) |
| | Version | 정수 | 날짜형식 (기본값: 20070901, 최신버젼: 20131101) |
| | outofStockfilter | 정수 | 양의정수(기본값:0) - 품절/절판 등 재고 없는 상품 필터링("1"이 제외 필터) |
| | Year, Month, Week | 정수 | 베스트셀러를 조회할 주간 (기본값:0)<br>QueryType=Bestseller인 경우, "Year=2022&Month=5&Week=3"형식으로 요청 |
| | OptResult | 문자열 | 부가 정보 (Array 형태로 요청)<br>**ebookList**: 해당 종이책의 전자책 정보<br>**usedList**: 해당 상품에 등록된 중고상품 정보<br>**fileFormatList**: 전자책의 포맷 및 용량 |

### 3) 상품 조회 API

#### 요청 방법
- **요청 URL**: `http://www.aladin.co.kr/ttb/api/ItemLookUp.aspx`
- **요청 URL 샘플**: 
  ```
  http://www.aladin.co.kr/ttb/api/ItemLookUp.aspx?ttbkey=[TTBKey]&itemIdType=ISBN&ItemId=[도서의ISBN]&output=xml&Version=20131101&OptResult=ebookList,usedList,reviewList
  ```
- **결과 샘플**:
  - XML형식: `http://www.aladin.co.kr/ttb/api/test/ItemLookUp_20131101.xml`
  - JavaScript형식: `http://www.aladin.co.kr/ttb/api/test/ItemLookUp_20131101.js`

> **참고**: 상품 조회 응답 결과값의 스펙은 검색 응답 결과값과 동일한 결과에 단순히 부가정보가 추가되는 것입니다.

#### 상품 조회 API 요청 파라미터

| 구분 | 요청변수 | 변수종류 | 설명 |
|------|----------|----------|------|
| **필수** | TTBKey | 문자열 | 부여받은 TTBKey값 |
| | ItemId | 문자열/숫자 | 상품을 구분짓는 유일한 값 (ItemIdType으로 정수값과 ISBN중에 택일) |
| **옵션** | ItemIdType | 문자열 | **ISBN(기본값)**: ISBN 10자리<br>**ISBN13**: ISBN 13자리<br>**ItemId**: 알라딘고유의 ItemId값<br>*가급적 13자리 ISBN을 이용해주세요* |
| | Cover | 문자열 | **Big**: 큰 크기<br>**MidBig**: 중간 큰 크기<br>**Mid(기본값)**: 중간 크기<br>**Small**: 작은 크기<br>**Mini**: 매우 작은 크기<br>**None**: 없음 |
| | Output | 문자열 | **XML(기본값)**: REST XML형식<br>**JS**: JSON방식 |
| | Partner | 문자열 | 제휴와 관련한 파트너코드 |
| | Version | 정수 | 날짜형식 (기본값: 20070901, 최신버젼: 20131101) |
| | includeKey | 정수 | 양의정수(기본값:0) - includeKey가 1인 경우 결과의 상품페이지 링크값에 TTBKey가 포함됨 |
| | offCode | 문자열 | 중고상품 보유 매장 검색 API에서 얻어낸 중고매장의 offCode값 |
| | OptResult | 문자열 | 부가 정보 (Array 형태로 요청)<br>**ebookList**: 해당 종이책의 전자책 정보<br>**usedList**: 해당 상품에 등록된 중고상품 정보<br>**fileFormatList**: 전자책의 포맷 및 용량<br>**c2binfo**: 중고C2B 매입여부 및 매입가 조회<br>**packing**: 판형 정보, 포장 관련 정보<br>**b2bSupply**: 전자책 B2B 납품가능 여부<br>**subbarcode**: 부가기호<br>**cardReviewImgList**: 카드리뷰 일부 이미지 경로<br>**ratingInfo**: 상품의 별 평점,100자평 개수, 마이리뷰 개수<br>**bestSellerRank**: 상품의 주간베스트셀러 순위<br>**previewImgList**: 미리보기 이미지 경로<br>**eventList**: 관련 이벤트 정보<br>**authors**: 상품의 저자/아티스트 정보 목록<br>**reviewList**: 상품에 등록된 리뷰 목록<br>**fulldescription**: 상품의 설명 및 출판사 상품소개<br>**fulldescription2**: 출판사(제작사) 제공 상품소개<br>**Toc**: 상품의 목차<br>**Story**: 줄거리<br>**categoryIdList**: 전체 분야<br>**mdrecommend**: 편집장의 선택<br>**phraseList**: 책속에서 (최대 3개까지 노출) |

### 4) 중고상품 보유 매장 검색 API

#### 요청 방법
- **요청 URL**: `http://www.aladin.co.kr/ttb/api/ItemOffStoreList.aspx`
- **요청 URL 샘플**: 
  ```
  http://www.aladin.co.kr/ttb/api/ItemOffStoreList.aspx?ttbkey=[TTBKey]&itemIdType=ISBN&ItemId=[도서의ISBN]&output=xml
  ```
- **결과 샘플**:
  - XML형식: `http://www.aladin.co.kr/ttb/api/test/ItemOffStoreList_20131101.xml`
  - JavaScript형식: `http://www.aladin.co.kr/ttb/api/test/ItemOffStoreList_20131101.js`

#### 중고상품 보유 매장 검색 API 요청 파라미터

| 구분 | 요청변수 | 변수종류 | 설명 |
|------|----------|----------|------|
| **필수** | TTBKey | 문자열 | 부여받은 TTBKey값 |
| | ItemId | 문자열/숫자 | 상품을 구분짓는 유일한 값 (ItemIdType으로 정수값과 ISBN중에 택일) |
| **옵션** | ItemIdType | 문자열 | **ISBN(기본값)**: ISBN 10자리<br>**ISBN13**: ISBN 13자리<br>**ItemId**: 알라딘고유의 ItemId값 |

---

## 2. 응답 (Response)

### 1) 상품 검색/상품 리스트/상품 조회 API

#### 기본 응답 필드

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| version | API Version | 정수형 날짜 |
| title | API 결과의 제목 | 문자열 |
| link | API 결과와 관련된 알라딘 페이지 URL 주소 | 문자열(URL) |
| pubDate | API 출력일 | 문자형 날짜 |
| totalResults | API의 총 결과수 | 정수 |
| startIndex | Page수 | 정수 |
| itemsPerPage | 한 페이지에 출력될 상품 수 | 정수 |
| query | API로 조회한 쿼리 | 문자열 |
| searchCategoryId | 분야로 조회한 경우 해당 분야의 ID | 정수 |
| searchCategoryName | 분야로 조회한 경우 해당 분야의 분야명 | 문자열 |

#### 상품 정보 필드 (item)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| title | 상품명 | 문자열 |
| link | 상품 링크 URL | 문자열(URL) |
| author | 저자/아티스트 | 문자열 |
| pubdate | 출간일(출시일) | 날짜 |
| description | 상품설명 (요약) | 문자열 |
| isbn | 10자리 ISBN | 문자열 |
| isbn13 | 13자리 ISBN | 문자열 |
| pricesales | 판매가 | 정수 |
| pricestandard | 정가 | 정수 |
| mallType | 상품의 몰타입 (BOOK, MUSIC, DVD, FOREIGN, EBOOK, USED) | 문자열 |
| stockstatus | 재고상태(정상유통일 경우 비어있음, 품절/절판 등) | 문자열 |
| mileage | 마일리지 | 정수 |
| cover | 커버(표지) | 문자열(URL) |
| publisher | 출판사(제작사/출시사) | 문자열 |
| salesPoint | 판매지수 | 정수 |
| adult | 성인 등급 여부 (true인 경우 성인 등급 도서) | bool |
| fixedPrice | (종이책/전자책인 경우) 정가제 여부 | bool |
| subbarcode | 부가기호 | 문자열 |
| customerReviewRank | 회원 리뷰 평점(별점 평균) : 0~10점 | 정수 |
| bestDuration | (베스트셀러인 경우만 노출) 베스트셀러 순위 관련 추가 정보 | 문자열 |
| bestRank | (베스트셀러인 경우만 노출) 베스트셀러 순위 정보 | 정수 |

#### 시리즈 정보 (seriesInfo)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| seriesId | 시리즈 ID | 정수 |
| seriesLink | 해당 시리즈의 조회 URL | 문자열(URL) |
| seriesName | 시리즈 이름 | 문자열 |

#### 부가 정보 (subInfo)

##### 전자책 리스트 (ebookList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| itemId | 해당 종이책의 전자책 ItemId | 정수 |
| ISBN | 해당 종이책의 전자책 ISBN | 문자열 |
| isbn13 | 해당 종이책의 전자책 13자리 ISBN | 문자열 |
| priceSales | 해당 종이책의 전자책 판매가 | 정수 |
| link | 해당 종이책의 전자책 상품페이지 링크 | 문자열(URL) |

##### 중고상품 리스트 (usedList)

###### 알라딘 직접 배송 중고 (aladinUsed)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| itemCount | 알라딘 직접 배송 중고의 보유 상품수 | 정수 |
| minPrice | 알라딘 직접 배송 중고의 보유 상품중 최저가 상품 판매가격 | 정수 |
| link | 알라딘 직접 배송 중고의 리스트 페이지 URL | 문자열(URL) |

###### 회원 직접 배송 중고 (userUsed)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| itemCount | 회원 직접 배송 중고의 보유 상품수 | 정수 |
| minPrice | 회원 직접 배송 중고의 보유 상품중 최저가 상품 판매가격 | 정수 |
| link | 회원 직접 배송 중고의 리스트 페이지 URL | 문자열(URL) |

###### 광활한 우주점 중고 (spaceUsed)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| itemCount | 광활한 우주점(매장 배송) 중고의 보유 상품수 | 정수 |
| minPrice | 광활한 우주점(매장 배송) 중고의 보유 상품중 최저가 상품 판매가격 | 정수 |
| link | 광활한 우주점(매장 배송) 중고의 리스트 페이지 URL | 문자열(URL) |

##### 새책 정보 (newBookList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| itemId | 중고상품의 새책 itemId | 정수 |
| isbn | 중고상품의 새책 isbn | 문자열 |
| isbn13 | 중고상품의 새책 13자리 isbn | 문자열 |
| priceSales | 중고상품의 새책 판매가격 | 정수 |
| link | 중고상품의 새책 상품페이지 URL | 문자열(URL) |

##### 종이책 정보 (paperBookList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| itemId | 전자책에 해당하는 종이책 itemId | 정수 |
| isbn | 전자책에 해당하는 종이책 isbn | 문자열 |
| priceSales | 전자책에 해당하는 종이책 판매가격 | 정수 |
| link | 전자책에 해당하는 종이책 상품페이지 URL | 문자열(URL) |

##### 파일 포맷 정보 (fileFormatList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| fileType | 전자책의 포맷정보 (ex: EPUB, PDF) | 문자열 |
| fileSize | 전자책의 용량정보 (byte단위) | 정수 |

### 2) 상품 조회 API : 부가정보

> **참고**: 주황색 정보의 경우, 일반적인 스펙에는 포함되지 않고, 별도로 협의 후 제공

#### 상품 상세 정보

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| fullDescription | 책소개 | 문자열 |
| fullDescription2 | 출판사 제공 책소개 | 문자열 |
| subTitle | 부제 | 문자열 |
| originalTitle | 원제 | 문자열 |
| itemPage | 상품의 쪽수 | 숫자 |
| subbarcode | 부가기호 | 문자열 |
| taxFree | 비과세 여부(True의 경우 비과세) | bool |
| toc | 목차 | 문자열 |

#### 이미지 및 리뷰 정보

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| previewImgList | Let's Look(미리보기) 이미지 경로 | 문자열(URL) |
| cardReviewImgList | 카드리뷰 이미지 경로 | 문자열(URL) |

#### 평점 정보 (ratingInfo)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| ratingScore | 상품의 별 평점 | 숫자 (실수) |
| ratingCount | 상품에 별을 남긴 개수 | 정수 |
| commentReviewCount | 100자평 남긴 개수 | 정수 |
| myReviewCount | 마이리뷰 남긴 개수 | 정수 |

#### 저자 정보 (authors)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| authorId | 참여 작가/아티스트의 ID | 정수 |
| authorName | 참여 작가/아티스트의 이름 | 문자열 |
| authorType | 참여 작가/아티스트의 참여 타입 | 문자열 |
| authorTypeDesc | 참여 작가/아티스트의 참여 타입 설명 | 문자열 |
| authorInfo | 참여 작가/아티스트의 정보 및 소개 | 문자열 |
| authorInfoLink | 참여 작가/아티스트의 정보 및 소개 페이지 URL | 문자열(URL) |

#### 중고 매입 정보

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| c2bsales | 중고 C2B 매입 여부. 1이면 매입가능. 2면 매입불가 상태 | 숫자 |
| c2bsales_price | 중고 C2B 매입가. AA는 최상급 상태일 경우, A는 상급, B는 중급, C는 균일가매입 | 숫자(금액 원) |

#### DVD 관련 정보

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| catno | 음반 고유의 번호 | 문자열 |
| recommendationComment | 음반 추천글 | 문자열 |
| specialFeature | DVD의 Special Feature | 문자열 |
| story | 도서/DVD의 줄거리 | 문자열 |
| disc | DVD의 Disc장수 | 정수 |
| playtime | DVD의 상영시간 | 문자열 |
| language | DVD의 언어 | 문자열 |
| caption | DVD의 자막 | 문자열 |
| screenrate | DVD의 화면비율 | 문자열 |
| recordingtype | DVD의 오디오 | 문자열 |
| areacode | DVD의 지역코드 | 숫자 |

#### 이벤트 정보 (eventList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| link | 상품과 연관된 이벤트 페이지 URL | 문자열(URL) |
| title | 상품과 연관된 이벤트 제목 | 문자열 |

#### 리뷰 정보 (reviewList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| reviewRank | 상품에 등록된 리뷰 평점 (10점 만점) | 정수 |
| writer | 상품에 등록된 리뷰 작성자(닉네임) | 문자열 |
| link | 상품에 등록된 리뷰 연결 URL | 문자열(URL) |
| title | 상품에 등록된 리뷰 제목 | 문자열 |

#### 중고 매장 정보 (offStoreInfo)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| offCode | 중고 매장의 offCode 값 | 문자열 |
| offName | 중고 매장의 매장명 | 문자열 |
| link | 중고 매장 상품페이지 링크 URL | 문자열(URL) |
| hasStock | 중고 매장내 상품 보유 여부 (1:보유, 0:보유하지 않음) | 정수(bool) |
| maxPrice | 중고 매장내 상품 중 최고가 금액 | 정수 |
| minPrice | 중고 매장내 상품 중 최저가 금액 | 정수 |
| location | 중고 매장내 상품의 위치 정보 | 문자열 |

#### 포장 정보 (packing)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| styleDesc | 판형 정보 (예: 양장본, 반양장본 등등) | 문자열 |
| weight | 무게 (그램 기준) | 정수 |
| sizeDepth | 깊이 (mm 기준) | 정수 |
| sizeHeight | 세로 (mm 기준) | 정수 |
| sizeWidth | 가로 (mm 기준) | 정수 |

#### 책속에서 정보 (phraseList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| pageNo | 책속에서 코너의 페이지 범위 정보 | 문자열 |
| phrase | 책속에서 코너의 문구 정보, 혹은 이미지 정보 (HTML) | 문자열 |

#### 편집장의 선택 정보 (mdRecommendList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| title | 편집장의 선택 코너의 제목 | 문자열 |
| comment | 편집장의 선택 코너의 내용 (HTML) | 문자열 |
| mdName | 편집장의 선택 코너의 MD이름/선택 날짜 | 문자열 |

#### 카테고리 정보 (categoryIdList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| categoryId | 전체 분야 정보 - 카테고리 ID | 정수 |
| categoryName | 전체 분야 정보 - 카테고리 명 | 문자열 |

### 3) 중고상품 보유 매장 검색 API

#### 기본 응답 필드

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| version | API Version | 정수형 날짜 |
| link | API 결과와 관련된 알라딘 페이지 URL 주소 | 문자열(URL) |
| pubDate | API 출력일 | 문자형 날짜 |
| query | API로 조회한 쿼리 | 문자열 |

#### 중고 매장 정보 (itemOffStoreList)

| 필드명 | 설명 | 자료형 |
|--------|------|--------|
| offCode | 중고 매장의 offCode 값 | 문자열 |
| offName | 중고 매장의 매장 명 | 문자열 |
| link | 중고 매장 상품 링크 URL | 문자열(URL) |

---

## 주요 사항

1. **TTBKey**: 모든 API 호출시 필수로 제공해야 하는 인증키입니다.

2. **버전 정보**: 최신 버전은 20131101입니다. 가급적 최신 버전을 사용하시기 바랍니다.

3. **ISBN 사용**: 상품 조회시 가급적 13자리 ISBN을 사용하시기 바랍니다.

4. **검색 제한**: 
   - 한 페이지에 최대 50개 결과
   - 전체 검색 결과는 최대 200개까지

5. **인코딩**: 기본 인코딩은 UTF-8이며, EUC-KR도 지원합니다.

6. **출력 형식**: XML(기본값) 또는 JSON 형식으로 결과를 받을 수 있습니다.

7. **부가 정보**: OptResult 파라미터를 통해 다양한 부가 정보를 요청할 수 있습니다.

---

## 예시 URL

### 상품 검색 예시
```
http://www.aladin.co.kr/ttb/api/ItemSearch.aspx?ttbkey=YOUR_TTB_KEY&Query=파이썬&QueryType=Title&MaxResults=10&start=1&SearchTarget=Book&output=xml&Version=20131101
```

### 상품 리스트 예시
```
http://www.aladin.co.kr/ttb/api/ItemList.aspx?ttbkey=YOUR_TTB_KEY&QueryType=Bestseller&MaxResults=10&start=1&SearchTarget=Book&output=xml&Version=20131101
```

### 상품 조회 예시
```
http://www.aladin.co.kr/ttb/api/ItemLookUp.aspx?ttbkey=YOUR_TTB_KEY&itemIdType=ISBN13&ItemId=9791234567890&output=xml&Version=20131101&OptResult=ebookList,usedList,reviewList
```

### 중고상품 보유 매장 검색 예시
```
http://www.aladin.co.kr/ttb/api/ItemOffStoreList.aspx?ttbkey=YOUR_TTB_KEY&itemIdType=ISBN13&ItemId=9791234567890&output=xml
```

---

**문서 버전**: 2022-07-13 최종 업데이트