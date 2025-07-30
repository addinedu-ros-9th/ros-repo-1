# Stock Package

도서 입고 관리를 위한 ROS2 패키지입니다.

## 기능

- ISBN으로 도서 검색
- **바코드 스캔을 통한 자동 ISBN 입력**
- 알라딘 API를 통한 도서 정보 조회
- GUI를 통한 도서 입고 관리
- 데이터베이스 자동 저장

## 사용법

### 빌드
```bash
cd ros_ws
colcon build --packages-select stock
source install/setup.bash
```

### 실행
```bash
ros2 run stock stock_gui
```

### 바코드 스캔 사용법

1. **바코드 스캔 버튼 클릭**: UI의 "바코드 스캔" 버튼을 클릭합니다.
2. **카메라 권한 허용**: 시스템에서 카메라 접근 권한을 허용합니다.
3. **바코드 비추기**: 도서의 바코드를 카메라에 비춥니다.
4. **자동 검색**: 바코드가 감지되면 자동으로 ISBN이 입력되고 검색이 실행됩니다.
5. **스캔 중지**: 바코드가 감지되면 자동으로 스캔이 중지됩니다.

### 바코드 스캔 테스트

바코드 스캔 기능을 독립적으로 테스트하려면:
```bash
cd ros_ws/src/stock
python3 test_barcode_scanner.py
```

## UI 구성

- **ISBN 검색**: 도서의 ISBN을 입력하여 검색
- **검색 버튼**: 입력된 ISBN으로 도서 검색
- **바코드 스캔 버튼**: 카메라를 통한 바코드 스캔으로 자동 ISBN 입력
- **입고 버튼**: 선택된 도서를 데이터베이스에 등록
- **도서 리스트**: 검색된 도서 목록 표시
- **도서 정보**: 선택된 도서의 상세 정보 표시

## 의존성

- ROS2 Jazzy
- PyQt5
- opencv-python (바코드 스캔용)
- imutils (비디오 스트림용)
- pyzbar (바코드 디코딩용)
- numpy
- pymysql
- requests
- main_server 패키지의 데이터베이스 및 API 클라이언트

## 카테고리별 위치 매핑

- 컴퓨터: D5
- 언어: D7  
- 소설: C8 