# 필요한 라이브러리들을 가져옵니다.
import cv2  # OpenCV: 카메라 제어 및 이미지 처리를 위해 사용
import socket  # 소켓: 네트워크 통신(UDP)을 위해 사용
import time  # 시간 관련 함수: 전송 간격 조절(딜레이)을 위해 사용
import json  # JSON: 데이터 구조를 만들기 위해 사용 (헤더 생성)
from datetime import datetime, timezone  # 날짜 및 시간: 타임스탬프 생성을 위해 사용

def main(args=None):
    # ===== 전송 대상 설정 =====
    # 모니터링 서비스 (옵션)
    UDP_IP_MONITORING = "127.0.0.1"
    UDP_PORT_MONITORING = 7001
    SEND_TO_MONITORING = False

    # ADMIN_PC로 영상 전송
    ADMIN_PC_IP = "127.0.0.1"  # 로컬 테스트용으로 localhost로 변경
    ADMIN_PC_PORT = 7021
    SEND_TO_ADMIN_PC = True

    # 사용할 카메라 경로 (로컬 테스트용)
    # Linux: 0, 1, 2... 또는 /dev/video0, /dev/video1...
    # Windows: 0, 1, 2...
    # macOS: 0, 1, 2...
    WEBCAM_PATH = '/dev/video1'  # 로컬 테스트용으로 video1 사용
    
    # udev 규칙으로 고정된 경로 (실제 로봇 환경용)
    # WEBCAM_PATH = '/dev/my_webcam_2'

    # ===== 소켓 설정 =====
    sock_monitoring = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_admin = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ===== 카메라 초기화 =====
    print(f"🔍 카메라 연결 시도 중... (경로: {WEBCAM_PATH})")
    
    # 로컬 환경에서는 cv2.CAP_V4L2 대신 기본 백엔드 사용
    if isinstance(WEBCAM_PATH, int):
        cap = cv2.VideoCapture(WEBCAM_PATH)  # 숫자인 경우 기본 백엔드 사용
    else:
        cap = cv2.VideoCapture(WEBCAM_PATH, cv2.CAP_V4L2)  # 경로인 경우 V4L2 사용
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 사이즈 줄여서 지연 최소화
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 해상도 설정
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"❌ 카메라 연결 실패: '{WEBCAM_PATH}'")
        print("💡 다른 카메라 번호를 시도해보세요 (0, 1, 2...)")
        return
    
    print(f"✅ 카메라 연결 성공!")
    
    # 카메라 정보 출력
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"📷 카메라 정보: {width}x{height}, FPS: {fps}")

    frame_id = 0

    try:
        print(f"✅ 카메라 스트리밍 시작...")
        print(f"📡 전송 대상: {ADMIN_PC_IP}:{ADMIN_PC_PORT}")
        print(f"⏱️  전송 간격: 0.07초 (약 30fps)")
        print("🛑 중단하려면 Ctrl+C를 누르세요")
        print("-" * 50)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️  프레임 캡처 실패.")
                continue

            frame = cv2.resize(frame, (640, 480))

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                print("⚠️  JPEG 인코딩 실패.")
                continue

            jpeg_bytes = jpeg.tobytes()

            header = {
                "direction": "front",  # front로 수정 (원래 rear였음)
                "frame_id": frame_id,
                "timestamp": datetime.now(timezone.utc).isoformat()
            }
            header_str = json.dumps(header)
            message = header_str.encode() + b'|' + jpeg_bytes + b'\n'

            if SEND_TO_MONITORING:
                sock_monitoring.sendto(message, (UDP_IP_MONITORING, UDP_PORT_MONITORING))
            if SEND_TO_ADMIN_PC:
                sock_admin.sendto(message, (ADMIN_PC_IP, ADMIN_PC_PORT))

            frame_id += 1
            
            # 100프레임마다 상태 출력
            if frame_id % 100 == 0:
                print(f"📊 전송된 프레임: {frame_id}개")
            
            time.sleep(0.07)  # 약 30fps로 조절

    except KeyboardInterrupt:
        print("\n[종료] cam_sender stopped by user.")

    finally:
        print("Releasing camera and closing sockets...")
        cap.release()
        sock_monitoring.close()
        sock_admin.close()

if __name__ == '__main__':
    main()
