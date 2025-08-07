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
    ADMIN_PC_IP = "192.168.1.7"
    ADMIN_PC_PORT = 7021
    SEND_TO_ADMIN_PC = True

    # 사용할 카메라 경로 (udev 규칙으로 고정된 경로)
    WEBCAM_PATH = '/dev/my_webcam_2'

    # ===== 소켓 설정 =====
    sock_monitoring = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_admin = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ===== 카메라 초기화 =====
    cap = cv2.VideoCapture(WEBCAM_PATH, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 사이즈 줄여서 지연 최소화

    if not cap.isOpened():
        raise RuntimeError(f"Camera path '{WEBCAM_PATH}' not found or could not be opened.")

    frame_id = 0

    try:
        print(f"✅ Starting camera streaming from '{WEBCAM_PATH}'...")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️  Failed to capture frame.")
                continue

            frame = cv2.resize(frame, (640, 480))

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                print("⚠️  JPEG encoding failed.")
                continue

            jpeg_bytes = jpeg.tobytes()

            header = {
                "direction": "rear",
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
