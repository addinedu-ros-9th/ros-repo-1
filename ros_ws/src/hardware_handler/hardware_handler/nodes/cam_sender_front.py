# 필요한 라이브러리들을 가져옵니다.
import cv2  # OpenCV: 카메라 제어 및 이미지 처리를 위해 사용
import socket  # 소켓: 네트워크 통신(UDP)을 위해 사용
import time  # 시간 관련 함수: 전송 간격 조절(딜레이)을 위해 사용
import json  # JSON: 데이터 구조를 만들기 위해 사용 (헤더 생성)
from datetime import datetime, timezone  # 날짜 및 시간: 타임스탬프 생성을 위해 사용
import threading  # 스레딩: 동시 실행을 위해 추가
import queue      # 큐: 스레드 간 안전한 데이터 교환을 위해 추가

def udp_sender(sock, data_queue, destination_address, name):
    """
    큐에서 메시지를 가져와 지정된 주소로 UDP 패킷을 전송하는 스레드 함수.
    """
    print(f"✅ Starting sender thread for {name} -> {destination_address}")
    while True:
        try:
            # 큐에서 보낼 메시지를 가져옴. 큐가 비어있으면 메시지가 들어올 때까지 대기.
            message = data_queue.get()
            sock.sendto(message, destination_address)
        except Exception as e:
            print(f"⚠️ Error in {name} sender thread: {e}")
            time.sleep(1) # 오류 발생 시 잠시 대기

def main(args=None):
    # ===== 전송 대상 설정 (이제 모두 필수) =====
    # 모니터링 서비스
    # UDP_IP_MONITORING = "127.0.0.1"
    UDP_IP_MONITORING = "192.168.1.7"
    UDP_PORT_MONITORING = 7022

    # ADMIN_PC로 영상 전송
    # ADMIN_PC_IP = "192.168.1.2"
    ADMIN_PC_IP = "192.168.1.2"
    ADMIN_PC_PORT = 7021

    # 사용할 카메라 경로 (udev 규칙으로 고정된 경로)
    # WEBCAM_PATH = '/dev/my_webcam_2'
    WEBCAM_PATH = '/dev/my_rear_cam'
    
    # ===== 스레드 간 데이터 전송을 위한 큐 생성 =====
    # maxsize를 지정하여 한쪽 스레드가 멈췄을 때 메모리가 무한정 쌓이는 것을 방지
    q_monitoring = queue.Queue(maxsize=10)
    q_admin = queue.Queue(maxsize=10)


    # ===== 소켓 설정 =====
    sock_monitoring = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_admin = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ===== 전송 스레드 생성 및 시작 =====
    # daemon=True: 메인 프로그램이 종료되면 스레드도 함께 종료됨
    thread_monitoring = threading.Thread(
        target=udp_sender,
        args=(sock_monitoring, q_monitoring, (UDP_IP_MONITORING, UDP_PORT_MONITORING), "Monitoring"),
        daemon=True
    )
    thread_admin = threading.Thread(
        target=udp_sender,
        args=(sock_admin, q_admin, (ADMIN_PC_IP, ADMIN_PC_PORT), "AdminPC"),
        daemon=True
    )
    thread_monitoring.start()
    thread_admin.start()


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

            # 헤더 생성
            header = {
                "direction": "rear",
                "frame_id": frame_id,
                "timestamp": datetime.now(timezone.utc).isoformat()
            }
            header_str = json.dumps(header)
            message = header_str.encode() + b'|' + jpeg_bytes + b'\n'

            # ===== 생성된 메시지를 각 큐에 넣기 =====
            # 큐가 가득 차 있으면 오래된 프레임은 버리고 계속 진행 (put_nowait)
            try:
                q_monitoring.put_nowait(message)
            except queue.Full:
                print("⚠️ Monitoring queue is full, dropping a frame.")
            
            try:
                q_admin.put_nowait(message)
            except queue.Full:
                print("⚠️ Admin PC queue is full, dropping a frame.")

            frame_id += 1
            time.sleep(0.033)  # 약 30fps로 조절 (기존 0.07 -> 약 14fps)

    except KeyboardInterrupt:
        print("\n[종료] cam_sender stopped by user.")

    finally:
        print("Releasing camera and closing sockets...")
        cap.release()
        sock_monitoring.close()
        sock_admin.close()

if __name__ == '__main__':
    main()