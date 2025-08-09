# 필요한 라이브러리들을 가져옵니다.
import cv2  # OpenCV: 카메라 제어 및 이미지 처리를 위해 사용
import socket  # 소켓: 네트워크 통신(UDP)을 위해 사용
import time  # 시간 관련 함수: 전송 간격 조절(딜레이)을 위해 사용
import json  # JSON: 데이터 구조를 만들기 위해 사용 (헤더 생성)
from datetime import datetime, timezone  # 날짜 및 시간: 타임스탬프 생성을 위해 사용
import threading  # 스레딩: 동시 실행을 위해 추가
import queue      # 큐: 스레드 간 안전한 데이터 교환을 위해 추가

# --- 스레드 작업 함수 ---
def udp_sender(sock, data_queue, destination_address, name):
    """
    큐(Queue)에서 메시지를 계속 꺼내서 지정된 주소로 UDP 패킷을 전송합니다.
    이 함수는 각 전송 대상마다 별도의 스레드에서 실행됩니다.
    """
    print(f"✅ Starting sender thread for {name} -> {destination_address}")
    while True:
        try:
            # 큐에 메시지가 들어올 때까지 대기(blocking)하고, 들어오면 가져옵니다.
            message = data_queue.get()
            # 가져온 메시지를 지정된 목적지로 전송합니다.
            sock.sendto(message, destination_address)
        except Exception as e:
            # 스레드 내에서 오류 발생 시, 해당 스레드만 영향을 받도록 처리합니다.
            print(f"⚠️ Error in '{name}' sender thread: {e}")
            time.sleep(1) # 오류 발생 시 잠시 대기 후 재시도

# --- 메인 프로그램 ---
def main(args=None):
    # ===== 설정: 전송할 대상 및 카메라 옵션을 정의하는 부분 =====
    # 모니터링 서비스(수신자 1)의 IP 주소 및 포트
    # UDP_IP_MONITORING = "127.0.0.1"
    UDP_IP_MONITORING = "192.168.1.2"
    # UDP_IP_MONITORING = "192.168.1.7"
    UDP_PORT_MONITORING = 7101

    # AI 서비스(수신자 2)의 IP 주소 및 포트
    UDP_IP_FOLLOWING = "127.0.0.1"
    # UDP_IP_FOLLOWING = "192.168.1.7" # 실제 IP 사용 시 주석 해제
    UDP_PORT_FOLLOWING = 7020

    # 사용할 카메라의 고정 경로
    WEBCAM_PATH = '/dev/my_webcam'
    
    # ===== 스레드 간 데이터 전송을 위한 큐 생성 =====
    # 각 목적지별로 독립된 큐를 생성합니다.
    # maxsize는 큐에 쌓일 수 있는 최대 데이터 개수로, 메모리가 과도하게 사용되는 것을 방지합니다.
    q_monitoring = queue.Queue(maxsize=10)
    q_following = queue.Queue(maxsize=10)

    # ===== 소켓 설정: UDP 통신을 위한 소켓을 생성하는 부분 =====
    sock_monitoring = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_following = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ===== 전송 스레드 생성 및 시작 =====
    # daemon=True: 메인 프로그램이 종료되면 이 스레드들도 함께 자동 종료됩니다.
    thread_monitoring = threading.Thread(
        target=udp_sender, 
        args=(sock_monitoring, q_monitoring, (UDP_IP_MONITORING, UDP_PORT_MONITORING), "Monitoring"),
        daemon=True
    )
    thread_following = threading.Thread(
        target=udp_sender,
        args=(sock_following, q_following, (UDP_IP_FOLLOWING, UDP_PORT_FOLLOWING), "AI_Following"),
        daemon=True
    )
    
    # 생성된 스레드들을 실행합니다.
    thread_monitoring.start()
    thread_following.start()

    # ===== 카메라 초기화 =====
    cap = cv2.VideoCapture(WEBCAM_PATH, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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

            # ===== 생성된 메시지를 각 큐에 넣기 =====
            # 기존의 sendto() 호출 대신, 각 큐에 데이터를 넣습니다.
            # put_nowait()은 큐가 가득 찼을 때 기다리지 않고 예외를 발생시켜,
            # 실시간 영상 전송에서 프레임이 밀리는 현상을 방지합니다.
            try:
                q_monitoring.put_nowait(message)
            except queue.Full:
                print("⚠️ Monitoring queue is full, dropping a frame.")
            
            try:
                q_following.put_nowait(message)
            except queue.Full:
                print("⚠️ AI_Following queue is full, dropping a frame.")

            frame_id += 1
            # 전송 속도(FPS)를 조절합니다. time.sleep(0.033)은 약 30FPS에 해당합니다.
            time.sleep(0.033)

    except KeyboardInterrupt:
        print("\n[종료] cam_sender stopped by user.")

    finally:
        print("Releasing camera and closing sockets...")
        cap.release()
        sock_monitoring.close()
        sock_following.close()

# 스크립트가 직접 실행될 때 main 함수를 호출하도록 하는 부분
if __name__ == '__main__':
    main()