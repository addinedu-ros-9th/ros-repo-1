# 필요한 라이브러리들을 가져옵니다.
import cv2  # OpenCV: 카메라 제어 및 이미지 처리를 위해 사용
import socket  # 소켓: 네트워크 통신(UDP)을 위해 사용
import time  # 시간 관련 함수: 전송 간격 조절(딜레이)을 위해 사용
import json  # JSON: 데이터 구조를 만들기 위해 사용 (헤더 생성)
from datetime import datetime, timezone # 날짜 및 시간: 타임스탬프 생성을 위해 사용

def main(args=None):
    # ===== 설정: 전송할 대상 및 카메라 옵션을 정의하는 부분 =====
    # 모니터링 서비스(수신자 1)의 IP 주소. "127.0.0.1"은 자기 자신(localhost)을 의미합니다.
    UDP_IP_MONITORING = "127.0.0.1"
    # 모니터링 서비스가 수신 대기하고 있는 포트 번호입니다.
    UDP_PORT_MONITORING = 7001

    # AI 서비스(수신자 2)의 IP 주소입니다.
    UDP_IP_FOLLOWING = "127.0.0.1"
    # UDP_IP_FOLLOWING = "192.168.1.7"
    # AI 서비스가 수신 대기하고 있는 포트 번호입니다.
    UDP_PORT_FOLLOWING = 7003

    # 모니터링 서비스로 영상을 전송할지 여부를 결정하는 플래그(True: 전송, False: 전송 안 함)
    SEND_TO_MONITORING = True
    # AI 서비스로 영상을 전송할지 여부를 결정하는 플래그
    SEND_TO_FOLLOWING = True

    # 사용할 카메라의 번호. 0은 보통 내장 웹캠을 의미합니다.
    CAMERA_INDEX = 0

    # ===== 소켓 설정: UDP 통신을 위한 소켓을 생성하는 부분 =====
    # 모니터링 서비스로 데이터를 보내기 위한 UDP 소켓 생성
    sock_monitoring = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # AI 서비스로 데이터를 보내기 위한 UDP 소켓 생성
    sock_following = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ===== 카메라 초기화: OpenCV를 사용하여 카메라를 여는 부분 =====
    # 지정된 번호(CAMERA_INDEX)의 카메라를 엽니다.
    cap = cv2.VideoCapture(CAMERA_INDEX)
    # 카메라가 성공적으로 열렸는지 확인하고, 실패 시 에러를 발생시켜 프로그램을 중단합니다.
    if not cap.isOpened():
        raise RuntimeError(f"Camera index {CAMERA_INDEX} not found.")

    # 전송하는 각 프레임에 고유 번호를 붙이기 위한 카운터 변수
    frame_id = 0

    try:
        # 프로그램이 중단되기 전까지 계속해서 프레임을 캡처하고 전송하는 메인 루프
        while True:
            # 카메라에서 현재 프레임을 한 장 읽어옵니다.
            # ret은 성공 여부(True/False), frame은 실제 이미지 데이터(Numpy 배열)입니다.
            ret, frame = cap.read()
            # 프레임 읽기에 실패하면 경고 메시지를 출력하고 다음 루프로 넘어갑니다.
            if not ret:
                print("⚠️  Failed to capture frame.")
                continue

            # 네트워크 부하를 줄이기 위해 이미지 크기를 640x480으로 조절합니다.
            frame = cv2.resize(frame, (640, 480))

            # 이미지를 JPEG 형식으로 압축합니다.
            # 압축 품질을 40으로 설정 (0~100, 값이 낮을수록 압축률이 높아지고 화질은 낮아짐)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
            # 설정된 품질로 프레임을 JPEG으로 인코딩합니다.
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
            # 인코딩에 실패하면 경고 메시지를 출력하고 다음 루프로 넘어갑니다.
            if not ret:
                print("⚠️  JPEG encoding failed.")
                continue

            # 인코딩된 JPEG 데이터를 전송 가능한 바이트(bytes) 형태로 변환합니다.
            jpeg_bytes = jpeg.tobytes()

            # 이미지와 함께 보낼 메타데이터(헤더)를 JSON 형식으로 생성합니다.
            header = {
                "direction": "rear",  # 카메라 방향
                "frame_id": frame_id, # 프레임 고유 번호
                # 현재 시간을 UTC 기준 ISO 8601 형식의 문자열로 생성 (표준 시간 형식)
                "timestamp": datetime.now(timezone.utc).isoformat()
            }
            # 파이썬 딕셔너리(header)를 JSON 문자열로 변환합니다.
            header_str = json.dumps(header)

            # 최종 전송 메시지를 만듭니다. '헤더 | 이미지 데이터 \n' 구조를 가집니다.
            # 헤더 문자열을 바이트로 변환하고, 구분자(|), 이미지 바이트, 종료 문자(\n)를 차례로 붙입니다.
            message = header_str.encode() + b'|' + jpeg_bytes + b'\n'

            # 설정된 플래그에 따라 각 목적지로 메시지를 전송합니다.
            if SEND_TO_MONITORING:
                sock_monitoring.sendto(message, (UDP_IP_MONITORING, UDP_PORT_MONITORING))
            if SEND_TO_FOLLOWING:
                sock_following.sendto(message, (UDP_IP_FOLLOWING, UDP_PORT_FOLLOWING))

            # 다음 프레임을 위해 ID를 1 증가시킵니다.
            frame_id += 1
            # 0.1초간 대기하여 전송률을 약 10fps로 조절합니다. (CPU 부하 감소 효과)
            time.sleep(0.1)

    # 사용자가 Ctrl+C를 눌러 프로그램을 중단하려고 할 때 발생하는 예외 처리
    except KeyboardInterrupt:
        print("\n[종료] cam_sender stopped by user.")

    # try 블록이 정상적으로 끝나거나, 예외가 발생해도 항상 실행되는 코드 블록
    finally:
        print("Releasing camera and closing sockets...")
        # 사용하던 카메라 리소스를 해제합니다.
        cap.release()
        # 사용하던 소켓 리소스를 모두 닫습니다.
        sock_monitoring.close()
        sock_following.close()


# 스크립트가 직접 실행될 때 main 함수를 호출하도록 하는 부분
if __name__ == '__main__':
    main()