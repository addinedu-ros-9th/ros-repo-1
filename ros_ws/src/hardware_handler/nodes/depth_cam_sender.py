import cv2
import socket
import json
import datetime
import time

# ======= 설정 =======
LIBO_SERVICE = '192.168.0.1'     # 실제 서버 IP로 변경
AI_SERVICE = '192.168.0.1'  # 실제 서버 IP로 변경
GUI_RECIVER = 7000
VISION_MANAGER = 7001

CAM_INDEX = 0              # 내장/USB 카메라 인덱스
JPEG_QUALITY = 80          # 영상 압축률 (0~100)

# ======= UDP 소켓 준비 =======
gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def now_iso8601():
    return datetime.datetime.utcnow().isoformat()

def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Camera open failed!")
        return

    frame_id = 1

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read fail")
            continue

        # Depth 카메라라면: frame이 1채널(Gray/16bit)이면 아래 수정
        # (여기서는 일반 카메라 예시)
        _, img_encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        img_bytes = img_encoded.tobytes()

        # JSON 헤더 생성
        header = {
            "frame_id": frame_id,
            "timestamp": now_iso8601()
        }
        header_bytes = json.dumps(header).encode('utf-8')

        # 패킷 구성: {json} + b'|' + JPEG + b'\n'
        packet = header_bytes + b'|' + img_bytes + b'\n'

        # GUI Reciver 전송
        gui_sock.sendto(packet, (LIBO_SERVICE, GUI_RECIVER))

        # Vision Manager 전송
        vision_sock.sendto(packet, (AI_SERVICE, VISION_MANAGER))

        frame_id += 1
        time.sleep(1/15)  # 15fps

if __name__ == '__main__':
    main()
