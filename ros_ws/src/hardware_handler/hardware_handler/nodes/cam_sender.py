import cv2
import numpy as np
import socket
import json
import time
from datetime import datetime
import pytz
import threading

# ======================== 상수 정의 =========================
LIBO_SERVICE_IP = "127.0.0.1"            # Libo Service IP (임시)
MONITORING_CAMERA_PORT = 7001            # 이미지 전송용 포트

DEPTH_CAM_INDEX = 0                      # 뎁스 카메라 인덱스
WEBCAM_INDEX = 2                         # 웹캠 카메라 인덱스

JPEG_QUALITY = 90                        # JPEG 품질 (0-100)
FPS_TARGET = 30                          # 목표 FPS
# ===========================================================

def get_kr_time():
    """한국 시간 ISO 포맷으로 반환"""
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).isoformat()

def create_frame_header(direction, frame_id):
    """프레임 헤더 생성"""
    return {
        "direction": direction,
        "frame_id": frame_id,
        "timestamp": get_kr_time()
    }

class CameraSender:
    def __init__(self, camera_index, direction):
        self.cap = cv2.VideoCapture(camera_index)
        self.direction = direction
        self.frame_id = 0
        
        if not self.cap.isOpened():
            raise RuntimeError(f"{direction} 카메라를 열 수 없습니다. (index: {camera_index})")
            
        print(f"[{get_kr_time()}][INIT] {direction} 카메라 초기화 완료")

    def get_frame(self):
        """프레임 캡처 및 인코딩"""
        ret, frame = self.cap.read()
        if not ret:
            return None
            
        self.frame_id += 1
        
        try:
            # JPEG 인코딩
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            
            # 헤더 생성
            header = create_frame_header(self.direction, self.frame_id)
            header_bytes = json.dumps(header).encode()
            
            # 패킷 구성: 헤더와 이미지 데이터를 \n으로 구분
            return header_bytes + b'\n' + jpeg.tobytes()
            
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] 프레임 인코딩 실패: {str(e)}")
            return None

    def release(self):
        self.cap.release()

def main():
    print(f"[{get_kr_time()}][INIT] cam_sender 초기화 중...")
    
    # UDP 소켓 초기화
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # 카메라 초기화
        depth_cam = CameraSender(DEPTH_CAM_INDEX, "depth")
        webcam = CameraSender(WEBCAM_INDEX, "webcam")
        print(f"[{get_kr_time()}][INIT] UDP 전송 시작 (목표 FPS: {FPS_TARGET})")
        
        frame_interval = 1.0 / FPS_TARGET
        last_time = time.time()
        frames_sent = 0
        
        while True:
            current_time = time.time()
            
            # FPS 제어
            if current_time - last_time >= frame_interval:
                # 뎁스카메라 프레임 전송
                depth_frame = depth_cam.get_frame()
                if depth_frame:
                    sock.sendto(depth_frame, (LIBO_SERVICE_IP, MONITORING_CAMERA_PORT))
                    frames_sent += 1
                
                # 웹캠 프레임 전송
                webcam_frame = webcam.get_frame()
                if webcam_frame:
                    sock.sendto(webcam_frame, (LIBO_SERVICE_IP, MONITORING_CAMERA_PORT))
                    frames_sent += 1
                
                # 매 초마다 상태 출력
                if int(current_time) > int(last_time):
                    actual_fps = frames_sent / (current_time - last_time)
                    print(f"[{get_kr_time()}][STATUS] 전송 FPS: {actual_fps:.1f}")
                    frames_sent = 0
                    last_time = current_time
            
            # CPU 사용률 조절
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print(f"[{get_kr_time()}][SYSTEM] 사용자에 의해 중지됨")
    except Exception as e:
        print(f"[{get_kr_time()}][ERROR] 예외 발생: {str(e)}")
    finally:
        print(f"[{get_kr_time()}][CLEANUP] 리소스 정리 중...")
        depth_cam.release()
        webcam.release()
        sock.close()
        print(f"[{get_kr_time()}][SYSTEM] 프로그램 종료")

if __name__ == '__main__':
    main()
