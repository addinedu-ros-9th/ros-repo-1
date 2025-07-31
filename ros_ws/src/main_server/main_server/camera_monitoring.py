import cv2
import numpy as np
import socket
import json
import threading
from datetime import datetime
import pytz

# ======================== 상수 정의 =========================
BUFFER_SIZE = 65535                      # UDP 버퍼 크기
HARDWARE_HANDLER_IP = "127.0.0.1"        # 하드웨어 핸들러 IP (임시)
MONITORING_CAMERA_PORT = 7001            # 이미지 수신용 포트
WINDOW_NAME_DEPTH = "Depth Camera"       # 뎁스카메라 창 이름
WINDOW_NAME_WEBCAM = "Webcam"           # 웹캠 창 이름
# ===========================================================

def get_kr_time():
    """한국 시간 ISO 포맷으로 반환"""
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).isoformat()

class ImageReceiver:
    def __init__(self):
        # UDP 소켓 초기화
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HARDWARE_HANDLER_IP, MONITORING_CAMERA_PORT))
        
        # 각 카메라의 최신 프레임 저장용
        self.depth_frame = None
        self.webcam_frame = None
        self.depth_frame_id = 0
        self.webcam_frame_id = 0
        
        # OpenCV 윈도우 생성
        cv2.namedWindow(WINDOW_NAME_DEPTH)
        cv2.namedWindow(WINDOW_NAME_WEBCAM)
        print(f"[{get_kr_time()}][INIT] 이미지 수신기 초기화 완료 (포트: {MONITORING_CAMERA_PORT})")
        
    def receive_frame(self):
        """UDP 패킷 수신 및 디코딩"""
        try:
            # 전체 데이터 수신
            data, _ = self.sock.recvfrom(BUFFER_SIZE)
            
            # 헤더와 이미지 데이터 분리
            parts = data.split(b'\n')
            if len(parts) < 2:
                print(f"[{get_kr_time()}][ERROR] 잘못된 패킷 형식")
                return None
                
            # 헤더 파싱
            try:
                header = json.loads(parts[0].decode())
            except json.JSONDecodeError as e:
                print(f"[{get_kr_time()}][ERROR] 헤더 파싱 실패: {str(e)}")
                return None
            
            # 이미지 데이터 디코딩
            image_bytes = parts[1]
            frame = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            if frame is None:
                print(f"[{get_kr_time()}][ERROR] 프레임 디코딩 실패")
                return None
                
            return header["direction"], frame, header["frame_id"]
            
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] 프레임 수신 중 오류: {str(e)}")
            return None
    
    def run(self):
        """메인 루프"""
        try:
            print(f"[{get_kr_time()}][SYSTEM] 이미지 수신 시작")
            
            while True:
                result = self.receive_frame()
                if result is None:
                    continue
                    
                direction, frame, frame_id = result
                
                # 수신된 프레임을 적절한 변수에 저장
                if direction == "depth":
                    self.depth_frame = frame.copy()
                    self.depth_frame_id = frame_id
                else:  # webcam
                    self.webcam_frame = frame.copy()
                    self.webcam_frame_id = frame_id
                
                # 뎁스카메라 프레임이 있으면 표시
                if self.depth_frame is not None:
                    # FPS 및 프레임 정보 표시
                    depth_display = self.depth_frame.copy()
                    cv2.putText(depth_display, f"DEPTH - Frame #{self.depth_frame_id}", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow(WINDOW_NAME_DEPTH, depth_display)
                
                # 웹캠 프레임이 있으면 표시
                if self.webcam_frame is not None:
                    # FPS 및 프레임 정보 표시
                    webcam_display = self.webcam_frame.copy()
                    cv2.putText(webcam_display, f"WEBCAM - Frame #{self.webcam_frame_id}", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow(WINDOW_NAME_WEBCAM, webcam_display)
                
                # 'q' 키 입력 시 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print(f"[{get_kr_time()}][SYSTEM] 사용자에 의해 중지됨")
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] 예외 발생: {str(e)}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """리소스 정리"""
        print(f"[{get_kr_time()}][CLEANUP] 리소스 정리 중...")
        cv2.destroyAllWindows()
        self.sock.close()
        print(f"[{get_kr_time()}][SYSTEM] 프로그램 종료")

def main():
    receiver = ImageReceiver()
    receiver.run()

if __name__ == '__main__':
    main()
