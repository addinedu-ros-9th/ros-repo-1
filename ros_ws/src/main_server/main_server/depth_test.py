import cv2
import numpy as np

DEVICE_PATH = '/dev/video2'  # 뎁스카메라 장치 경로

def setup_camera():
    """카메라 초기화 및 설정"""
    cap = cv2.VideoCapture(DEVICE_PATH)
    
    # 기본 설정
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG 포맷
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)     # 넓이
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)    # 높이
    cap.set(cv2.CAP_PROP_FPS, 30)             # 프레임레이트
    
    # 추가 설정 (필요한 경우)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)      # 밝기
    cap.set(cv2.CAP_PROP_CONTRAST, 32)       # 대비
    cap.set(cv2.CAP_PROP_SATURATION, 64)     # 채도
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 자동 노출
    
    return cap

def check_camera_settings(cap):
    """현재 카메라 설정 확인"""
    settings = {
        "FOURCC": cap.get(cv2.CAP_PROP_FOURCC),
        "FPS": cap.get(cv2.CAP_PROP_FPS),
        "WIDTH": cap.get(cv2.CAP_PROP_FRAME_WIDTH),
        "HEIGHT": cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
        "BRIGHTNESS": cap.get(cv2.CAP_PROP_BRIGHTNESS),
        "CONTRAST": cap.get(cv2.CAP_PROP_CONTRAST),
        "SATURATION": cap.get(cv2.CAP_PROP_SATURATION),
        "AUTO_EXPOSURE": cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
    }
    
    print("\n현재 카메라 설정:")
    for key, value in settings.items():
        print(f"{key}: {value}")
    print()

def main():
    # 카메라 초기화
    print(f"카메라 초기화 중... ({DEVICE_PATH})")
    cap = setup_camera()
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다!")
        return
    
    # 현재 설정 출력
    check_camera_settings(cap)
    print("카메라 스트리밍 시작...")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다!")
                break
            
            # 프레임 표시
            cv2.imshow('Depth Camera', frame)
            
            # 디버그 정보 (1초에 한 번만 출력)
            if int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % 30 == 0:
                print(f"Frame Info - Shape: {frame.shape}, Type: {frame.dtype}")
            
            # 'q' 키로 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("사용자가 종료를 요청했습니다.")
                break
                
    except Exception as e:
        print(f"에러 발생: {str(e)}")
        
    finally:
        print("정리 중...")
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
