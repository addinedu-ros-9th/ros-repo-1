# 필요한 라이브러리들을 가져옵니다.
import cv2
import torch
import time
import socket
import json
import numpy as np
from pathlib import Path
from strongsort.strong_sort import StrongSORT
import threading

# ===== 설정: 모델, 통신, 추적 파라미터 정의 =====
# --- 모델 설정 ---
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
YOLO_MODEL_NAME = 'yolov5m'
REID_WEIGHT_PATH = Path('./osnet_x1_0_msmt17.pt') # 실제 Re-ID 모델 가중치 경로로 수정하세요.

# --- 통신 설정 ---
# 1. 이미지 스트림을 수신할 포트 (cam_sender가 보내는 곳)
IMAGE_LISTEN_PORT = 7003
# 2. ROS2 노드로 추적 상태를 전송할 주소
ROS_BRIDGE_IP = '127.0.0.1'
ROS_BRIDGE_PORT = 7008
# 3. ROS2 노드로부터 명령을 수신할 포트
CMD_LISTEN_PORT = 7009

# --- 추적 파라미터 ---
REID_THRESHOLD = 0.4
FEATURE_UPDATE_ALPHA = 0.1

# ===== 모델 로드: YOLOv5와 StrongSORT 초기화 =====
print("🤖 모델을 로딩합니다...")
yolo_model = torch.hub.load('ultralytics/yolov5', YOLO_MODEL_NAME, pretrained=True).to(DEVICE)
tracker = StrongSORT(
    model_weights=REID_WEIGHT_PATH, device=DEVICE, fp16=False,
    max_age=200, max_dist=0.3, max_iou_distance=0.7, n_init=3
)
print("✅ 모델 로딩 완료.")

# ===== 소켓 설정: 3개의 통신 채널을 위한 소켓 생성 =====
# 1. 이미지 수신용 소켓
image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
image_sock.bind(('0.0.0.0', IMAGE_LISTEN_PORT))

# 2. 상태 전송용 소켓
status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 3. 명령 수신용 소켓
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(('0.0.0.0', CMD_LISTEN_PORT))

# ===== 상태 변수: 스레드 간 공유 데이터 =====
target_id = None
target_lost_time = None
target_mean_feature = None
state_lock = threading.Lock() # 스레드 동기화를 위한 Lock

def command_listener():
    """ROS2 노드로부터 오는 명령(타겟 지정/해제)을 수신하는 스레드 함수"""
    global target_id, target_mean_feature, target_lost_time
    print(f"👂 ROS2 명령 수신 대기 시작 (포트: {CMD_LISTEN_PORT})")
    while True:
        try:
            data, _ = cmd_sock.recvfrom(1024)
            command = json.loads(data.decode())
            
            with state_lock:
                if command.get('command') == 'set_target':
                    new_target_id = command.get('target_id')
                    if new_target_id is not None:
                        print(f"🎯 명령 수신: 타겟을 ID {new_target_id}(으)로 설정")
                        target_id, target_mean_feature, target_lost_time = new_target_id, None, None
                elif command.get('command') == 'clear_target':
                    print("🗑️ 명령 수신: 타겟 해제")
                    target_id, target_mean_feature, target_lost_time = None, None, None
        except Exception as e:
            print(f"❗ 명령 수신 중 오류 발생: {e}")

# 명령 수신 스레드 시작
threading.Thread(target=command_listener, daemon=True).start()

print(f"🚀 추적 시스템 시작. UDP 포트 {IMAGE_LISTEN_PORT}에서 이미지 수신 대기 중...")

try:
    while True:
        # ===== 1. 이미지 수신 및 디코딩 =====
        # UDP 패킷 수신 (버퍼 크기를 넉넉하게 설정)
        data, _ = image_sock.recvfrom(65536)

        # '헤더 | 이미지' 형식의 데이터를 분리
        separator_pos = data.find(b'|')
        if separator_pos == -1:
            print("⚠️ 수신된 패킷 형식이 잘못되었습니다. (구분자 '|' 없음)")
            continue
        
        # 헤더와 JPEG 이미지 바이트 분리
        # header_bytes = data[:separator_pos] # 헤더 정보는 현재 사용하지 않음
        jpeg_bytes = data[separator_pos+1:]

        # JPEG 바이트를 OpenCV가 사용할 수 있는 이미지(Numpy 배열)로 디코딩
        np_arr = np.frombuffer(jpeg_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            print("⚠️ JPEG 이미지 디코딩에 실패했습니다.")
            continue

        # ===== 2. 객체 탐지 및 추적 (기존 로직과 동일) =====
        results = yolo_model(frame)
        detections = results.xyxy[0]
        person_detections = detections[detections[:, 5] == 0]

        tracked_outputs = tracker.update(person_detections.cpu(), frame) if len(person_detections) > 0 else []
        if len(person_detections) == 0:
            tracker.increment_ages()

        current_tracks = {t.track_id: t for t in tracker.tracker.tracks}
        found_target = False
        lost_time = 0

        with state_lock:
            if target_id is not None and target_id in current_tracks:
                found_target = True
                target_lost_time = None
                current_feature = current_tracks[target_id].features[-1]
                if target_mean_feature is None:
                    target_mean_feature = current_feature
                else:
                    target_mean_feature = (1 - FEATURE_UPDATE_ALPHA) * target_mean_feature + FEATURE_UPDATE_ALPHA * current_feature
                    target_mean_feature /= np.linalg.norm(target_mean_feature)
            
            if not found_target and target_id is not None:
                if target_lost_time is None: target_lost_time = time.time()
                lost_time = time.time() - target_lost_time

                if target_mean_feature is not None and len(current_tracks) > 0:
                    best_match_id, min_dist = -1, float('inf')
                    for track_id, track in current_tracks.items():
                        if track.is_confirmed() and track.features:
                            dist = 1 - np.dot(track.features[-1], target_mean_feature)
                            if dist < min_dist: min_dist, best_match_id = dist, track_id
                    
                    if min_dist < REID_THRESHOLD:
                        print(f"🔄 타겟 재식별 성공! ID {target_id} -> {best_match_id} (거리: {min_dist:.3f})")
                        target_id, target_lost_time, lost_time = best_match_id, None, 0
        
        # ===== 3. 상태 전송 및 시각화 (기존 로직과 동일) =====
        message = {'timestamp': time.time(), 'lost_time': lost_time}
        status_sock.sendto(json.dumps(message).encode(), (ROS_BRIDGE_IP, ROS_BRIDGE_PORT))

        for output in tracked_outputs:
            x1, y1, x2, y2, track_id_out = map(int, output[:5])
            color, label = ((255, 0, 0), f"ID {track_id_out}")
            if track_id_out == target_id:
                color, label = ((0, 255, 0), f"TARGET: {track_id_out}")
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if lost_time > 0:
            cv2.putText(frame, f"TARGET LOST: {lost_time:.1f}s", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Object Tracker (UDP Input)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n[종료] 사용자에 의해 프로그램이 중단되었습니다.")
finally:
    print("소켓을 닫고 프로그램을 종료합니다...")
    image_sock.close()
    status_sock.close()
    cmd_sock.close()
    cv2.destroyAllWindows()