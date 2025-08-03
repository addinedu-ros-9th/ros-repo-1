import cv2
import torch
import time
import socket
import json
import numpy as np
from pathlib import Path
from collections import deque
from strongsort.strong_sort import StrongSORT

# === 설정 ===
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
YOLO_MODEL_NAME = 'yolov5m'
REID_WEIGHT_PATH = Path('/home/ptw/dev_ws/ro2_study/ros-repo-1/data/strongSort/osnet_x1_0_msmt17.pt')  # 수정: 문자열 -> Path 객체
DEST_IP = '127.0.0.1'
DEST_PORT = 7008
CAMERA_INDEX = 0
REID_THRESHOLD = 0.4

# === 모델 로드 ===
yolo_model = torch.hub.load('ultralytics/yolov5', YOLO_MODEL_NAME, pretrained=True).to(DEVICE)
tracker = StrongSORT(
    model_weights=REID_WEIGHT_PATH,
    device=DEVICE,
    fp16=False,
    max_age=200,
    max_dist=0.3,
    max_iou_distance=0.7,
    n_init=3
)

# === 통신 설정 ===
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# === 카메라 설정 ===
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError(f"Camera index {CAMERA_INDEX} not available")

# === 상태 변수 ===
target_id = None
target_lost_time = None
target_features = deque(maxlen=100)

print("[StrongSORT Client] Started. Sending tracking info via UDP...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = yolo_model(frame)
    detections = results.xyxy[0]
    person_detections = detections[detections[:, 5] == 0]

    if len(person_detections) > 0:
        tracked_outputs = tracker.update(person_detections.cpu(), frame)
    else:
        tracker.increment_ages()
        tracked_outputs = []

    # 타겟 추적 여부 판단 및 특징 저장
    found_target = False
    for output in tracked_outputs:
        track_id = int(output[4])
        if target_id is None:
            target_id = track_id
            found_target = True
            break
        elif target_id == track_id:
            found_target = True
            break

    # 특징 업데이트
    if found_target:
        for track in tracker.tracker.tracks:
            if track.track_id == target_id and track.features:
                target_features.append(track.features[-1])

    # 타겟 상태 갱신 및 감지 실패 시간 계산
    lost_time = 0
    if found_target:
        target_lost_time = None
    else:
        if target_lost_time is None:
            target_lost_time = time.time()
        else:
            lost_time = time.time() - target_lost_time

        # 재식별 시도
        if len(tracked_outputs) > 0 and len(target_features) > 0:
            best_match_id = -1
            min_dist = float('inf')
            for track in tracker.tracker.tracks:
                if track.is_confirmed() and track.features:
                    current_feature = track.features[-1]
                    distances = [1 - np.dot(current_feature, f) for f in target_features]
                    avg_dist = np.mean(distances)
                    if avg_dist < min_dist:
                        min_dist = avg_dist
                        best_match_id = track.track_id
            if min_dist < REID_THRESHOLD:
                print(f"🔄 타겟 재식별 성공! ID {target_id} -> {best_match_id} (거리: {min_dist:.3f})")
                target_id = best_match_id

    # 전송
    message = {
        'timestamp': time.time(),
        'lost_time': lost_time
    }
    sock.sendto(json.dumps(message).encode(), (DEST_IP, DEST_PORT))

    # 시각화
    for output in tracked_outputs:
        x1, y1, x2, y2, track_id = map(int, output[:5])
        if track_id == target_id:
            color = (0, 255, 0)
            label = f"TARGET: {track_id}"
        else:
            color = (255, 0, 0)
            label = f"ID {track_id}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    if lost_time > 0:
        cv2.putText(frame, f"LOST: {lost_time:.1f}s", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("StrongSORT Tracker", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
