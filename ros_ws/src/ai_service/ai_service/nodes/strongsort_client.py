# object_tracker.py

# 필요한 라이브러리들을 임포트합니다.
import cv2  # OpenCV: 이미지 및 비디오 처리를 위한 라이브러리
import torch  # PyTorch: 딥러닝 모델을 사용하기 위한 프레임워크
import time  # 시간 관련 함수를 사용하기 위한 라이브러리
import socket  # UDP 통신을 위한 소켓 라이브러리
import json  # JSON 형식의 데이터를 다루기 위한 라이브러리
import numpy as np  # 수치 계산, 특히 행렬 연산을 위한 라이브러리
from pathlib import Path  # 파일 경로를 객체 지향적으로 다루기 위한 라이브러리
from strongsort.strong_sort import StrongSORT  # 객체 추적 알고리즘 StrongSORT 임포트
import threading  # 병렬 처리를 위한 스레딩 라이브러리
import math  # 수학 계산을 위한 라이브러리

# ===== 설정 (Configuration) =====
# 이 스크립트의 동작을 제어하는 주요 변수들을 정의합니다.

# 딥러닝 연산을 수행할 장치를 설정합니다. NVIDIA GPU(cuda)가 있으면 사용하고, 없으면 CPU를 사용합니다.
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
# 사용할 YOLO(You Only Look Once) 객체 탐지 모델의 버전을 지정합니다. 'm'은 medium 모델을 의미합니다.
YOLO_MODEL_NAME = 'yolov5m'
# 재식별(Re-identification) 모델의 가중치 파일 경로입니다. StrongSORT가 객체의 외모 특징을 추출하는 데 사용합니다.
REID_WEIGHT_PATH = Path('./osnet_x1_0_msmt17.pt')
# 이미지(비디오 프레임)를 수신할 UDP 포트 번호입니다.
IMAGE_LISTEN_PORT = 7003
# 추적 상태 정보를 전송할 ROS 브릿지의 IP 주소입니다. (로컬 환경에서는 127.0.0.1)
ROS_BRIDGE_IP = '127.0.0.1'
# 추적 상태 정보를 전송할 ROS 브릿지의 포트 번호입니다.
ROS_BRIDGE_PORT = 7008
# 외부(ROS)로부터 제어 명령을 수신할 UDP 포트 번호입니다.
CMD_LISTEN_PORT = 7009
# 재식별 시, 두 객체의 외모 특징 벡터 간의 거리가 이 값보다 작아야 동일 객체로 판단합니다. 값이 작을수록 더 엄격합니다.
REID_THRESHOLD = 0.4
# 추적 중인 타겟의 외모 특징을 갱신할 때 사용하는 학습률입니다. 조명이나 자세 변화에 점진적으로 적응하게 해줍니다.
FEATURE_UPDATE_ALPHA = 0.1
last_status_send_time = 0  # 루프 밖에 정의

# ===== 모델 로드 (Model Loading) =====
print("🤖 모델을 로딩합니다...")
# YOLOv5 모델을 PyTorch Hub를 통해 로드하고, 지정된 장치(DEVICE)로 보냅니다.
yolo_model = torch.hub.load('ultralytics/yolov5', YOLO_MODEL_NAME, pretrained=True).to(DEVICE)
# StrongSORT 추적기를 초기화합니다.
tracker = StrongSORT(
    model_weights=REID_WEIGHT_PATH,  # 재식별 모델 가중치
    device=DEVICE,  # 연산 장치
    fp16=False,  # 반정밀도(16비트) 부동소수점 연산 사용 여부
    max_age=200,  # 객체가 화면에서 사라진 후 최대 몇 프레임까지 정보를 유지할지 결정 (타이머 문제의 핵심)
    max_dist=0.3,  # 외모 특징(Re-ID) 기반 매칭 시 최대 허용 거리
    max_iou_distance=0.7,  # IoU(Intersection over Union) 기반 매칭 시 최대 허용 거리
    n_init=3  # 새로운 트랙이 '확정(confirmed)' 상태가 되기 위해 필요한 최소 프레임 수
)
print("✅ 모델 로딩 완료.")

# ===== 소켓 설정 (Socket Setup) =====
# UDP 소켓을 생성합니다. UDP는 실시간 영상 스트리밍처럼 약간의 데이터 손실이 있어도 빠른 전송이 중요할 때 유용합니다.
image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 이미지 수신용 소켓
image_sock.bind(('0.0.0.0', IMAGE_LISTEN_PORT))  # 모든 IP 주소로부터의 이미지 수신을 허용
status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 상태 전송용 소켓
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 명령 수신용 소켓
cmd_sock.bind(('0.0.0.0', CMD_LISTEN_PORT))  # 모든 IP 주소로부터의 명령 수신을 허용

# ===== 상태 변수 (State Variables) =====
# 스크립트 전역에서 사용되는 상태 정보들을 저장하는 변수입니다.
target_id = None  # 현재 추적 중인 타겟의 고유 ID
target_lost_time = None  # 타겟을 처음 놓친 시점의 타임스탬프
target_mean_feature = None  # 추적 중인 타겟의 평균적인 외모 특징 벡터 (재식별을 위한 '기억')
find_center_target_flag = False  # '중앙 타겟 찾기' 명령이 수신되었는지 여부를 나타내는 플래그
state_lock = threading.Lock()  # 여러 스레드가 상태 변수들을 동시에 수정하는 것을 방지하는 잠금 장치 (매우 중요)


def command_listener():
    """ROS2 노드로부터 명령을 수신하는 별도의 스레드에서 실행될 함수"""
    global target_id, target_mean_feature, target_lost_time, find_center_target_flag
    print(f"👂 ROS2 명령 수신 대기 시작 (포트: {CMD_LISTEN_PORT})")
    while True:  # 무한 루프를 돌며 계속해서 명령을 기다립니다.
        try:
            # 소켓을 통해 최대 1024바이트의 데이터를 수신합니다.
            data, _ = cmd_sock.recvfrom(1024)
            # 수신된 데이터를 JSON 형식으로 파싱합니다.
            command = json.loads(data.decode())
            
            # state_lock을 사용하여 메인 스레드와의 충돌을 방지하며 상태 변수를 수정합니다.
            with state_lock:
                if command.get('command') == 'activate_and_find_center':
                    print("🎯 명령 수신: 중앙 타겟 찾기 활성화")
                    # 기존 타겟 정보를 모두 초기화하고, 타겟 찾기 플래그를 True로 설정합니다.
                    target_id, target_mean_feature, target_lost_time = None, None, None
                    find_center_target_flag = True 
                elif command.get('command') == 'clear_target':
                    print("🗑️ 명령 수신: 타겟 해제")
                    # 모든 타겟 정보를 초기화하여 추적을 중지합니다.
                    target_id, target_mean_feature, target_lost_time = None, None, None
                    find_center_target_flag = False
        except Exception as e:
            print(f"❗ 명령 수신 중 오류 발생: {e}")

# command_listener 함수를 데몬 스레드로 생성하고 시작합니다.
# 데몬 스레드는 메인 프로그램이 종료될 때 함께 종료됩니다.
threading.Thread(target=command_listener, daemon=True).start()

print(f"🚀 추적 시스템 시작. UDP 포트 {IMAGE_LISTEN_PORT}에서 이미지 수신 대기 중...")

try:
    # 메인 루프: 프로그램의 핵심 로직을 무한 반복합니다.
    while True:
        # 1. 이미지 수신 및 디코딩
        data, _ = image_sock.recvfrom(65536)  # UDP 버퍼 크기를 크게 잡아 이미지 데이터를 수신합니다.
        separator_pos = data.find(b'|')  # 커스텀 프로토콜의 구분자를 찾습니다.
        if separator_pos == -1: continue  # 구분자가 없으면 데이터를 무시합니다.
        jpeg_bytes = data[separator_pos+1:]  # 구분자 이후의 순수 JPEG 데이터를 추출합니다.
        np_arr = np.frombuffer(jpeg_bytes, np.uint8)  # JPEG 바이트를 NumPy 배열로 변환합니다.
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # NumPy 배열을 OpenCV 이미지 프레임으로 디코딩합니다.
        if frame is None: continue  # 디코딩 실패 시 다음 루프로 넘어갑니다.

        # 2. 객체 탐지 및 추적
        results = yolo_model(frame)  # YOLO 모델로 현재 프레임의 모든 객체를 탐지합니다.
        detections = results.xyxy[0]  # 탐지 결과를 [x1, y1, x2, y2, conf, class] 형식으로 가져옵니다.
        person_detections = detections[detections[:, 5] == 0]  # 클래스 ID가 0인 '사람'만 필터링합니다.
        # 필터링된 사람 정보를 StrongSORT 추적기에 전달하여 ID가 포함된 추적 결과를 받습니다.
        tracked_outputs = tracker.update(person_detections.cpu(), frame) if len(person_detections) > 0 else []
        if len(person_detections) == 0: tracker.increment_ages()  # 탐지된 사람이 없으면 모든 트랙의 '나이'를 증가시킵니다.

        # 3. 상태 관리 및 추적 로직
        current_tracks = {t.track_id: t for t in tracker.tracker.tracks}  # 추적기가 '기억'하는 모든 트랙 정보
        current_frame_track_ids = {int(output[4]) for output in tracked_outputs}  # 현재 프레임에 '보이는' 트랙들의 ID 집합
        lost_time = 0.0  # 현재 프레임에서의 분실 시간을 0으로 초기화합니다.

        with state_lock:  # 스레드 안전 구역 시작
            # 3.1. 중앙 타겟 찾기 명령 처리
            if find_center_target_flag and len(tracked_outputs) > 0:
                frame_center_x, frame_center_y = frame.shape[1] // 2, frame.shape[0] // 2
                min_dist, center_target_id = float('inf'), None
                # 현재 보이는 모든 객체에 대해 화면 중앙과의 거리를 계산합니다.
                for output in tracked_outputs:
                    x1, y1, x2, y2, track_id_out = map(int, output[:5])
                    box_center_x, box_center_y = (x1 + x2) / 2, (y1 + y2) / 2
                    dist = math.sqrt((box_center_x - frame_center_x)**2 + (box_center_y - frame_center_y)**2)
                    if dist < min_dist:
                        min_dist, center_target_id = dist, track_id_out
                
                # 가장 가까운 객체를 새로운 타겟으로 지정합니다.
                if center_target_id is not None:
                    target_id = center_target_id
                    target_lost_time = None  # 새 타겟이므로 타이머 초기화
                    target_mean_feature = None  # 새 타겟이므로 외모 특징 초기화
                    print(f"✅ 중앙 타겟 결정: ID {target_id}")
                
                find_center_target_flag = False  # 명령을 처리했으므로 플래그를 다시 False로 설정

            # 3.2. 타겟 추적 상태 머신 (State Machine)
            is_target_defined = target_id is not None  # 타겟이 지정되어 있는가?
            is_target_in_frame = is_target_defined and target_id in current_frame_track_ids  # 타겟이 현재 프레임에 보이는가?

            if is_target_in_frame:
                # [상태: 발견] - 타겟이 현재 프레임에 보이는 경우
                target_lost_time = None  # 분실 타이머를 리셋합니다.
                if target_id in current_tracks:
                    current_feature = current_tracks[target_id].features[-1]
                    # 타겟의 외모 특징을 부드럽게 갱신하여 변화에 적응합니다.
                    if target_mean_feature is None:
                        target_mean_feature = current_feature
                    else:
                        target_mean_feature = (1 - FEATURE_UPDATE_ALPHA) * target_mean_feature + FEATURE_UPDATE_ALPHA * current_feature
                        target_mean_feature /= np.linalg.norm(target_mean_feature)
            
            elif is_target_defined and not is_target_in_frame:
                # [상태: 분실] - 타겟이 지정은 되어있지만 현재 프레임에 보이지 않는 경우
                if target_lost_time is None:
                    target_lost_time = time.time()  # '방금' 사라졌다면, 현재 시간을 분실 시작 시점으로 기록합니다.

                # [수정된 재인식 로직]
                re_id_succeeded = False
                # 현재 '보이는' 다른 사람이 있고, 기억하는 타겟의 외모 특징이 있을 때만 재인식을 시도합니다.
                if target_mean_feature is not None and len(current_frame_track_ids) > 0:
                    best_match_id, min_dist = -1, float('inf')
                    # '유령' 트랙이 아닌 '현재 보이는' 트랙들 중에서만 재인식 후보를 찾습니다.
                    for visible_id in current_frame_track_ids:
                        if visible_id in current_tracks: # 트랙 정보가 메모리에 있는지 확인
                            track = current_tracks[visible_id]
                            # 확정된 트랙이고 외모 특징이 있을 경우
                            if track.is_confirmed() and track.features:
                                dist = 1 - np.dot(track.features[-1], target_mean_feature)
                                if dist < min_dist:
                                    min_dist, best_match_id = dist, visible_id
                    
                    # 가장 닮은 사람을 찾았고, 그 유사도가 임계값(THRESHOLD)보다 높으면
                    if min_dist < REID_THRESHOLD:
                        print(f"🔄 타겟 재식별 성공! ID {target_id} -> {best_match_id} (거리: {min_dist:.3f})")
                        target_id = best_match_id
                        target_lost_time = None  # 재식별에 성공했으므로 분실 타이머를 리셋합니다.
                        re_id_succeeded = True
            
            # 3.3. 최종 lost_time 계산
            # target_lost_time에 값이 있다면 (즉, 분실 상태가 지속되고 있다면)
            if target_lost_time is not None:
                 lost_time = time.time() - target_lost_time

        # 4. 상태 전송 및 시각화
        current_time = time.time()
        if current_time - last_status_send_time > 1.0:  # 1초마다 전송
        # current_time = time.time()
            message = {'timestamp': time.time(), 'lost_time': lost_time}
            status_sock.sendto(json.dumps(message).encode(), (ROS_BRIDGE_IP, ROS_BRIDGE_PORT))
            last_status_send_time = current_time

        # 현재 프레임에 보이는 모든 객체에 대해 바운딩 박스를 그립니다.
        for output in tracked_outputs:
            x1, y1, x2, y2, track_id_out = map(int, output[:5])
            color = (255, 0, 0)  # 기본 색상 (파란색)
            label = f"ID {track_id_out}"
            # 만약 현재 객체가 우리의 타겟이라면 색상과 라벨을 변경합니다.
            if track_id_out == target_id:
                color = (0, 255, 0)  # 타겟 색상 (녹색)
                label = f"TARGET: {track_id_out}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 분실 시간이 0보다 크면 화면에 "TARGET LOST" 메시지를 표시합니다.
        if lost_time > 0:
            cv2.putText(frame, f"TARGET LOST: {lost_time:.1f}s", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Object Tracker (UDP Input)", frame)
        # 'q' 키를 누르면 루프를 탈출하여 프로그램을 종료합니다.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    # Ctrl+C를 눌러 프로그램을 종료할 때 메시지를 출력합니다.
    print("\n[종료] 사용자에 의해 프로그램이 중단되었습니다.")
finally:
    # 프로그램 종료 시 항상 실행되는 코드로, 자원을 정리합니다.
    print("소켓을 닫고 프로그램을 종료합니다...")
    image_sock.close()
    status_sock.close()
    cmd_sock.close()
    cv2.destroyAllWindows()