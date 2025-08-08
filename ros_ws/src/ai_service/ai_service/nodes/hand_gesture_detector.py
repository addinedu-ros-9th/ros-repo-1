#!/usr/bin/env python3
# Hand Gesture Detector - UDP 기반 카메라 입력 수신 버전
import cv2
import mediapipe as mp
import numpy as np
import socket
import threading
import time
import json

# 상수 정의
FRONT_CAM_GESTURE = 7022   # 프론트 카메라 포트 (수신)
HAND_GESTURE_BRIDGE = 7023  # 손 제스처 브릿지 포트 (송신)
LOCAL_IP = '0.0.0.0'        # 모든 인터페이스에서 수신
TARGET_IP = '127.0.0.1'     # 결과 전송 대상 IP
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
BUFFER_SIZE = 65536         # UDP 패킷 최대 크기

class HandGestureDetector:
    """
    손 제스처를 감지하여 UDP로 전송하는 클래스
    
    제스처 종류:
    - go: 팔목보다 손가락이 높은 상태로 "일로와~" 모양
    - back: 팔목보다 손가락이 낮은 상태로 "저리가~" 모양
    - left: 엄지손가락을 왼쪽으로
    - right: 엄지손가락을 오른쪽으로
    - stop: 주먹을 쥐는 동작
    - none: 인식된 제스처가 없음
    """
    def __init__(self):
        # 로그 출력
        print('손 제스처 감지 모듈 초기화 중...')
        
        # MediaPipe 손 솔루션 초기화
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  # 한 손만 감지
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # 디버깅을 위한 화면 표시 설정
        self.show_debug_window = True  # 디버그 화면 표시 여부
        self.debug_window_name = "Hand Gesture Debug"  # 창 이름
        
        # OpenCV 창 설정 (창 크기 조절 가능하도록)
        if self.show_debug_window:
            try:
                cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.debug_window_name, 640, 480)  # 윈도우 크기 줄임
            except Exception as e:
                print(f"OpenCV 창 생성 실패: {e}")
                self.show_debug_window = False
        
        # 프레임 수신 관련 변수
        self.current_frame = None
        self.frame_ready = False
        self.frame_lock = threading.Lock()  # 프레임 접근을 위한 뮤텍스
        
        # UDP 소켓 설정 (제스처 결과 송신용)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # UDP는 연결 상태를 유지할 필요 없음
        
        # 제스처 안정화를 위한 변수
        self.current_gesture = "none"
        self.gesture_history = []
        self.history_max_size = 5  # 히스토리 최대 크기
        self.min_gesture_count = 3  # 특정 제스처로 판단하기 위한 최소 발생 횟수
        self.same_gesture_count = 0  # 같은 제스처가 연속된 횟수
        self.last_sent_time = 0     # 마지막 전송 시간
        self.send_interval = 0.5    # 전송 간격 (초)
        
        # 로봇 ID
        self.robot_id = "libo_a"  # 기본 로봇 ID
        
        # 처리 관련 변수
        self.is_running = True
        
        print('손 제스처 감지 모듈 초기화 완료!')
        
    def start(self):
        """UDP 소켓 및 수신 설정"""
        print(f"UDP 프레임 수신 및 제스처 감지 시작...")
        
        # 수신용 UDP 소켓 초기화
        try:
            # 프레임 수신용 소켓
            self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 버퍼 크기 증가 (대용량 패킷 처리) - 더 크게 설정
            recv_buffer_size = 4 * 1024 * 1024  # 4MB 버퍼 (기존 1MB -> 4MB로 확장)
            self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buffer_size)
            
            # 소켓 타임아웃 설정 (더 짧게)
            self.recv_socket.settimeout(0.1)  # 0.1초 타임아웃으로 변경 (더 자주 확인)
            
            try:
                # 바인딩 전에 기존 소켓 연결 해제 시도 (오류 무시)
                self.recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except:
                pass  # SO_REUSEPORT가 지원되지 않는 플랫폼에서는 무시
            
            # 포트 바인딩
            self.recv_socket.bind((LOCAL_IP, FRONT_CAM_GESTURE))  # 모든 인터페이스에서 수신
            print(f"✅ 프레임 수신 UDP 소켓 초기화 성공 (포트: {FRONT_CAM_GESTURE}, 버퍼: {recv_buffer_size/1024:.0f}KB)")
            
            # 제스처 전송용 소켓
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print(f"✅ 제스처 전송 UDP 소켓 초기화 성공 (포트: {HAND_GESTURE_BRIDGE})")
            
            # cam_sender_front.py의 형식 설명 로그
            print(f"✅ cam_sender_front.py 형식 메시지 예상: header_str.encode() + b'|' + jpeg_bytes + b'\\n'")
            print(f"✅ 예상 방향(direction): 'rear'")
            
            # 프레임 수신 스레드 시작
            self.receiver_thread = threading.Thread(target=self.receive_frames, daemon=True)
            self.receiver_thread.start()
            print("✅ 프레임 수신 스레드 시작")
            
            return True
            
        except Exception as e:
            print(f"⚠️ UDP 소켓 초기화 실패: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def process_frames_loop(self):
        """메인 처리 루프"""
        frames_processed = 0
        last_fps_report = time.time()
        no_frame_count = 0  # 프레임이 없는 상태 카운터
        
        while self.is_running:
            try:
                # 수신한 프레임 가져오기
                frame = None
                with self.frame_lock:
                    if self.frame_ready:
                        frame = self.current_frame.copy()
                
                # 프레임이 없으면 대기하고 로그 출력
                if frame is None:
                    no_frame_count += 1
                    # 100회마다 로그 출력 (대략 1초마다)
                    if no_frame_count % 100 == 0:
                        print(f"[정보] UDP 프레임 대기 중... ({no_frame_count/100:.0f}초)")
                    time.sleep(0.01)  # CPU 사용량 줄이기 위해 짧게 대기
                    continue
                else:
                    # 프레임을 받았으면 카운터 리셋
                    if no_frame_count > 0:
                        print(f"[정보] 프레임 수신 재개됨!")
                    no_frame_count = 0
                
                # 프레임 좌우 반전 (거울 모드)
                frame = cv2.flip(frame, 1)
                
                # 프레임 크기가 너무 크면 리사이징
                if frame.shape[0] > IMAGE_HEIGHT or frame.shape[1] > IMAGE_WIDTH:
                    frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
                
                # BGR을 RGB로 변환 (MediaPipe 요구사항)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # MediaPipe 손 감지 실행
                results = self.hands.process(frame_rgb)
                
                # 손 제스처 처리
                self.process_hand_gesture(frame, results)
                
                # FPS 계산
                frames_processed += 1
                current_time = time.time()
                if current_time - last_fps_report >= 5.0:  # 5초마다 FPS 보고
                    fps = frames_processed / (current_time - last_fps_report)
                    print(f'[정보] 현재 처리 속도: {fps:.1f} FPS')
                    frames_processed = 0
                    last_fps_report = current_time
                
                # OpenCV 이벤트 처리 (waitKey가 중요한 역할 - OpenCV 창 업데이트)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' 또는 ESC 키로 종료
                    self.is_running = False
                    break
                    
            except Exception as e:
                print(f"[오류] 프레임 처리 루프 오류: {str(e)}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)  # 오류 발생시 잠시 대기
    
    def process_hand_gesture(self, frame, results):
        """손 제스처 감지 및 처리"""
        gesture = "none"  # 기본값
        debug_info = {}  # 디버그 정보 저장
        
        # 디버깅용 화면에 표시할 프레임 복사
        debug_frame = None
        if self.show_debug_window:
            debug_frame = frame.copy()
        
        if results.multi_hand_landmarks:
            # 첫 번째 감지된 손에 대한 제스처 판단
            hand_landmarks = results.multi_hand_landmarks[0]
            
            # 랜드마크 그리기 (디버깅용)
            if debug_frame is not None:
                # 손 랜드마크 더 명확하게 그리기
                self.mp_drawing.draw_landmarks(
                    debug_frame, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=4),
                    self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                )
                
                h, w, _ = debug_frame.shape
                
                # 주요 관절 강조 표시
                landmarks_to_highlight = {
                    "Wrist": self.mp_hands.HandLandmark.WRIST,
                    "Thumb_TIP": self.mp_hands.HandLandmark.THUMB_TIP,
                    "Index_TIP": self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
                    "Middle_TIP": self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                    "Ring_TIP": self.mp_hands.HandLandmark.RING_FINGER_TIP,
                    "Pinky_TIP": self.mp_hands.HandLandmark.PINKY_TIP
                }
                
                # 손목 위치 강조 (더 크게)
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                wrist_x, wrist_y = int(wrist.x * w), int(wrist.y * h)
                cv2.circle(debug_frame, (wrist_x, wrist_y), 10, (0, 0, 255), -1)
                cv2.putText(debug_frame, "Wrist", (wrist_x-30, wrist_y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # BACK 제스처 인식을 위한 손목 기준선 표시
                wrist_threshold_y = int((wrist.y + 0.1) * h)  # 손목보다 10% 아래
                cv2.line(debug_frame, (0, wrist_threshold_y), (w, wrist_threshold_y), 
                        (0, 128, 255), 1, cv2.LINE_AA)
                cv2.putText(debug_frame, "BACK threshold", (10, wrist_threshold_y-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 1)
                
                # 손가락 끝 강조 표시 (각 손가락마다 다른 색상)
                colors = {
                    "Thumb_TIP": (255, 0, 0),   # 빨강 (엄지)
                    "Index_TIP": (0, 255, 0),   # 녹색 (검지)
                    "Middle_TIP": (0, 0, 255),  # 파랑 (중지)
                    "Ring_TIP": (255, 255, 0),  # 노랑 (약지)
                    "Pinky_TIP": (255, 0, 255)  # 핑크 (새끼)
                }
                
                # 손바닥 중앙 계산 및 시각화
                palm_center_x = (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].x +
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].x) / 2
                palm_center_y = (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y +
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].y) / 2
                
                palm_x, palm_y = int(palm_center_x * w), int(palm_center_y * h)
                cv2.circle(debug_frame, (palm_x, palm_y), 15, (200, 100, 0), 2)  # 손바닥 중앙
                cv2.putText(debug_frame, "Palm", (palm_x+5, palm_y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 100, 0), 1)
                
                # STOP 인식을 위한 중앙 영역 표시 (타원)
                threshold_radius_x = int(0.08 * w)
                threshold_radius_y = int(0.1 * h)
                cv2.ellipse(debug_frame, (palm_x, palm_y), (threshold_radius_x, threshold_radius_y), 
                           0, 0, 360, (0, 0, 255), 1)
                
                # 손가락 끝 관절 강조
                for name, landmark_idx in landmarks_to_highlight.items():
                    if name == "Wrist":
                        continue  # 손목은 이미 처리됨
                    
                    landmark = hand_landmarks.landmark[landmark_idx]
                    x, y = int(landmark.x * w), int(landmark.y * h)
                    color = colors.get(name, (200, 200, 200))
                    
                    # 더 큰 원과 텍스트로 표시
                    cv2.circle(debug_frame, (x, y), 8, color, -1)
                    if name == "Thumb_TIP":  # 엄지는 제스처 구분에 중요하므로 더 강조
                        cv2.circle(debug_frame, (x, y), 12, color, 2)
                        # 엄지가 손바닥 중앙 영역 안에 있는지 표시
                        in_center = (abs(landmark.x - palm_center_x) < 0.08 and
                                     abs(landmark.y - palm_center_y) < 0.1)
                        status = "CENTER" if in_center else "OUTSIDE"
                        cv2.putText(debug_frame, status, (x+15, y), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255) if in_center else (200, 200, 200), 1)
            
            # 제스처 분류 및 디버그 정보 수집
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
            index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
            middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
            pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
            
            # 디버그 정보 수집
            thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            index_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
            middle_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
            ring_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP]
            pinky_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP]
            
            # 손가락 펴짐 상태 판별
            index_extended = index_tip.y < index_pip.y - 0.03
            middle_extended = middle_tip.y < middle_pip.y - 0.03
            ring_extended = ring_tip.y < ring_pip.y - 0.03
            pinky_extended = pinky_tip.y < pinky_pip.y - 0.03
            
            # 손가락 펴짐 상태 저장
            debug_info["wrist_y"] = wrist.y
            debug_info["index_extended"] = index_extended
            debug_info["middle_extended"] = middle_extended
            debug_info["ring_extended"] = ring_extended
            debug_info["pinky_extended"] = pinky_extended
            debug_info["fingers_extended_count"] = sum([index_extended, middle_extended, ring_extended, pinky_extended])
            debug_info["delta_index"] = index_tip.y - wrist.y
            debug_info["delta_middle"] = middle_tip.y - wrist.y
            debug_info["delta_ring"] = ring_tip.y - wrist.y
            debug_info["delta_pinky"] = pinky_tip.y - wrist.y
            
            # 제스처 분류
            gesture = self.classify_gesture(hand_landmarks)
            
            # 좌/우 또는 GO/BACK 제스처가 감지되면 더 자세한 디버그 정보 출력
            if gesture in ["left", "right", "go", "back"] or self.current_gesture in ["left", "right", "go", "back"]:
                # 손가락 펼침 상태 표시 (T: 펴짐, F: 접힘)
                fingers_status = f"손가락 펼침: 검지={index_extended}, 중지={middle_extended}, 약지={ring_extended}, 소지={pinky_extended}"
                fingers_count = sum([index_extended, middle_extended, ring_extended, pinky_extended])
                
                print(f"\n===== 제스처 디버그 정보 =====")
                print(f"손가락 상태: {fingers_count}개 펴짐 ({fingers_status})")
                print(f"손가락 끝점 위치 정보:")
                print(f"  - 손목 Y: {wrist.y:.3f}")
                print(f"  - 검지 Y: {index_tip.y:.3f} (손목과 차이: {(index_tip.y - wrist.y):.3f})")
                print(f"  - 중지 Y: {middle_tip.y:.3f} (손목과 차이: {(middle_tip.y - wrist.y):.3f})")
                print(f"  - 약지 Y: {ring_tip.y:.3f} (손목과 차이: {(ring_tip.y - wrist.y):.3f})")
                print(f"  - 소지 Y: {pinky_tip.y:.3f} (손목과 차이: {(pinky_tip.y - wrist.y):.3f})")
                print(f"제스처 판정: {gesture.upper()} (이전: {self.current_gesture.upper()})")
                
                # 손가락 위치 판정 결과
                fingers_below_count = 0
                if index_tip.y > wrist.y + 0.02: fingers_below_count += 1
                if middle_tip.y > wrist.y + 0.02: fingers_below_count += 1
                if ring_tip.y > wrist.y + 0.02: fingers_below_count += 1
                if pinky_tip.y > wrist.y + 0.02: fingers_below_count += 1
                
                # GO/BACK 제스처 조건 설명
                if fingers_count >= 2:
                    print(f"손가락이 펴짐 ({fingers_count}/4) → GO/BACK 제스처 판별 가능")
                    print(f"  손목보다 아래에 있는 손가락: {fingers_below_count}개")
                    
                    # 손가락이 손목보다 아래인지 판정 결과
                    if fingers_below_count >= 2:
                        print(f"  손가락이 손목보다 아래에 있음 → BACK 제스처 조건 충족")
                    else:
                        print(f"  손가락이 손목보다 위에 있음 → GO 제스처 조건 충족")
                        
                    if gesture in ["go", "back"]:
                        print(f"  → {gesture.upper()} 제스처로 판정")
                else:
                    print(f"손가락이 대부분 접힘 ({fingers_count}/4) → LEFT/RIGHT 제스처 판별 가능")
                    if gesture in ["left", "right"]:
                        print(f"  → {gesture.upper()} 제스처로 판정")
                
                print(f"===========================")
            
            # 디버그 화면에 손가락 위치 정보 표시
            if self.show_debug_window and debug_frame is not None:
                # 손가락 펴짐 상태 표시
                extended_count = sum([index_extended, middle_extended, ring_extended, pinky_extended])
                finger_status = f"Fingers: I{'↑' if index_extended else '↓'} M{'↑' if middle_extended else '↓'} R{'↑' if ring_extended else '↓'} P{'↑' if pinky_extended else '↓'} ({extended_count}/4)"
                cv2.putText(debug_frame, finger_status, (10, h-100), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 손가락 끝과 손목의 Y위치 차이 표시 (양수면 손목보다 아래)
                index_diff = index_tip.y - wrist.y
                middle_diff = middle_tip.y - wrist.y
                ring_diff = ring_tip.y - wrist.y
                pinky_diff = pinky_tip.y - wrist.y
                
                # 손가락별 위치 상태 (위/아래) 표시
                below_count = 0
                if index_diff > 0.02: below_count += 1
                if middle_diff > 0.02: below_count += 1
                if ring_diff > 0.02: below_count += 1
                if pinky_diff > 0.02: below_count += 1
                
                # 각 손가락의 위치 (위/아래) 표시
                y_pos = f"Above/Below: I{'↓' if index_diff > 0.02 else '↑'} M{'↓' if middle_diff > 0.02 else '↑'} R{'↓' if ring_diff > 0.02 else '↑'} P{'↓' if pinky_diff > 0.02 else '↑'} ({below_count}/4)"
                cv2.putText(debug_frame, y_pos, (10, h-80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 손가락 끝과 손목의 Y위치 차이 표시
                y_info = f"Y-Wrist: {index_diff:.2f}, {middle_diff:.2f}, {ring_diff:.2f}, {pinky_diff:.2f}"
                cv2.putText(debug_frame, y_info, (10, h-60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 현재 제스처 표시
                if gesture in ["left", "right", "go", "back"]:
                    cv2.putText(debug_frame, f"{gesture.upper()} GESTURE DETECTED", (10, h-40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # 제스처 판별 기준 표시
                    if extended_count >= 2:
                        back_check = "BACK" if below_count >= 2 else "GO"
                        criteria = f"Criteria: Fingers Extended ({extended_count}/4), Position: {back_check}"
                    else:
                        criteria = "Criteria: Fist with thumb (LEFT/RIGHT)"
                    cv2.putText(debug_frame, criteria, (10, h-20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        
        # 제스처 히스토리에 추가 (더 긴 히스토리 유지)
        self.gesture_history.append(gesture)
        if len(self.gesture_history) > self.history_max_size:
            self.gesture_history.pop(0)  # 가장 오래된 제스처 제거
        
        # 안정화된 제스처 가져오기
        stable_gesture = self.get_stable_gesture()
        
        # 제스처 카운트 및 전송 로직
        now = time.time()
        if stable_gesture == self.current_gesture:
            self.same_gesture_count += 1
        else:
            self.same_gesture_count = 1
            self.current_gesture = stable_gesture
            self.last_sent_time = 0  # 새 제스처 인식시 즉시 발행
        
        # 제스처 전송 조건: 3프레임 연속 같은 제스처 또는 0.5초마다 재전송
        if self.same_gesture_count == self.min_gesture_count or \
           (self.same_gesture_count > self.min_gesture_count and now - self.last_sent_time > self.send_interval):
            # 제스처 전송
            self.send_gesture_data(self.current_gesture)
            self.last_sent_time = now
            
        # 디버깅 화면 표시
        self.display_debug_info(debug_frame, self.current_gesture)
    
    def display_debug_info(self, debug_frame, gesture):
        """디버깅 정보를 화면에 표시"""
        if not self.show_debug_window or debug_frame is None:
            return
            
        try:
            # 화면 상단에 정보 표시를 위한 영역 생성
            info_panel = np.zeros((80, debug_frame.shape[1], 3), dtype=np.uint8)
            
            # 제스처별 색상 및 표시
            if gesture == "stop":
                gesture_color = (0, 0, 255)  # 빨간색 (정지)
            elif gesture == "go":
                gesture_color = (0, 255, 0)  # 녹색 (전진)
            elif gesture == "back":
                gesture_color = (0, 128, 255)  # 주황색 (후진)
            elif gesture == "left" or gesture == "right":
                gesture_color = (255, 0, 0)  # 파란색 (좌/우 방향)
            else:
                gesture_color = (200, 200, 200)  # 회색 (제스처 없음)
            
            # 제스처 이름 표시 - 더 크고 두껍게, 더 큰 글씨
            cv2.putText(info_panel, f"Gesture: {gesture.upper()}", 
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.2, gesture_color, 3)
            
            # 제스처별 시각적 아이콘 표시 (크게 개선)
            icon_pos = (debug_frame.shape[1] - 80, 45)
            icon_size = 35  # 아이콘 크기 증가
            
            if gesture == "go":
                # 위쪽 화살표 (더 크고 굵게)
                cv2.arrowedLine(info_panel, 
                              (icon_pos[0], icon_pos[1]+icon_size), 
                              (icon_pos[0], icon_pos[1]-icon_size), 
                              (0, 255, 0), 5, tipLength=0.4)
                # 손바닥 아이콘 추가
                cv2.circle(info_panel, (icon_pos[0]-icon_size, icon_pos[1]), 10, (0, 255, 0), -1)
            
            elif gesture == "back":
                # 아래쪽 화살표 (더 크고 굵게)
                cv2.arrowedLine(info_panel, 
                              (icon_pos[0], icon_pos[1]-icon_size), 
                              (icon_pos[0], icon_pos[1]+icon_size), 
                              (0, 128, 255), 5, tipLength=0.4)
                # 손바닥 아이콘 추가
                cv2.circle(info_panel, (icon_pos[0]-icon_size, icon_pos[1]), 10, (0, 128, 255), -1)
                
            elif gesture == "left":
                # 왼쪽 화살표 (더 크고 굵게)
                cv2.arrowedLine(info_panel, 
                              (icon_pos[0]+icon_size, icon_pos[1]), 
                              (icon_pos[0]-icon_size, icon_pos[1]), 
                              (255, 0, 0), 5, tipLength=0.4)
                # 엄지 방향 표시
                thumb_start = (icon_pos[0], icon_pos[1]+15)
                thumb_end = (icon_pos[0]-20, icon_pos[1]-15)
                cv2.line(info_panel, thumb_start, thumb_end, (255, 0, 0), 4)
                
            elif gesture == "right":
                # 오른쪽 화살표 (더 크고 굵게)
                cv2.arrowedLine(info_panel, 
                              (icon_pos[0]-icon_size, icon_pos[1]), 
                              (icon_pos[0]+icon_size, icon_pos[1]), 
                              (255, 0, 0), 5, tipLength=0.4)
                # 엄지 방향 표시
                thumb_start = (icon_pos[0], icon_pos[1]+15)
                thumb_end = (icon_pos[0]+20, icon_pos[1]-15)
                cv2.line(info_panel, thumb_start, thumb_end, (255, 0, 0), 4)
                
            elif gesture == "stop":
                # 정지 표시 (팔각형 + STOP 텍스트)
                # 빨간색 팔각형 (더 명확한 정지 표시)
                points = []
                for i in range(8):
                    angle = 2 * np.pi * i / 8
                    x = int(icon_pos[0] + 25 * np.cos(angle))
                    y = int(icon_pos[1] + 25 * np.sin(angle))
                    points.append([x, y])
                
                pts = np.array(points, np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.fillPoly(info_panel, [pts], (0, 0, 255))
                
                # STOP 텍스트 (흰색)
                cv2.putText(info_panel, "STOP", 
                          (icon_pos[0]-25, icon_pos[1]+5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            # 안정성 정보 표시 - 더 명확하게
            stability = len([g for g in self.gesture_history if g == gesture]) / max(1, len(self.gesture_history))
            stability_text = f"Stability: {stability*100:.0f}%"
            
            # 안정성 게이지 막대 추가
            bar_width = 150
            bar_height = 15
            bar_x = 10
            bar_y = 80
            
            # 바 배경 (회색)
            cv2.rectangle(info_panel, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (100, 100, 100), -1)
            
            # 안정성 레벨 (파란색~녹색)
            filled_width = int(bar_width * stability)
            bar_color = (0, 255 * stability, 255 * (1 - stability))  # 불안정(파란색)에서 안정(녹색)으로
            cv2.rectangle(info_panel, (bar_x, bar_y), (bar_x + filled_width, bar_y + bar_height), bar_color, -1)
            
            # 안정성 텍스트
            cv2.putText(info_panel, stability_text, 
                        (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 정보 패널과 디버그 프레임 결합
            display_frame = np.vstack((info_panel, debug_frame))
            
            # 화면 표시
            cv2.imshow(self.debug_window_name, display_frame)
        except Exception as e:
            print(f"[오류] 화면 표시 실패: {str(e)}")
            try:
                # OpenCV 윈도우 재생성 시도
                cv2.destroyWindow(self.debug_window_name)
                cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
            except:
                pass
    
    def classify_gesture(self, hand_landmarks):
        """
        손 랜드마크를 기반으로 제스처 분류
        
        Args:
            hand_landmarks: MediaPipe의 손 랜드마크
            
        Returns:
            str: 감지된 제스처 ("go", "back", "left", "right", "stop", "none" 중 하나)
        """
        # 필요한 랜드마크 추출
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]  # 손목
        thumb_cmc = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC]  # 엄지 시작점 (손바닥)
        thumb_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP]  # 엄지 첫 관절
        thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]    # 엄지 중간 관절
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]  # 엄지 끝
        
        # 검지~새끼손가락 끝과 각 관절점
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        
        index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]  # 손바닥 시작점
        middle_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        ring_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP]
        pinky_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]
        
        index_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]  # 중간 관절
        middle_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
        ring_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP]
        pinky_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP]
        
        # 손바닥 중앙 계산 (보다 정확하게)
        palm_center_x = (index_mcp.x + middle_mcp.x + ring_mcp.x + pinky_mcp.x) / 4
        palm_center_y = (index_mcp.y + middle_mcp.y + ring_mcp.y + pinky_mcp.y) / 4
        palm_center_z = (index_mcp.z + middle_mcp.z + ring_mcp.z + pinky_mcp.z) / 4
        
        # ===== 손가락 펴짐 여부 계산 (개선된 방식) =====
        
        # 엄지 특별 처리 (왼손/오른손 모두 지원)
        # 방법 1: 엄지 끝과 검지 MCP 사이의 거리
        thumb_index_distance = self.distance_3d(thumb_tip, index_mcp)
        # 방법 2: 엄지 끝과 엄지 시작점 사이의 각도
        thumb_angle = self.calculate_angle(thumb_cmc, thumb_mcp, thumb_tip)
        
        # 엄지가 펴져있는지 결정 (두 방법 결합)
        thumb_extended = (thumb_index_distance > 0.08 and thumb_angle > 30)
        
        # 엄지 방향 판단 - LEFT/RIGHT 구분에 중요
        # 엄지가 손바닥 중앙보다 확실히 왼쪽/오른쪽에 있는지 확인
        thumb_pointing_left = thumb_tip.x < palm_center_x - 0.1
        thumb_pointing_right = thumb_tip.x > palm_center_x + 0.1
        
        # 검지~새끼손가락은 y축 방향으로 비교 (끝이 중간 관절보다 위에 있으면 펴진 것)
        # 더 엄격한 조건으로 설정 (-0.03 는 임계값으로 손가락이 확실히 펴진 상태를 의미)
        index_extended = index_tip.y < index_pip.y - 0.03
        middle_extended = middle_tip.y < middle_pip.y - 0.03
        ring_extended = ring_tip.y < ring_pip.y - 0.03
        pinky_extended = pinky_tip.y < pinky_pip.y - 0.03
        
        # 손가락 펴짐 상태 (엄지 제외)
        other_fingers_extended = [index_extended, middle_extended, ring_extended, pinky_extended]
        
        # 손바닥이 위/아래 방향을 향하는지 확인 - 개선된 방법
        # 손가락 끝이 손목보다 확실히 아래에 있는지 확인 (BACK 제스처 개선)
        # y값이 클수록 화면 아래쪽
        # 손가락 끝점들의 y좌표 평균 위치가 손목보다 아래인지 확인
        # 더 정확한 방식으로 구현
        finger_tips_avg_y = (index_tip.y + middle_tip.y + ring_tip.y + pinky_tip.y) / 4
        
        # 각 손가락 끝이 손목보다 아래에 있는지 확인 (임계값 완화)
        # 임계값을 0.05에서 0.02로 줄여서 손가락 위치가 손목과 비슷해도 아래로 간주
        num_fingers_below_wrist = 0
        index_below = index_tip.y > wrist.y + 0.02
        middle_below = middle_tip.y > wrist.y + 0.02
        ring_below = ring_tip.y > wrist.y + 0.02
        pinky_below = pinky_tip.y > wrist.y + 0.02
        
        if index_below: num_fingers_below_wrist += 1
        if middle_below: num_fingers_below_wrist += 1
        if ring_below: num_fingers_below_wrist += 1
        if pinky_below: num_fingers_below_wrist += 1
        
        # 손가락 끝이 손목보다 아래에 있는지를 판단 (BACK 제스처 필요조건)
        # 손가락 끝의 평균 위치가 손목보다 아래이거나
        # 최소 2개 이상의 손가락이 손목보다 아래에 있으면 BACK으로 간주
        fingers_below_wrist = (
            num_fingers_below_wrist >= 2 or  # 2개 이상의 손가락이 아래
            finger_tips_avg_y > wrist.y  # 손가락 끝의 평균 위치가 손목보다 아래
        )
        
        # 손바닥이 위쪽을 향하는지 여부
        palm_facing_up = not fingers_below_wrist  # 손가락이 손목보다 아래에 있지 않으면 손바닥이 위쪽
        
        # ===== LEFT/RIGHT 제스처 (최우선 확인) =====
        # LEFT/RIGHT는 주먹을 쥔 상태에서 엄지만 분명하게 좌우로 향하는 상태
        # 다른 손가락들이 모두 접혀 있어야 함 (주먹 쥔 상태)
        other_fingers_all_folded = sum(other_fingers_extended) == 0  # 모든 손가락이 접혀있어야 함
        
        # 주먹을 쥐고 엄지만 뻗은 상태에서만 좌/우 인식
        if other_fingers_all_folded and thumb_extended:
            if thumb_pointing_left:
                return "left"
            elif thumb_pointing_right:
                return "right"
        
        # ===== GO/BACK 제스처 =====
        # 손바닥을 활짝 편 상태에서만 GO/BACK 제스처 판단
        # 최소 2개 이상의 손가락이 펴져 있으면 판별 가능하도록 조건 완화
        fingers_extended_count = sum(other_fingers_extended)
        fingers_fully_extended = fingers_extended_count >= 2
        
        # 손바닥을 펴고 있는 경우 (2개 이상의 손가락이 펴진 상태)
        if fingers_fully_extended:
            # 손가락이 손목보다 위에 있으면 GO, 아래에 있으면 BACK
            if palm_facing_up:
                return "go"  # 손바닥이 위로 향함 (손가락이 손목보다 위에 있음)
            else:
                return "back"  # 손바닥이 아래로 향함 (손가락이 손목보다 아래에 있음)
        
        # ===== STOP 제스처 =====
        # 모든 손가락이 접힌 상태 (주먹 쥔 모양) 또는 엄지가 손바닥 중앙 근처에 있을 때
        thumb_in_center = (
            abs(thumb_tip.x - palm_center_x) < 0.08 and  # 엄지 끝이 손바닥 중앙 x좌표 근처
            abs(thumb_tip.y - palm_center_y) < 0.1   # 엄지 끝이 손바닥 중앙 y좌표 근처
        )
        
        # 관절들의 위치 확인 (중간 관절들이 손목보다 아래에 있는지)
        # 이 조건이 추가되면 "BACK" 제스처가 아닌 경우를 더 정확히 "STOP"으로 분류할 수 있음
        joints_below_wrist = (
            index_pip.y > wrist.y + 0.02 and 
            middle_pip.y > wrist.y + 0.02 and 
            ring_pip.y > wrist.y + 0.02 and 
            pinky_pip.y > wrist.y + 0.02
        )
        
        # STOP 제스처 조건 개선
        # 1. 주먹을 쥐었거나 엄지가 손바닥 중앙에 위치하는 경우
        # 2. 또는 대부분의 관절이 손목보다 아래에 있고, 손가락은 펴지지 않은 경우 (주먹 쥔 상태와 유사)
        if ((not any(other_fingers_extended) and (not thumb_extended or thumb_in_center)) or
            (joints_below_wrist and sum(other_fingers_extended) <= 1)):
            return "stop"
        
        # 위의 조건에 맞지 않으면 인식된 제스처 없음
        return "none"
    
    def distance_3d(self, point1, point2):
        """두 3D 포인트 사이의 유클리드 거리 계산"""
        return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)**0.5
        
    def calculate_angle(self, point1, point2, point3):
        """
        세 점 사이의 각도 계산 (degree)
        point2가 각도의 정점(vertex)
        """
        # 두 벡터 계산
        vector1 = [point1.x - point2.x, point1.y - point2.y, point1.z - point2.z]
        vector2 = [point3.x - point2.x, point3.y - point2.y, point3.z - point2.z]
        
        # 벡터의 크기
        length1 = sum(x*x for x in vector1) ** 0.5
        length2 = sum(x*x for x in vector2) ** 0.5
        
        # 영벡터 체크
        if length1 == 0 or length2 == 0:
            return 0
            
        # 내적 계산
        dot_product = sum(v1 * v2 for v1, v2 in zip(vector1, vector2))
        
        # 각도 계산 (라디안)
        angle_rad = np.arccos(max(-1.0, min(1.0, dot_product / (length1 * length2))))
        
        # 라디안에서 각도로 변환
        angle_deg = np.degrees(angle_rad)
        
        return angle_deg
    
    def get_stable_gesture(self):
        """
        히스토리에서 가장 빈번하게 나타난 제스처를 반환
        최소 min_gesture_count 이상 나타나야 함
        """
        if not self.gesture_history:
            return "none"
            
        # 각 제스처 카운트
        gesture_counts = {}
        for gesture in self.gesture_history:
            if gesture in gesture_counts:
                gesture_counts[gesture] += 1
            else:
                gesture_counts[gesture] = 1
                
        # 가장 많이 발생한 제스처
        most_common_gesture = max(gesture_counts.items(), key=lambda x: x[1])
        
        # 최소 카운트 검사
        if most_common_gesture[1] >= self.min_gesture_count:
            return most_common_gesture[0]
        else:
            return "none"
    
    def receive_frames(self):
        """UDP로 카메라 프레임을 수신하는 메서드 (별도 스레드에서 실행)"""
        print("프레임 수신 스레드 시작됨")
        frames_received = 0
        last_report_time = time.time()
        
        while self.is_running:
            try:
                # UDP 패킷 수신
                data, addr = self.recv_socket.recvfrom(BUFFER_SIZE)
                
                # 데이터 형식: header_str.encode() + b'|' + jpeg_bytes + b'\n'
                # (cam_sender_front.py와 일치하도록 조정)
                try:
                    # 데이터 분리 (헤더|JPEG 데이터\n 형식)
                    parts = data.split(b'|', 1)
                    if len(parts) < 2:
                        print(f"[경고] 잘못된 데이터 형식: 구분자 '|' 없음, 데이터 길이: {len(data)}")
                        continue
                    
                    header_data, jpeg_data = parts
                    # 마지막 '\n' 제거
                    if jpeg_data.endswith(b'\n'):
                        jpeg_data = jpeg_data[:-1]
                    
                    # 헤더 파싱 (JSON)
                    header = json.loads(header_data.decode('utf-8'))
                    frame_id = header.get('frame_id', 0)
                    direction = header.get('direction', 'unknown')
                    timestamp = header.get('timestamp', '')
                    
                    # 디버그 정보 주기적으로 출력 (5초마다)
                    frames_received += 1
                    current_time = time.time()
                    if current_time - last_report_time >= 5.0:
                        fps = frames_received / (current_time - last_report_time)
                        print(f"[정보] 프레임 수신 중: {fps:.1f} FPS, 방향: {direction}, ID: {frame_id}")
                        frames_received = 0
                        last_report_time = current_time
                    
                    # JPEG 데이터를 이미지로 디코딩
                    nparr = np.frombuffer(jpeg_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # 프레임 저장 (스레드 안전하게)
                        with self.frame_lock:
                            self.current_frame = frame
                            self.frame_ready = True
                    else:
                        print(f"[경고] 프레임 디코딩 실패: 빈 프레임 (JPEG 데이터 길이: {len(jpeg_data)})")
                except json.JSONDecodeError as je:
                    print(f"[오류] 헤더 JSON 파싱 실패: {str(je)}")
                    print(f"헤더 데이터 일부: {header_data[:50]}...")
                    continue
                except Exception as e:
                    print(f"[오류] 프레임 처리 실패: {str(e)}")
                    import traceback
                    traceback.print_exc()  # 스택 트레이스 출력
                    continue
                
            except socket.timeout:
                # 타임아웃은 정상적인 상황 (주기적으로 스레드 종료 조건 확인을 위함)
                pass
            except Exception as e:
                print(f"[오류] 프레임 수신 중 오류 발생: {str(e)}")
                time.sleep(0.1)  # 오류 시 잠시 대기
    
    def send_gesture_data(self, gesture):
        """감지된 제스처를 UDP로 전송"""
        from datetime import datetime, timezone
        
        # 전송할 데이터 생성 (JSON 형식)
        # cam_sender_front.py 형식과 일관성 유지
        gesture_data = {
            "robot_id": self.robot_id,
            "gesture": gesture,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "source": "hand_gesture_detector3"
        }
        
        # JSON 직렬화
        json_data = json.dumps(gesture_data).encode('utf-8')
        
        try:
            # cam_sender_front.py 형식으로 맞춤: header + b'|' + data + b'\n'
            # 헤더만 있고 데이터는 없으므로 빈 바이트 추가
            message = json_data + b'|' + b'' + b'\n'
            
            # UDP로 전송 (TARGET_IP의 수신 포트로)
            self.udp_socket.sendto(message, (TARGET_IP, HAND_GESTURE_BRIDGE))
            
            if gesture != "none":  # none은 로그가 너무 많아 출력하지 않음
                print(f'[정보] UDP로 제스처 전송: {gesture}')
        except Exception as e:
            print(f'[오류] UDP 제스처 전송 실패: {str(e)}')
    
    def stop(self):
        """모든 리소스 정리 및 종료"""
        print('종료 프로세스 시작...')
        self.is_running = False
        
        # 먼저 OpenCV 창부터 닫기 (UI 반응 안함 방지)
        if self.show_debug_window:
            try:
                cv2.destroyAllWindows()
                # 모든 창이 안전하게 닫힐 수 있도록 잠시 대기
                for _ in range(5):
                    cv2.waitKey(1)
                print('디버그 창 닫힘')
            except Exception as e:
                print(f'창 닫기 실패: {e}')
        
        # 수신 UDP 소켓 닫기
        if hasattr(self, 'recv_socket'):
            try:
                # 타임아웃을 0으로 설정하여 즉시 종료되도록
                self.recv_socket.settimeout(0)
                self.recv_socket.close()
                print('프레임 수신 UDP 소켓 닫힘')
            except Exception as e:
                print(f'수신 소켓 닫기 실패: {e}')
        
        # 송신 UDP 소켓 닫기
        if hasattr(self, 'udp_socket'):
            try:
                self.udp_socket.close()
                print('제스처 전송 UDP 소켓 닫힘')
            except Exception as e:
                print(f'송신 소켓 닫기 실패: {e}')
        
        # 스레드 종료 대기 (소켓을 먼저 닫아야 스레드가 빨리 종료됨)
        if hasattr(self, 'receiver_thread') and self.receiver_thread.is_alive():
            try:
                self.receiver_thread.join(timeout=1.0)  # 최대 1초만 대기 (소켓이 닫혀 있으므로 빠르게 종료되어야 함)
                print('프레임 수신 스레드 종료됨')
            except Exception as e:
                print(f'스레드 종료 중 오류: {e}')
            
        print('손 제스처 감지 모듈 종료 완료')

def main():
    detector = None
    
    try:
        # MediaPipe 초기화 중 오류 발생 가능성 있음
        detector = HandGestureDetector()
        
        if not detector.start():
            print("UDP 소켓을 초기화할 수 없어 프로그램을 종료합니다.")
            return
            
        print("프로그램이 실행 중입니다. 'q' 키 또는 ESC 키를 눌러 종료하세요.")
        print("수신 중인 포트: FRONT_CAM_GESTURE =", FRONT_CAM_GESTURE)
        print("전송 중인 포트: HAND_GESTURE_BRIDGE =", HAND_GESTURE_BRIDGE)
        
        # 메인 처리 루프 시작
        detector.process_frames_loop()
                
    except KeyboardInterrupt:
        print('\n키보드 인터럽트 감지. 종료 중...')
    except Exception as e:
        print(f"[오류] 예기치 않은 오류 발생: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # 항상 리소스를 정리
        if detector is not None:
            try:
                detector.stop()
            except Exception as e:
                print(f"[오류] 프로그램 종료 중 오류 발생: {str(e)}")
                import traceback
                traceback.print_exc()
        
        # 모든 OpenCV 창 강제 종료
        try:
            cv2.destroyAllWindows()
            for _ in range(10):  # 확실하게 종료하기 위해 여러 번 시도
                cv2.waitKey(1)
        except:
            pass

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"[심각한 오류] 메인 함수에서 처리되지 않은 예외: {str(e)}")
        import traceback
        traceback.print_exc()
        
        # 열린 창 모두 닫기 시도
        try:
            cv2.destroyAllWindows()
            for _ in range(10):
                cv2.waitKey(1)
        except:
            pass
