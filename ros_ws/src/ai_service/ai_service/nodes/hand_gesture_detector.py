#!/usr/bin/env python3
import cv2
import mediapipe as mp
import numpy as np
import socket
import struct
import threading
import time
import json

# 상수 정의
FRONT_CAM_GESTURE = 7022  # 프론트 카메라 포트 (수신)
HAND_GESTURE_BRIDGE = 7023  # 손 제스처 브릿지 포트 (송신)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FPS = 10

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
        
        # UDP 소켓 설정 (카메라 스트림 수신용)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(('0.0.0.0', FRONT_CAM_GESTURE))
        
        # UDP 소켓 설정 (제스처 결과 송신용)
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 버퍼 설정
        self.buffer_size = IMAGE_WIDTH * IMAGE_HEIGHT * 3 + 1024  # 여유 있게 버퍼 크기 설정
        
        # 현재 감지된 제스처 및 신뢰도
        self.current_gesture = "none"
        self.gesture_confidence = 0.0
        self.robot_id = "libo_a"  # 기본 로봇 ID
        
        # 제스처 안정화를 위한 변수
        self.gesture_history = []
        self.history_max_size = 5  # 히스토리 최대 크기
        self.min_gesture_count = 3  # 특정 제스처로 판단하기 위한 최소 발생 횟수
        
        # 처리 관련 변수
        self.is_running = True
        self.frame_data = None
        self.frame_lock = threading.Lock()
        
        # 스레드 시작
        self.receive_thread = threading.Thread(target=self.receive_frames)
        self.process_thread = threading.Thread(target=self.process_frames_loop)
        
        self.receive_thread.daemon = True
        self.process_thread.daemon = True
        
        print('손 제스처 감지 모듈 초기화 완료!')
        
    def start(self):
        """스레드 시작"""
        self.receive_thread.start()
        self.process_thread.start()
        print(f'카메라 스트림 수신 시작 (포트: {FRONT_CAM_GESTURE})')
        print(f'제스처 데이터 전송 준비 완료 (포트: {HAND_GESTURE_BRIDGE})')
        
    def receive_frames(self):
        """UDP를 통한 이미지 프레임 수신 스레드"""
        while self.is_running:
            try:
                data, addr = self.recv_sock.recvfrom(self.buffer_size)
                
                # 헤더 파싱 (예: 프레임 크기, 타임스탬프 등)
                # 실제 구현은 카메라 스트림 형식에 따라 달라질 수 있음
                if len(data) < 12:  # 최소 헤더 크기
                    continue
                
                # 간단한 예로, 첫 8바이트를 타임스탬프로, 다음 4바이트를 프레임 크기로 가정
                # timestamp = struct.unpack('!Q', data[0:8])[0]
                frame_size = struct.unpack('!I', data[8:12])[0]
                
                # 이미지 데이터 추출 및 디코딩
                jpg_data = data[12:]
                
                # 프레임 크기 확인
                if len(jpg_data) != frame_size:
                    self.get_logger().warning(f'잘못된 프레임 크기: 예상 {frame_size}, 실제 {len(jpg_data)}')
                    continue
                
                # JPEG 데이터를 numpy 배열로 변환
                nparr = np.frombuffer(jpg_data, np.uint8)
                
                # 이미지 디코딩
                with self.frame_lock:
                    self.frame_data = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
            except Exception as e:
                print(f'[오류] 프레임 수신 오류: {str(e)}')
                time.sleep(0.1)  # 오류 발생 시 잠시 대기
                
    def process_frames_loop(self):
        """프레임 처리 루프"""
        last_process_time = time.time()
        target_interval = 1.0 / FPS  # 초당 FPS 프레임 처리
        
        while self.is_running:
            current_time = time.time()
            elapsed = current_time - last_process_time
            
            # 일정 간격으로 프레임 처리
            if elapsed >= target_interval:
                self.process_frame()
                last_process_time = current_time
            else:
                # CPU 사용량 감소를 위한 짧은 대기
                time.sleep(0.001)
    
    def process_frame(self):
        """프레임 처리 및 제스처 감지 함수"""
        with self.frame_lock:
            frame = self.frame_data
        
        if frame is None:
            return
            
        # BGR을 RGB로 변환 (MediaPipe 요구사항)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # MediaPipe 손 감지 실행
        results = self.hands.process(frame_rgb)
        
        # 감지 결과 처리
        gesture = "none"  # 기본값
        
        if results.multi_hand_landmarks:
            # 첫 번째 감지된 손에 대한 제스처 판단
            hand_landmarks = results.multi_hand_landmarks[0]
            
            # 랜드마크 그리기 (디버깅용)
            # self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            
            # 제스처 분류
            gesture = self.classify_gesture(hand_landmarks)
        
        # 제스처 히스토리에 추가
        self.gesture_history.append(gesture)
        if len(self.gesture_history) > self.history_max_size:
            self.gesture_history.pop(0)  # 가장 오래된 제스처 제거
            
        # 가장 많이 나타난 제스처 결정
        stable_gesture = self.get_stable_gesture()
        
        # 현재 제스처가 변경되었을 때만 UDP 전송
        if stable_gesture != self.current_gesture:
            self.current_gesture = stable_gesture
            self.send_gesture_data(stable_gesture)
            
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
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]  # 엄지 끝
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]  # 검지 끝
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]  # 중지 끝
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]  # 약지 끝
        pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]  # 새끼손가락 끝
        
        # 손가락 끝 점들의 y좌표 평균
        finger_tips_y_avg = (index_tip.y + middle_tip.y + ring_tip.y + pinky_tip.y) / 4
        
        # 손가락이 펴져 있는지 확인 (엄지 제외)
        fingers_extended = []
        
        # 검지 ~ 새끼손가락 펴짐 여부
        for i in range(8, 21, 4):  # 검지, 중지, 약지, 새끼손가락 끝 인덱스
            finger_tip = hand_landmarks.landmark[i]
            finger_pip = hand_landmarks.landmark[i - 2]  # PIP 관절 (중간 마디)
            
            # 손끝이 PIP보다 위에 있으면 손가락이 펴진 것
            fingers_extended.append(finger_tip.y < finger_pip.y)
        
        # 엄지는 다르게 계산 (x 좌표 기준)
        thumb_extended = thumb_tip.x < hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].x
        fingers_extended.insert(0, thumb_extended)
        
        # 제스처 분류 로직
        
        # 주먹 (stop): 모든 손가락이 접혀 있음
        if all(not extended for extended in fingers_extended[1:]):
            return "stop"
        
        # 왼쪽 (left): 엄지가 왼쪽으로 뻗음
        if fingers_extended[0] and thumb_tip.x < wrist.x:
            return "left"
            
        # 오른쪽 (right): 엄지가 오른쪽으로 뻗음
        if fingers_extended[0] and thumb_tip.x > wrist.x:
            return "right"
        
        # 일로와 (go): 대부분의 손가락이 펴지고, 손가락 끝이 손목보다 위에 있음
        if sum(fingers_extended[1:]) >= 3 and finger_tips_y_avg < wrist.y:
            return "go"
            
        # 저리가 (back): 대부분의 손가락이 펴지고, 손가락 끝이 손목보다 아래에 있음
        if sum(fingers_extended[1:]) >= 3 and finger_tips_y_avg > wrist.y:
            return "back"
            
        return "none"  # 기본값
    
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
    
    def send_gesture_data(self, gesture):
        """감지된 제스처를 UDP로 전송"""
        # 전송할 데이터 생성 (JSON 형식)
        gesture_data = {
            "robot_id": self.robot_id,
            "gesture": gesture,
            "timestamp": time.time()
        }
        
        # JSON 직렬화
        json_data = json.dumps(gesture_data).encode('utf-8')
        
        try:
            # UDP로 전송
            self.send_sock.sendto(json_data, ('127.0.0.1', HAND_GESTURE_BRIDGE))
            print(f'[정보] 제스처 전송: {gesture}')
        except Exception as e:
            print(f'[오류] 제스처 전송 실패: {str(e)}')
    
    def stop(self):
        """모든 스레드 정지"""
        self.is_running = False
        if self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        if self.process_thread.is_alive():
            self.process_thread.join(timeout=1.0)
        self.recv_sock.close()
        self.send_sock.close()
        print('손 제스처 감지 모듈 종료')

def main():
    detector = HandGestureDetector()
    
    try:
        detector.start()
        # 메인 스레드는 무한 루프
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('키보드 인터럽트 감지. 종료 중...')
    finally:
        detector.stop()

if __name__ == '__main__':
    main()
