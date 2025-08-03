#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
from libo_interfaces.srv import SetGoal  # SetGoal 서비스 추가
from libo_interfaces.srv import NavigationResult  # NavigationResult 서비스 추가
from libo_interfaces.srv import ActivateDetector  # ActivateDetector 서비스 추가
from libo_interfaces.srv import DeactivateDetector  # DeactivateDetector 서비스 추가
from libo_interfaces.srv import ActivateQRScanner  # ActivateQRScanner 서비스 추가
from libo_interfaces.srv import DeactivateQRScanner  # DeactivateQRScanner 서비스 추가
from libo_interfaces.srv import CancelNavigation  # CancelNavigation 서비스 추가
from libo_interfaces.srv import EndTask  # EndTask 서비스 추가
from libo_interfaces.srv import RobotQRCheck  # RobotQRCheck 서비스 추가
from libo_interfaces.srv import ActivateTalker  # ActivateTalker 서비스 추가
from libo_interfaces.srv import DeactivateTalker  # DeactivateTalker 서비스 추가
from libo_interfaces.srv import ActivateTracker  # ActivateTracker 서비스 추가
from libo_interfaces.srv import DeactivateTracker  # DeactivateTracker 서비스 추가
from libo_interfaces.srv import AddGoalLocation  # AddGoalLocation 서비스 추가
from libo_interfaces.msg import Heartbeat  # Heartbeat 메시지 추가
from libo_interfaces.msg import OverallStatus  # OverallStatus 메시지 추가
from libo_interfaces.msg import TaskStatus  # TaskStatus 메시지 추가
from libo_interfaces.msg import DetectionTimer  # DetectionTimer 메시지 추가
from libo_interfaces.msg import VoiceCommand  # VoiceCommand 메시지 추가
from libo_interfaces.msg import Expression  # Expression 메시지 추가
from std_msgs.msg import Float32  # 무게 데이터 메시지 추가
from std_msgs.msg import String  # LED 제어용 메시지
import time  # 시간 관련 기능
import uuid  # 고유 ID 생성
import random  # 랜덤 좌표 생성용
from enum import Enum  # 상태 열거형
import threading  # 스레드 관리

# 좌표 매핑 딕셔너리 (A1~E9까지 총 45개 좌표)
LOCATION_COORDINATES = {
    # A열 좌표들 (이미지 데이터 기반)
    'A1': (-5.54, 5.76), 'A2': (-3.59, 5.69), 'A3': (-0.19, 5.53), 'A4': (0.0, 0.0), 'A5': (0.0, 0.0),
    'A6': (0.0, 0.0), 'A7': (0.0, 0.0), 'A8': (0.0, 0.0), 'A9': (0.0, 0.0),
    
    # B열 좌표들 (이미지 데이터 기반)
    'B1': (5.57, 4.90), 'B2': (0.0, 0.0), 'B3': (0.01, 4.01), 'B4': (0.0, 0.0), 'B5': (0.0, 0.0),
    'B6': (0.0, 0.0), 'B7': (0.0, 0.0), 'B8': (0.0, 0.0), 'B9': (0.0, 0.0),
    
    # C열 좌표들 (이미지 데이터 기반)
    'C1': (-5.79, 3.25), 'C2': (0.0, 0.0), 'C3': (0.06, 2.70), 'C4': (1.61, 2.65), 'C5': (2.97, 2.57),
    'C6': (4.45, 2.58), 'C7': (5.74, 2.49), 'C8': (7.53, 2.36), 'C9': (8.97, 1.96),
    
    # D열 좌표들 (이미지 데이터 기반)
    'D1': (0.0, 0.0), 'D2': (0.0, 0.0), 'D3': (0.03, 0.96), 'D4': (0.0, 0.0), 'D5': (2.92, 0.98),
    'D6': (0.0, 0.0), 'D7': (5.74, 1.18), 'D8': (0.0, 0.0), 'D9': (9.10, 1.11),
    
    # E열 좌표들 (이미지 데이터 기반)
    'E1': (0.0, 0.0), 'E2': (0.0, 0.0), 'E3': (0.05, -0.34), 'E4': (1.66, -0.53), 'E5': (3.06, -0.51),
    'E6': (4.53, -0.53), 'E7': (5.74, -0.12), 'E8': (7.67, -0.10), 'E9': (8.98, -0.16),
    
    # Base 좌표 (스테이지 3 완료 후 돌아갈 위치) - E3로 고정
    'Base': (0.05, -0.34),  # E3 좌표와 동일
    
    # Admin Desk 좌표 (Delivery Task용)
    'admin_desk': (-5.79, 3.25)  # 관리자 데스크 위치
}

# 관리자 이름 리스트 (QR Check용)
ADMIN_NAMES = ['김대인', '김민수', '박태환', '이건우', '이승훈']

# 음성 명령 상수 정의
VOICE_COMMANDS = {
    # 공통 음성 명령
    "common": {
        "power_on": "power_on.mp3",  # 전원 켜지는 소리 - 삐빅
        "initialized": "robot_initialized.mp3",  # 초기화 완료 소리 - 띠리리리리링
        "charging": "충전을 시작하겠습니다.",
        "battery_sufficient": "배터리가 충분합니다. 대기모드로 전환합니다.",
        "depart_base": "출발합니다~ (충전기를 뽑고)",
        "obstacle_detected": "honk.mp3",  # 장애물이 감지됐습니다. 정지합니다. / 빵!!!!!!!!!!!
        "reroute": "새로운 경로로 안내합니다.",
        "return": "complete.mp3",  # 복귀하겠습니다. / (복귀음 소리 - 삐빅)
        "arrived_base": "Base에 도착했습니다.",
        "navigation_canceled": "네비게이션이 취소되었습니다.",
        "emergency_stop": "비상 정지! 안전을 위해 모든 작업을 중단합니다.",  # 비상 정지 알림
        "emergency_recovery": "비상 상황이 해결되었습니다. 정상 상태로 복구합니다."  # 복구 완료 알림
    },
    
    # 안내 관련 음성 명령
    "escort": {
        "depart_base": "출발합니다~",
        "arrived_kiosk": "책 위치까지 에스코팅을 시작하겠습니다, 뒤로 따라와주시길 바랍니다.",
        "lost_user": "손님이 보이지 않습니다. 20초 뒤에 자동종료 됩니다.",
        "user_reconnected": "reconnected.mp3",  # 다시 연결된 소리. 뾰로롱? 삐빅?
        "arrived_destination": "도착했습니다. 더 필요한 것이 있으면 키오스크에서 불러주세요.",
        "return": "complete.mp3",  # 복귀하겠습니다. / (복귀음 소리 - 삐빅)
        "arrived_base": "Base에 도착했습니다."
    },
    
    # 배송 관련 음성 명령
    "delivery": {
        "depart_base": "출발합니다~",
        "arrived_admin_desk": "딜리버리 준비가 완료되었습니다. 다음 목적지를 선택해주세요.",
        "receive_next_goal": "목적지를 수신하였습니다. 출발하겠습니다.",
        "arrived_destination": "도착했습니다. 작업이 완료되면 말해주세요.",
        "called_by_staff": "ribo_response.mp3",  # 네? / (삐빅)
        "return": "complete.mp3",  # 복귀하겠습니다. / (복귀음 소리 - 삐빅)
        "arrived_base": "Base에 도착했습니다."
    },
    
    # 도움 관련 음성 명령
    "assist": {
        "depart_base": "출발합니다~",
        "arrived_kiosk": "어시스트를 시작하시려면 QR 코드를 카메라 앞에 대주세요",
        "qr_authenticated": "QR 인증 완료! 어시스트를 시작하려면 카메라 앞에서 대기 해주시길 바랍니다.",
        "no_person_5s": "감지 실패!",
        "person_detected": "감지 성공!",
        "called_by_staff": "ribo_response.mp3",  # 네? / (삐빅)
        "pause": "일시정지합니다.",
        "resume": "어시스트를 재개합니다.",
        "return": "complete.mp3",  # 복귀하겠습니다. / (복귀음 소리 - 삐빅)
        "arrived_base": "Base에 도착했습니다."
    }
}

# 로봇 상태 정의 (시스템 상태 제거)
class RobotState(Enum):  # 개별 로봇 상태
    INIT = "INIT"  # 초기화
    CHARGING = "CHARGING"  # 충전
    STANDBY = "STANDBY"  # 대기
    ESCORT = "ESCORT"  # 에스코트 작업
    DELIVERY = "DELIVERY"  # 딜리버리 작업
    ASSIST = "ASSIST"  # 어시스트 작업
    EMERGENCY = "EMERGENCY"  # 비상 상황 (모든 작업 중단, 안전 대기)

class Robot:  # 로봇 정보를 담는 클래스
    def __init__(self, robot_id):  # Robot 객체 초기화
        self.robot_id = robot_id  # 로봇 ID 저장
        self.last_heartbeat_time = time.time()  # 마지막 하트비트 수신 시간
        self.current_state = RobotState.INIT  # 현재 로봇 상태 (기본값: 초기화)
        self.is_available = False  # 초기화 상태는 사용 불가 (INIT 상태에 맞게)
        self.state_start_time = time.time()  # 현재 상태 시작 시간 추가
    
    def update_heartbeat(self):  # 하트비트 업데이트
        """하트비트를 받았을 때 호출되는 메서드"""
        self.last_heartbeat_time = time.time()  # 마지막 하트비트 시간 업데이트

    def check_timeout(self, timeout_seconds=3):  # 타임아웃 체크
        """지정된 시간(기본 3초) 이내에 하트비트가 수신되었는지 확인하는 메서드"""
        current_time = time.time()  # 현재 시간을 가져옴
        time_since_last_heartbeat = current_time - self.last_heartbeat_time  # 마지막 하트비트를 받은 후 얼마나 시간이 지났는지 계산
        return time_since_last_heartbeat <= timeout_seconds  # 타임아웃 여부를 직접 반환 (True: 정상, False: 타임아웃)
    
    def set_available(self, available):  # 사용 가능 상태 설정
        """로봇의 사용 가능 상태를 설정하는 메서드"""
        self.is_available = available  # 사용 가능 상태 업데이트
    
    def change_state(self, new_state):  # 로봇 상태 변경
        """로봇의 상태를 변경하고 해당 상태에 맞는 availability 설정"""
        old_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()  # 상태 변경 시간 기록
        
        # 상태에 따른 availability 자동 설정
        if new_state == RobotState.STANDBY:
            self.is_available = True  # 대기 상태일 때만 사용 가능
        elif new_state == RobotState.EMERGENCY:
            self.is_available = False  # 비상 상황 시 사용 불가
        else:
            self.is_available = False  # 나머지 상태는 모두 사용 불가
        
        return old_state, new_state
    
    def get_status_info(self):  # 로봇 상태 정보 반환
        """로봇의 현재 상태 정보를 문자열로 반환"""
        available_status = "사용가능" if self.is_available else "사용중"
        return f"Robot[{self.robot_id}] - {self.current_state.value} | {available_status}"

class Task:  # 작업 정보를 담는 클래스
    def __init__(self, robot_id, task_type, call_location, goal_location):  # Task 객체 초기화
        self.task_id = str(uuid.uuid4())[:8]  # 고유한 작업 ID 생성 (8자리)
        self.robot_id = robot_id  # 로봇 ID 저장
        self.task_type = task_type  # 작업 타입 저장
        self.call_location = call_location  # 호출지 위치 저장
        self.goal_location = goal_location  # 목적지 위치 저장
        self.start_time = time.time()  # 시작 시간 기록
        self.end_time = None  # 종료 시간 (아직 없음)
        self.status = "created"  # 작업 상태 (created, running, completed, failed)
        self.stage = 1  # 작업 단계 (1: 시작, 2: 진행중, 3: 완료 직전)
    
    def get_info(self):  # 작업 정보 반환
        """작업의 현재 정보를 문자열로 반환"""
        return f"Task[{self.task_id}] - {self.robot_id} | {self.task_type} | {self.call_location} -> {self.goal_location} | Status: {self.status} | Stage: {self.stage}"

class TaskManager(Node):
    def __init__(self):  # TaskManager 노드 초기화 및 서비스 서버 설정
        super().__init__('task_manager')
        
        # TaskRequest 서비스 서버 생성
        self.service = self.create_service(
            TaskRequest,
            'task_request',
            self.task_request_callback
        )
        
        # Navigator로 SetGoal 보내는 서비스 클라이언트 생성
        self.navigator_client = self.create_client(SetGoal, 'set_navigation_goal')
        
        # NavigationResult 서비스 서버 생성
        self.navigation_result_service = self.create_service(
            NavigationResult,
            'navigation_result',
            self.navigation_result_callback
        )
        
        # ActivateDetector 서비스 클라이언트 생성
        self.activate_detector_client = self.create_client(ActivateDetector, 'activate_detector')
        
        # DeactivateDetector 서비스 클라이언트 생성
        self.deactivate_detector_client = self.create_client(DeactivateDetector, 'deactivate_detector')
        
        # ActivateQRScanner 서비스 클라이언트 생성
        self.activate_qr_scanner_client = self.create_client(ActivateQRScanner, 'activate_qr_scanner')
        
        # DeactivateQRScanner 서비스 클라이언트 생성
        self.deactivate_qr_scanner_client = self.create_client(DeactivateQRScanner, 'deactivate_qr_scanner')
        
        # CancelNavigation 서비스 클라이언트 생성
        self.cancel_navigation_client = self.create_client(CancelNavigation, 'cancel_navigation')
        
        # ActivateTalker 서비스 클라이언트 생성
        self.activate_talker_client = self.create_client(ActivateTalker, 'activate_talker')
        
        # DeactivateTalker 서비스 클라이언트 생성
        self.deactivate_talker_client = self.create_client(DeactivateTalker, 'deactivate_talker')
        
        # ActivateTracker 서비스 클라이언트 생성
        self.activate_tracker_client = self.create_client(ActivateTracker, 'activate_tracker')
        
        # DeactivateTracker 서비스 클라이언트 생성
        self.deactivate_tracker_client = self.create_client(DeactivateTracker, 'deactivate_tracker')
        
        # EndTask 서비스 서버 생성
        self.end_task_service = self.create_service(
            EndTask,
            'end_task',
            self.end_task_callback
        )
        
        # RobotQRCheck 서비스 서버 생성
        self.robot_qr_check_service = self.create_service(
            RobotQRCheck,
            'robot_qr_check',
            self.robot_qr_check_callback
        )
        
        # AddGoalLocation 서비스 서버 생성
        self.add_goal_location_service = self.create_service(
            AddGoalLocation,
            'add_goal_location',
            self.add_goal_location_callback
        )
        
        # Heartbeat 토픽 구독자 생성
        self.heartbeat_subscription = self.create_subscription(
            Heartbeat,  # 메시지 타입
            'heartbeat',  # 토픽 이름
            self.heartbeat_callback,  # 콜백 함수
            # QoS 프로파일 설정 (Heartbeat Sender와 호환되도록)
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 최선 노력 수신
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # 휘발성
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,  # 마지막 N개 메시지만 유지
                depth=10  # 큐 깊이
            )
        )
        
        # DetectionTimer 토픽 구독자 생성
        self.detection_timer_subscription = self.create_subscription(
            DetectionTimer,  # 메시지 타입
            'detection_timer',  # 토픽 이름
            self.detection_timer_callback,  # 콜백 함수
            10  # QoS depth
        )
        
        # 무게 데이터 토픽 구독자 생성
        self.weight_subscription = self.create_subscription(
            Float32,  # 메시지 타입
            'weight_data',  # 토픽 이름
            self.weight_callback,  # 콜백 함수
            10  # QoS depth
        )
        
        # VoiceCommand 토픽 퍼블리셔 생성
        self.voice_command_publisher = self.create_publisher(VoiceCommand, 'voice_command', 10)
        
        # LED 제어용 퍼블리셔 생성
        self.led_publisher = self.create_publisher(String, 'led_status', 10)
        
        # Expression 퍼블리셔 생성
        self.expression_publisher = self.create_publisher(Expression, 'expression', 10)
        
        # 작업 목록을 저장할 리스트
        self.tasks = []  # 생성된 작업들을 저장할 리스트
        
        # 로봇 목록을 저장할 딕셔너리 (robot_id를 키로 사용)
        self.robots = {}  # 로봇들을 저장할 딕셔너리
        
        # 무게 데이터 저장 변수
        self.current_weight = 0.0  # 현재 무게 (g 단위)
        self.last_weight_update = None  # 마지막 무게 업데이트 시간
        
        # OverallStatus 퍼블리셔 생성
        self.status_publisher = self.create_publisher(OverallStatus, 'robot_status', 10)  # OverallStatus 토픽 퍼블리셔
        
        # TaskStatus 퍼블리셔 생성
        self.task_status_publisher = self.create_publisher(TaskStatus, 'task_status', 10)  # TaskStatus 토픽 퍼블리셔
        
        # 로봇 상태 체크 타이머 (1초마다 실행)
        self.robot_check_timer = self.create_timer(1.0, self.check_robot_timeouts)  # 1초마다 로봇 타임아웃 체크
        
        # 로봇 상태 발행 타이머 (1초마다 실행)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)  # 1초마다 로봇 상태 발행
        
        # TaskStatus 발행 타이머 (1초마다 실행)
        self.task_status_timer = self.create_timer(1.0, self.publish_task_status)  # 1초마다 더미 작업 상태 발행
        
        # 로봇 상태 관리 타이머 (1초마다 실행)
        self.robot_state_timer = self.create_timer(1.0, self.manage_robot_states)  # 1초마다 로봇 상태 관리
        
        # Task 타입별 Stage 로직 정의 (통합 관리)
        self.task_stage_logic = {
            
            # 구조 설명:
            # self.task_stage_logic = {
            #     'task_type': {                    # 작업 타입 (escort, assist, delivery)
            #         stage_number: {               # 스테이지 번호 (1, 2, 3)
            #             'event_type': [           # 이벤트 타입 (stage_start, timer_10s, timer_30s 등)
            #                 {'action': 'action_type', 'param': 'value'},  # 실행할 액션들
            #                 ...
            #             ]
            #         }
            #     }
            # }
            #
            # 이벤트 타입 종류:
            # - 'stage_start': 스테이지가 시작될 때 (Stage 1→2, 2→3, 3→완료 시)
            # - 'timer_10s': 타이머가 10초일 때 (DetectionTimer에서 발생)
            # - 'timer_30s': 타이머가 30초일 때 (DetectionTimer에서 발생)
            #
            # 액션 타입 종류:
            # - 'voice': 음성 명령 발행 (command: 음성 명령 종류)
            # - 'led': LED 제어 (emotion: 감정 상태)
            # - 'navigate': 네비게이션 (target: 목표 위치)
            # - 'activate_detector': 감지기 활성화
            # - 'deactivate_detector': 감지기 비활성화
            # - 'cancel_navigation': 네비게이션 취소
            # - 'force_stage': 강제 스테이지 변경 (target: 목표 스테이지)


            # Escort Task: 사용자 에스코팅 (사용자 추적 및 안내)
            # - Stage 1: 호출지로 이동
            # - Stage 2: 사용자 추적 (감지기 활성화) + 목적지로 이동
            # - Stage 3: Base로 복귀
            # - 특별 기능: timer_10s(사용자 분실 경고), timer_30s(강제 복귀)
            'escort': {  # 에스코트 작업 타입 정의
                1: {  # Stage 1: 호출지로 이동하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'voice', 'command': 'depart_base'},  # 출발 음성 명령
                        {'action': 'led', 'emotion': '슬픔'},  # 출근길 슬픔 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '슬픔'},  # 출근길 슬픔 표현
                        {'action': 'navigate', 'target': 'call_location'}  # 호출지로 네비게이션
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'arrived_kiosk'},  # 키오스크 도착 음성
                        {'action': 'advance_stage'}  # Stage 2로 진행
                    ]
                },
                2: {  # Stage 2: 사용자 추적 및 목적지로 이동하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'activate_detector'},  # 사용자 감지기 활성화
                        {'action': 'led', 'emotion': '화남'},  # 업무 중 화남 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '화남'},  # 업무 중 화남 표현
                        {'action': 'navigate', 'target': 'goal_location'}  # 목적지로 네비게이션
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'arrived_destination'},  # 목적지 도착 음성 명령
                        {'action': 'deactivate_detector'},  # 감지기 비활성화
                        {'action': 'advance_stage'}  # Stage 3으로 진행
                    ],
                    'timer_10s': [  # 10초 타이머 시 실행할 액션들
                        {'action': 'voice', 'command': 'lost_user'}  # 사용자 분실 경고 음성
                    ],
                    'timer_30s': [  # 30초 타이머 시 실행할 액션들
                        {'action': 'cancel_navigation'},  # 네비게이션 취소
                        {'action': 'deactivate_detector'},  # 감지기 비활성화
                        {'action': 'force_stage', 'target': 3}  # 강제로 Stage 3으로 이동
                    ]
                },
                3: {  # Stage 3: Base로 복귀하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'voice', 'command': 'return'},  # 복귀 음성 명령
                        {'action': 'led', 'emotion': '기쁨'},  # 퇴근길 기쁨 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '기쁨'},  # 퇴근길 기쁨 표현
                        {'action': 'navigate', 'target': 'base'}  # Base로 네비게이션
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'arrived_base'},  # Base 도착 음성 명령
                        {'action': 'advance_stage'}  # Task 완료 (Stage 4로 진행하여 완료 처리)
                    ]
                }
            },
            
            # Assist Task: 사용자 어시스트 (QR 인증 및 도움)
            # - Stage 1: 호출지로 이동
            # - Stage 2: QR 인증 대기 (목적지 없음)
            # - Stage 3: Base로 복귀
            'assist': {  # 어시스트 작업 타입 정의
                1: {  # Stage 1: 호출지로 이동하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'voice', 'command': 'depart_base'},  # 출발 음성 명령
                        {'action': 'led', 'emotion': '슬픔'},  # 슬픔 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '슬픔'},  # 슬픔 표현
                        {'action': 'navigate', 'target': 'call_location'}  # 호출지로 네비게이션
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'activate_qr_scanner'},  # QR Scanner 활성화
                        {'action': 'voice', 'command': 'arrived_kiosk'}  # 키오스크 도착 음성
                    ],
                    'navigation_canceled': [  # 네비게이션 취소 시 실행할 액션들
                        {'action': 'voice', 'command': 'navigation_canceled'},  # 네비게이션 취소 알림
                        {'action': 'force_stage', 'target': 3}  # Stage 3으로 강제 진행
                    ],
                    'qr_scanner_activated': [  # QR Scanner 활성화 성공 시 실행할 액션들
                        # QR Scanner 활성화 완료 - QR Check 메시지 대기 중
                        # TODO: KioskQRCheck.srv / RobotQRCheck 메시지 처리 후 qr_check_completed 이벤트 발생
                    ],
                    'qr_check_completed': [  # QR Check 완료 시 실행할 액션들
                        {'action': 'deactivate_qr_scanner'},  # QR Scanner 비활성화
                        {'action': 'voice', 'command': 'qr_authenticated'},  # QR 인증 완료 음성 명령
                        {'action': 'advance_stage'}  # Stage 2로 진행
                    ]
                },
                2: {  # Stage 2: QR 인증 대기하는 단계 (목적지 이동 없음)
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'led', 'emotion': '화남'},  # 화남 LED 표시 (네비게이션 없음)
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '화남'},  # 화남 표현
                        {'action': 'activate_tracker'},  # Tracker 활성화
                        {'action': 'activate_talker'}  # Talker 활성화
                    ],
                    'tracker_failed': [  # Tracker 활성화 실패 시 실행할 액션들
                        {'action': 'deactivate_talker'},  # Talker 비활성화
                        {'action': 'advance_stage'}  # 다음 스테이지로 진행
                    ],
                    'talker_failed': [  # Talker 활성화 실패 시 실행할 액션들
                        {'action': 'deactivate_tracker'},  # Tracker 비활성화
                        {'action': 'advance_stage'}  # 다음 스테이지로 진행
                    ],
                    'navigation_canceled': [  # 네비게이션 취소 시 실행할 액션들
                        {'action': 'voice', 'command': 'navigation_canceled'},  # 네비게이션 취소 알림
                        {'action': 'force_stage', 'target': 3}  # Stage 3으로 강제 진행
                    ],
                    'end_task': [  # EndTask 요청 시 실행할 액션들
                        {'action': 'deactivate_tracker'},  # Tracker 비활성화
                        {'action': 'deactivate_talker'},  # Talker 비활성화
                        {'action': 'advance_stage'}  # Stage 3으로 진행
                    ]
                },
                3: {  # Stage 3: Base로 복귀하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'voice', 'command': 'return'},  # 복귀 음성 명령
                        {'action': 'led', 'emotion': '기쁨'},  # 기쁨 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '기쁨'},  # 기쁨 표현
                        {'action': 'navigate', 'target': 'base'}  # Base로 네비게이션
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'arrived_base'},  # Base 도착 음성 명령
                        {'action': 'advance_stage'}  # Task 완료 (Stage 4로 진행하여 완료 처리)
                    ]
                }
            },
            
            # Delivery Task: 물품 배송
            # - Stage 1: admin PC로 이동
            # - Stage 2: 물품 수령 + 목적지로 이동
            # - Stage 3: Base로 복귀
            'delivery': {  # 배송 작업 타입 정의
                1: {  # Stage 1: admin PC로 이동하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'voice', 'command': 'depart_base'},  # 출발 음성 명령
                        {'action': 'led', 'emotion': '슬픔'},  # 슬픔 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '슬픔'},  # 슬픔 표현
                        {'action': 'navigate', 'target': 'admin_desk'}  # admin PC로 네비게이션
                    ],
                    'navigation_canceled': [  # 네비게이션 취소 시 실행할 액션들
                        {'action': 'voice', 'command': 'navigation_canceled'},  # 네비게이션 취소 알림
                        {'action': 'force_stage', 'target': 3}  # Stage 3으로 강제 진행
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'arrived_admin_desk'},  # admin PC 도착 음성
                        {'action': 'advance_stage'}  # Stage 2로 진행
                    ]
                },
                2: {  # Stage 2: 물품 수령 및 목적지로 이동하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        # 도착후 대기 한다고 알림
                        # 관리자가 맵으로 다음 목적지를 선택하기 전까지 대기
                        {'action': 'led', 'emotion': '화남'},  # 화남 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '화남'},  # 화남 표현
                        # AddGoalLocation.srv가 성공적으로 도달할 때까지 대기
                    ],
                    'goal_location_updated': [  # AddGoalLocation 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'receive_next_goal'},  # 다음 목적지 수령 알림
                        {'action': 'navigate', 'target': 'goal_location'}  # 목적지로 네비게이션
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'arrived_destination'},  # 목적지 도착 알림
                        # 관리자가 별도로 "이제 돌아가" 라고 지시 하지 않는이상 대기
                    ],
                    'navigation_canceled': [  # 네비게이션 취소 시 실행할 액션들
                        {'action': 'voice', 'command': 'navigation_canceled'},  # 네비게이션 취소 알림
                        {'action': 'force_stage', 'target': 3}  # Stage 3으로 강제 진행
                    ],
                    'end_task': [  # EndTask 요청 시 실행할 액션들
                        {'action': 'advance_stage'}  # Stage 3으로 진행
                    ]
                },
                3: {  # Stage 3: Base로 복귀하는 단계
                    'stage_start': [  # 스테이지 시작 시 실행할 액션들
                        {'action': 'voice', 'command': 'return'},  # 복귀 음성 명령
                        {'action': 'led', 'emotion': '기쁨'},  # 기쁨 LED 표시
                        {'action': 'expression', 'robot_id': 'robot_id', 'status': '기쁨'},  # 기쁨 표현
                        {'action': 'navigate', 'target': 'base'}  # Base로 네비게이션
                    ],
                    'navigation_success': [  # 네비게이션 성공 시 실행할 액션들
                        {'action': 'voice', 'command': 'arrived_base'},  # Base 도착 음성 명령
                        {'action': 'advance_stage'}  # Task 완료 (Stage 4로 진행하여 완료 처리)
                    ]
                }
            }
        }
        
        self.get_logger().info('🎯 Task Manager 시작됨 - task_request 서비스 대기 중...')
        self.get_logger().info('💓 Heartbeat 구독 시작됨 - heartbeat 토픽 모니터링 중...')
        self.get_logger().info('📡 OverallStatus 발행 시작됨 - robot_status 토픽으로 1초마다 발행...')
        self.get_logger().info('📋 TaskStatus 발행 시작됨 - task_status 토픽으로 1초마다 발행...')  # TaskStatus 로그 추가
        self.get_logger().info('🧭 Navigator 클라이언트 준비됨 - set_navigation_goal 서비스 연결...')  # Navigator 클라이언트 로그 추가
        self.get_logger().info('📍 NavigationResult 서비스 시작됨 - navigation_result 서비스 대기 중...')  # NavigationResult 서버 로그 추가
        self.get_logger().info('👁️ ActivateDetector 클라이언트 준비됨 - activate_detector 서비스 연결...')
        self.get_logger().info('👁️ DeactivateDetector 클라이언트 준비됨 - deactivate_detector 서비스 연결...')
        self.get_logger().info('👁️ ActivateQRScanner 클라이언트 준비됨 - activate_qr_scanner 서비스 연결...')
        self.get_logger().info('👁️ DeactivateQRScanner 클라이언트 준비됨 - deactivate_qr_scanner 서비스 연결...')
        self.get_logger().info('⏹️ CancelNavigation 클라이언트 준비됨 - cancel_navigation 서비스 연결...')
        self.get_logger().info('🏁 EndTask 서비스 시작됨 - end_task 서비스 대기 중...')
        self.get_logger().info('🔍 RobotQRCheck 서비스 시작됨 - robot_qr_check 서비스 대기 중...')
        self.get_logger().info('⏰ DetectionTimer 구독 시작됨 - detection_timer 토픽 모니터링 중...')
        self.get_logger().info('🗣️ VoiceCommand 퍼블리셔 준비됨 - voice_command 토픽으로 이벤트 기반 발행...')
        self.get_logger().info('⚖️ 무게 데이터 구독 시작됨 - weight_data 토픽 모니터링 중...')
        self.get_logger().info('🔄 통합 Task Stage 로직 시스템 활성화됨')
        self.get_logger().info('🗣️ ActivateTalker 클라이언트 준비됨 - activate_talker 서비스 연결...')
        self.get_logger().info('🗣️ DeactivateTalker 클라이언트 준비됨 - deactivate_talker 서비스 연결...')
        self.get_logger().info('🎯 ActivateTracker 클라이언트 준비됨 - activate_tracker 서비스 연결...')
        self.get_logger().info('🎯 DeactivateTracker 클라이언트 준비됨 - deactivate_tracker 서비스 연결...')
    
    def check_robot_timeouts(self):  # 로봇 타임아웃 체크
        """1초마다 로봇 목록을 확인하여 타임아웃된 로봇을 목록에서 제거"""
        inactive_robots = []  # 비활성 로봇 ID를 저장할 리스트
        for robot_id, robot in self.robots.items():  # 현재 등록된 모든 로봇에 대해 반복
            if not robot.check_timeout():  # 로봇의 타임아웃 여부를 확인
                inactive_robots.append(robot_id)  # 타임아웃된 로봇을 리스트에 추가
        
        for robot_id in inactive_robots:  # 비활성 로봇 리스트에 있는 모든 로봇에 대해 반복
            del self.robots[robot_id]  # 로봇 목록에서 해당 로봇을 제거
            self.get_logger().info(f'🤖 로봇 <{robot_id}> 제거됨 (사유: Heartbeat 타임아웃)')  # 로봇 제거 로그 출력
    
    def heartbeat_callback(self, msg):  # Heartbeat 메시지 수신 콜백
        """Heartbeat 메시지를 받았을 때 호출되는 콜백 함수"""
        try:
            # sender_id가 로봇인 경우에만 Robot 객체를 생성하거나 업데이트
            if msg.sender_id in self.robots:  # 이미 등록된 로봇이라면
                self.robots[msg.sender_id].update_heartbeat()  # 하트비트 시간만 갱신해줌
            else:  # 처음 보는 로봇이라면
                self.robots[msg.sender_id] = Robot(msg.sender_id)  # 새로운 로봇 객체를 생성해서 목록에 추가
                self.get_logger().info(f'🤖 새로운 로봇 <{msg.sender_id}> 감지됨')  # 새로운 로봇 감지 로그 출력
            
        except Exception as e:  # 예외 발생 시 처리
            self.get_logger().error(f'❌ Heartbeat 처리 중 오류: {e}')  # 에러 로그
    
    def publish_robot_status(self):  # 로봇 상태 발행
        """1초마다 현재 활성 로봇들의 OverallStatus 발행"""
        if not self.robots:  # 로봇이 없으면 로그만 출력
            self.get_logger().debug(f'📡 발행할 로봇이 없음 (등록된 로봇: 0개)')
            return
            
        for robot_id, robot in self.robots.items():  # 현재 활성 로봇들에 대해 반복 (robot 객체도 가져옴)
            status_msg = OverallStatus()  # OverallStatus 메시지 생성
            status_msg.timestamp = self.get_clock().now().to_msg()  # 현재 시간 설정
            status_msg.robot_id = robot_id  # 로봇 ID 설정
            status_msg.robot_state = robot.current_state.value  # 로봇 상태 추가 (INIT, CHARGING, STANDBY 등)
            status_msg.is_available = robot.is_available  # 실제 로봇의 사용 가능 상태 사용
            
            # 배터리 시뮬레이션 (시간에 따라 감소, CHARGING 상태일 때는 증가)
            current_time = time.time()
            if robot.current_state == RobotState.CHARGING:
                # 충전 중일 때는 배터리가 점진적으로 증가 (최대 100%)
                battery_increase = int((current_time - robot.state_start_time) * 2)  # 2% per second
                status_msg.battery = min(100, 20 + battery_increase)  # 최소 20%에서 시작해서 최대 100%
            elif robot.current_state == RobotState.STANDBY:
                # 대기 상태일 때는 배터리가 닳지 않음 (현재 배터리 유지)
                status_msg.battery = 100  # STANDBY 상태는 항상 100% 유지
            else:
                # 다른 상태일 때는 배터리가 점진적으로 감소 (최소 10%)
                battery_decrease = int((current_time - robot.state_start_time) * 0.5)  # 0.5% per second
                status_msg.battery = max(10, 100 - battery_decrease)  # 최대 100%에서 시작해서 최소 10%
            
            # 위치 및 방향 시뮬레이션 (상태에 따라 다른 위치)
            if robot.current_state == RobotState.INIT:
                # 초기화 상태: 기본 위치
                status_msg.position_x = 0.0
                status_msg.position_y = 0.0
                status_msg.position_yaw = 0.0
            elif robot.current_state == RobotState.CHARGING:
                # 충전 상태: 충전소 위치 (E3)
                status_msg.position_x = 3.9
                status_msg.position_y = 8.1
                status_msg.position_yaw = 0.0
            elif robot.current_state == RobotState.STANDBY:
                # 대기 상태: 대기 구역 위치 (A2)
                status_msg.position_x = 6.0
                status_msg.position_y = 0.0
                status_msg.position_yaw = 90.0
            elif robot.current_state in [RobotState.ESCORT, RobotState.DELIVERY, RobotState.ASSIST]:
                # 작업 상태: 현재 활성 task의 위치에 따라 설정
                if self.tasks and self.tasks[0].robot_id == robot_id:
                    current_task = self.tasks[0]
                    if current_task.stage == 1:
                        # Stage 1: CallLocation으로 이동 중
                        if current_task.call_location in LOCATION_COORDINATES:
                            x, y = LOCATION_COORDINATES[current_task.call_location]
                            status_msg.position_x = x
                            status_msg.position_y = y
                            status_msg.position_yaw = 45.0
                    elif current_task.stage == 2:
                        # Stage 2: GoalLocation으로 이동 중
                        if current_task.goal_location in LOCATION_COORDINATES:
                            x, y = LOCATION_COORDINATES[current_task.goal_location]
                            status_msg.position_x = x
                            status_msg.position_y = y
                            status_msg.position_yaw = 135.0
                    elif current_task.stage == 3:
                        # Stage 3: Base로 이동 중
                        x, y = LOCATION_COORDINATES['Base']
                        status_msg.position_x = x
                        status_msg.position_y = y
                        status_msg.position_yaw = 180.0
                else:
                    # Task가 없으면 기본 위치
                    status_msg.position_x = 5.0
                    status_msg.position_y = 5.0
                    status_msg.position_yaw = 0.0
            else:
                # 기타 상태: 기본 위치
                status_msg.position_x = 5.0
                status_msg.position_y = 5.0
                status_msg.position_yaw = 0.0
            
            # 무게 데이터 처리 (libo_a 로봇에만 적용)
            if robot_id == 'libo_a':
                # libo_a 로봇의 경우 실제 무게 데이터 사용 (상태와 무관하게)
                if self.is_weight_data_recent():  # 최근 무게 데이터가 있으면
                    # 실제 무게 적용 (g → kg 변환)
                    status_msg.book_weight = self.current_weight / 1000.0
                    self.get_logger().debug(f'📊 [libo_a] 실제 무게 적용: {self.current_weight:.1f}g → {status_msg.book_weight:.3f}kg')
                else:
                    # 무게 데이터가 없거나 오래된 경우 0.0
                    status_msg.book_weight = 0.0
                    self.get_logger().debug(f'📊 [libo_a] 무게 데이터 없음: 0.0kg')
            else:
                # libo_a가 아닌 다른 로봇들은 0.0
                status_msg.book_weight = 0.0
            
            self.status_publisher.publish(status_msg)  # 메시지 발행
            self.get_logger().debug(f'📡 로봇 상태 발행: {robot_id} → {robot.current_state.value} | {"사용가능" if robot.is_available else "사용중"} | 배터리: {status_msg.battery}%')
    
    def publish_task_status(self):  # 활성 작업들의 상태 발행
        """1초마다 현재 활성 Task들의 TaskStatus 발행"""
        if not self.tasks:  # Task가 없으면 발행하지 않음
            return
            
        for task in self.tasks:  # 현재 활성 Task들에 대해 반복
            task_status_msg = TaskStatus()  # TaskStatus 메시지 생성
            task_status_msg.task_id = task.task_id  # 실제 Task ID
            task_status_msg.robot_id = task.robot_id  # 실제 로봇 ID
            task_status_msg.task_type = task.task_type  # 실제 작업 타입
            task_status_msg.task_stage = task.stage  # 실제 Task stage 사용
            task_status_msg.call_location = task.call_location  # 실제 호출 위치
            task_status_msg.goal_location = task.goal_location  # 실제 목표 위치
            
            # Task 생성 시간을 사용 (현재 시간이 아님)
            task_status_msg.start_time.sec = int(task.start_time)  # Task 시작 시간 (초)
            task_status_msg.start_time.nanosec = int((task.start_time - int(task.start_time)) * 1000000000)  # 나노초 부분
            
            task_status_msg.end_time.sec = 0  # 진행중이므로 종료 시간은 0
            task_status_msg.end_time.nanosec = 0  # 진행중이므로 종료 시간은 0
            
            self.task_status_publisher.publish(task_status_msg)  # 메시지 발행

    def task_request_callback(self, request, response):  # 키오스크로부터 받은 작업 요청을 처리
        """TaskRequest 서비스 콜백"""
        self.get_logger().info(f'📥 Task Request 받음!')
        self.get_logger().info(f'   - Robot ID: {request.robot_id}')
        self.get_logger().info(f'   - Task Type: {request.task_type}')
        self.get_logger().info(f'   - Call Location: {request.call_location}')
        self.get_logger().info(f'   - Goal Location: {request.goal_location}')
        
        # 유효한 task type인지 먼저 확인
        valid_task_types = list(self.task_stage_logic.keys())  # task_stage_logic의 키들을 자동으로 가져옴
        if request.task_type not in valid_task_types:
            self.get_logger().error(f'❌ 유효하지 않은 Task Type: {request.task_type} - 요청 거절')
            response.success = False
            response.message = f"유효하지 않은 Task Type입니다. 지원되는 타입: {', '.join(valid_task_types)}"
            return response
        
        # escort task의 경우 로봇 ID를 무시하고 활성화된 로봇 중 하나를 임의로 선택
        selected_robot_id = request.robot_id
        
        if request.task_type == 'escort' or request.task_type == 'assist':
            self.get_logger().info(f'🚶 Escort/Assist task 감지됨 - 로봇 자동 할당 시작...')
            
            # 사용 가능한 로봇들 찾기
            available_robots = self.get_available_robots()
            
            if not available_robots:
                self.get_logger().error(f'❌ 사용 가능한 로봇이 없음 - Escort/Assist task 거절')
                response.success = False
                response.message = "사용 가능한 로봇이 없어서 Escort/Assist task를 수행할 수 없습니다."
                return response
            
            # 사용 가능한 로봇 중 하나를 임의로 선택
            import random
            selected_robot_id = random.choice(available_robots)
            self.get_logger().info(f'🎲 로봇 자동 할당: {selected_robot_id} (사용 가능한 로봇: {available_robots})')
        
        elif request.task_type == 'delivery':
            self.get_logger().info(f'📦 Delivery task 감지됨 - 로봇 확인 중...')
            
            # robot_id가 비어있거나 존재하지 않는 경우 자동 할당 시도
            if not request.robot_id or request.robot_id not in self.robots:
                self.get_logger().info(f' Delivery task - 로봇 자동 할당 시작... (요청된 로봇: {request.robot_id})')
                
                # 사용 가능한 로봇들 찾기
                available_robots = self.get_available_robots()
                
                if not available_robots:
                    self.get_logger().error(f'❌ 사용 가능한 로봇이 없음 - Delivery task 거절')
                    response.success = False
                    response.message = "사용 가능한 로봇이 없어서 Delivery task를 수행할 수 없습니다."
                    return response
                
                # 사용 가능한 로봇 중 하나를 임의로 선택
                import random
                selected_robot_id = random.choice(available_robots)
                self.get_logger().info(f'🎲 로봇 자동 할당: {selected_robot_id} (사용 가능한 로봇: {available_robots})')
            else:
                # 지정된 로봇이 존재하는 경우, 사용 가능한지 확인
                if not self.robots[request.robot_id].is_available:
                    self.get_logger().error(f'❌ 지정된 로봇 <{request.robot_id}>이 사용중임 - Delivery task 거절')
                    response.success = False
                    response.message = f"지정된 로봇 <{request.robot_id}>이 현재 사용중이어서 Delivery task를 수행할 수 없습니다."
                    return response
                
                selected_robot_id = request.robot_id
                self.get_logger().info(f'✅ 지정된 로봇 <{request.robot_id}> 확인됨 - Delivery task 진행')
        
        # 새로운 Task 객체 생성 (선택된 로봇 ID 사용)
        new_task = Task(selected_robot_id, request.task_type, request.call_location, request.goal_location)  # Task 객체 생성
        self.tasks.append(new_task)  # 작업 목록에 추가
        
        self.get_logger().info(f'✅ 새로운 작업 생성됨: {new_task.get_info()}')  # 생성된 작업 정보 출력
        
        # Task 생성 후 자동으로 로봇을 사용중으로 설정
        if self.set_robot_unavailable_for_task(selected_robot_id):
            self.get_logger().info(f'🔒 로봇 <{selected_robot_id}> 자동으로 사용중 상태로 변경됨')
        else:
            self.get_logger().warning(f'⚠️  로봇 <{selected_robot_id}> 상태 변경 실패 - 로봇이 존재하지 않음')
        
        # 로봇의 state를 task type과 동일하게 변경
        if selected_robot_id in self.robots:
            # task type을 RobotState enum으로 변환
            task_type_to_state = {
                'escort': RobotState.ESCORT,
                'delivery': RobotState.DELIVERY,
                'assist': RobotState.ASSIST
            }
            
            if request.task_type in task_type_to_state:
                new_state = task_type_to_state[request.task_type]
                old_state, _ = self.robots[selected_robot_id].change_state(new_state)
                self.get_logger().info(f'🔄 로봇 <{selected_robot_id}> 상태 변경: {old_state.value} → {new_state.value} (Task Type: {request.task_type})')
            else:
                self.get_logger().warning(f'⚠️  알 수 없는 Task Type: {request.task_type}')
        else:
            self.get_logger().warning(f'⚠️  로봇 <{selected_robot_id}> 찾을 수 없음 - state 변경 불가')
        
        # 새로운 Task의 Stage 1 시작 로직을 통합 시스템으로 처리
        self.get_logger().info(f'🚀 새로운 Task의 Stage 1 시작...')
        self.process_task_stage_logic(new_task, 1, 'stage_start')
        
        # 응답 설정
        response.success = True
        response.message = f"Task request 잘 받았음! Task ID: {new_task.task_id}, 할당된 로봇: {selected_robot_id}"
        
        self.get_logger().info(f'✅ Task Request 처리 완료: {response.message}')
        
        return response
    
    def set_robot_available(self, robot_id, available):  # 로봇 사용 가능 상태 설정
        """특정 로봇의 사용 가능 상태를 설정하는 메서드"""
        self.get_logger().info(f'🔍 로봇 상태 변경 시도: {robot_id} → {"사용가능" if available else "사용중"}')
        self.get_logger().info(f'📋 현재 등록된 로봇들: {list(self.robots.keys())}')
        
        if robot_id in self.robots:  # 로봇이 존재한다면
            old_status = self.robots[robot_id].is_available  # 이전 상태 저장
            self.robots[robot_id].set_available(available)  # 상태 변경
            status_text = "사용가능" if available else "사용중"
            self.get_logger().info(f'🔄 로봇 <{robot_id}> 상태 변경 성공: {old_status} → {available} ({status_text})')
            return True
        else:
            self.get_logger().warning(f'❌ 로봇 <{robot_id}> 찾을 수 없음 - 등록된 로봇: {list(self.robots.keys())}')
            return False
    
    def set_robot_unavailable_for_task(self, robot_id):  # Task 할당 시 로봇을 사용중으로 설정
        """Task가 할당될 때 로봇을 사용중으로 설정"""
        return self.set_robot_available(robot_id, False)
    
    def set_robot_available_after_task(self, robot_id):  # Task 완료 시 로봇을 사용가능으로 설정
        """Task가 완료될 때 로봇을 사용가능으로 설정"""
        return self.set_robot_available(robot_id, True)
    
    def get_available_robots(self):  # 사용 가능한 로봇들 목록 반환
        """현재 사용 가능한 로봇들의 목록을 반환"""
        available_robots = []
        for robot_id, robot in self.robots.items():
            if robot.is_available:  # is_active 체크 제거 (robots에 있다는 것 자체가 활성)
                available_robots.append(robot_id)
        return available_robots
    
    def send_goal_to_navigator(self, x, y):  # Navigator에게 목표 좌표 전송
        """Navigator에게 SetGoal 서비스 요청을 보내는 메서드 (비동기)"""
        # Navigator 서비스가 준비될 때까지 대기
        if not self.navigator_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Navigator 서비스를 찾을 수 없음 (set_navigation_goal)')
            return False
        
        # SetGoal 요청 생성
        request = SetGoal.Request()
        request.x = x  # 목표 x 좌표
        request.y = y  # 목표 y 좌표
        
        self.get_logger().info(f'🧭 Navigator에게 목표 좌표 전송: ({x}, {y})')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.navigator_client.call_async(request)
            future.add_done_callback(self.navigator_response_callback)
            self.get_logger().info(f'📤 Navigator 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ Navigator 통신 중 오류: {e}')
            return False
    
    def send_coordinate_for_stage(self, task):  # 스테이지별로 해당하는 좌표 전송
        """현재 스테이지에 따라 해당하는 좌표를 Navigator에게 전송하는 메서드"""
        if not self.tasks:  # 활성 task가 없으면 리턴
            self.get_logger().warning(f'⚠️  활성 task가 없어서 좌표 전송 불가')
            return False
        
        current_stage = task.stage  # 현재 스테이지
        target_location = None  # 목표 위치
        
        if current_stage == 1:  # 스테이지 1: CallLocation으로 이동
            target_location = task.call_location
            self.get_logger().info(f'🎯 Stage 1: CallLocation <{target_location}> 으로 이동')
        elif current_stage == 2:  # 스테이지 2: GoalLocation으로 이동
            target_location = task.goal_location
            self.get_logger().info(f'🎯 Stage 2: GoalLocation <{target_location}> 으로 이동')
        elif current_stage == 3:  # 스테이지 3: Base로 이동
            target_location = 'Base'
            self.get_logger().info(f'🎯 Stage 3: Base <{target_location}> 으로 이동')
        else:
            self.get_logger().warning(f'⚠️  알 수 없는 스테이지: {current_stage}')
            return False
        
        # 좌표 딕셔너리에서 해당 위치의 좌표 찾기
        if target_location in LOCATION_COORDINATES:
            x, y = LOCATION_COORDINATES[target_location]  # 좌표 추출
            self.get_logger().info(f'📍 좌표 매핑: {target_location} → ({x}, {y})')
            return self.send_goal_to_navigator(x, y)  # Navigator에게 좌표 전송
        else:
            self.get_logger().error(f'❌ 위치 <{target_location}> 에 대한 좌표를 찾을 수 없음')
            return False
    
    def navigator_response_callback(self, future):  # Navigator 응답 콜백
        """Navigator 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Navigator 응답 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️  Navigator 응답 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ Navigator 응답 처리 중 오류: {e}')
    
    def navigation_result_callback(self, request, response):  # NavigationResult 서비스 콜백
        """NavigationResult 요청을 받아서 처리하는 콜백"""
        self.get_logger().info(f'📍 NavigationResult 받음: {request.result}')
        
        try:
            # 현재 활성 task가 있는지 확인
            if not self.tasks or len(self.tasks) == 0:
                self.get_logger().warning(f'⚠️ NavigationResult를 받았지만 활성 task가 없음')
                response.success = True
                response.message = f"NavigationResult 처리 완료: {request.result} (활성 task 없음)"
                return response
            
            current_task = self.tasks[0]
            
            # NavigationResult를 이벤트로 변환하여 task_stage_logic에서 처리
            if request.result == "SUCCEEDED":
                self.get_logger().info(f'✅ 네비게이션 성공! Task[{current_task.task_id}] Stage {current_task.stage}')
                # navigation_success 이벤트를 task_stage_logic에서 처리
                self.process_task_stage_logic(current_task, current_task.stage, 'navigation_success')
                
            elif request.result == "FAILED":
                self.get_logger().warning(f'❌ 네비게이션 실패! Task[{current_task.task_id}] Stage {current_task.stage}')
                # navigation_failed 이벤트를 task_stage_logic에서 처리
                self.process_task_stage_logic(current_task, current_task.stage, 'navigation_failed')
                
            elif request.result == "CANCELED":
                self.get_logger().info(f'⏹️ 네비게이션 취소됨! Task[{current_task.task_id}] Stage {current_task.stage}')
                # navigation_canceled 공통 처리 로직 호출
                self.handle_navigation_canceled(current_task)
                
            else:
                self.get_logger().warning(f'⚠️ 알 수 없는 결과: {request.result}')
            
            # 성공 응답
            response.success = True
            response.message = f"NavigationResult 처리 완료: {request.result}"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'❌ NavigationResult 처리 중 오류: {e}')
            response.success = False
            response.message = f"처리 실패: {str(e)}"
            return response
    
    def advance_stage(self):  # 활성 task의 stage 증가
        """현재 활성화된 task의 stage를 1단계씩 증가시키는 메서드"""
        if not self.tasks:  # 활성 task가 없으면 리턴
            self.get_logger().warning(f'⚠️  SUCCEEDED를 받았지만 활성 task가 없음')
            return
        
        # 첫 번째 활성 task를 대상으로 함 (미니멀 구현)
        current_task = self.tasks[0]
        old_stage = current_task.stage
        
        current_task.stage += 1  # stage 1단계 증가
        
        # Stage별 아이콘
        stage_icons = {1: "🟡", 2: "🔵", 3: "🟢"}
        
        self.get_logger().info(f'🎯 Task[{current_task.task_id}] Stage 변화: {stage_icons.get(old_stage, "⚪")} {old_stage} → {stage_icons.get(current_task.stage, "⚪")} {current_task.stage}')
        
        # stage 3을 넘어가면 task 완료 및 제거
        if current_task.stage > 3:
            current_task.end_time = time.time()  # 종료 시간 기록
            current_task.status = "completed"  # 상태를 완료로 변경
            
            # 로봇을 사용가능 상태로 변경
            if self.set_robot_available_after_task(current_task.robot_id):
                self.get_logger().info(f'🔓 로봇 <{current_task.robot_id}> 사용가능 상태로 변경됨')
            
            # 로봇의 state를 CHARGING으로 변경 (Task 완료 후 충전 상태로)
            if current_task.robot_id in self.robots:
                old_state, _ = self.robots[current_task.robot_id].change_state(RobotState.CHARGING)
                self.get_logger().info(f'🔋 로봇 <{current_task.robot_id}> Task 완료 후 충전 상태로 변경: {old_state.value} → CHARGING')
                
                # Base 도착 음성 명령 발행
                self.get_logger().info(f'🗣️ Base 도착 음성 명령 발행: {current_task.task_type}.arrived_base')
                if self.send_voice_command_by_task_type(current_task.robot_id, current_task.task_type, 'arrived_base'):
                    self.get_logger().info(f'✅ Base 도착 음성 명령 발행 완료')
                else:
                    self.get_logger().warning(f'⚠️ Base 도착 음성 명령 발행 실패')
            else:
                self.get_logger().warning(f'⚠️  로봇 <{current_task.robot_id}> 찾을 수 없음 - state 변경 불가')
            
            # task 목록에서 제거
            self.tasks.remove(current_task)
            
            self.get_logger().info(f'🏁 Task[{current_task.task_id}] 완료 및 제거됨!')
            self.get_logger().info(f'📊 현재 활성 task 수: {len(self.tasks)}개')
        else:
            # Stage 3 이하일 때 현재 상태 로그
            stage_desc = {1: "시작", 2: "진행중", 3: "완료직전"}.get(current_task.stage, f"Stage {current_task.stage}")
            self.get_logger().info(f'📍 현재 상태: {stage_icons.get(current_task.stage, "⚪")} Stage {current_task.stage} ({stage_desc})')
            
            # 새로운 통합 시스템으로 stage_start 이벤트 처리
            self.process_task_stage_logic(current_task, current_task.stage, 'stage_start')
            
            # 기존 좌표 전송 로직은 navigate 액션에서 처리되므로 제거
            # (process_task_stage_logic에서 자동으로 처리됨)

    def test_navigator_communication(self):  # Navigator 통신 테스트
        """더미 좌표로 Navigator 통신을 테스트하는 메서드"""
        test_x = 1.0  # 더미 x 좌표
        test_y = 2.0  # 더미 y 좌표
        self.get_logger().info(f'🧪 Navigator 통신 테스트 시작: ({test_x}, {test_y})')
        result = self.send_goal_to_navigator(test_x, test_y)
        if result:
            self.get_logger().info(f'📤 테스트 요청 전송됨 - 응답은 콜백으로 처리됩니다')
        return result

    def activate_detector(self, robot_id):  # Vision Manager에게 감지기 활성화 요청
        """Vision Manager에게 ActivateDetector 서비스 요청을 보내는 메서드"""
        # Vision Manager 서비스가 준비될 때까지 대기
        if not self.activate_detector_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Vision Manager 서비스를 찾을 수 없음 (activate_detector)')
            return False
        
        # ActivateDetector 요청 생성
        request = ActivateDetector.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'👁️ Vision Manager에게 감지기 활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.activate_detector_client.call_async(request)
            future.add_done_callback(self.activate_detector_response_callback)
            self.get_logger().info(f'📤 감지기 활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ Vision Manager 통신 중 오류: {e}')
            return False

    def deactivate_detector(self, robot_id):  # Vision Manager에게 감지기 비활성화 요청
        """Vision Manager에게 DeactivateDetector 서비스 요청을 보내는 메서드"""
        # Vision Manager 서비스가 준비될 때까지 대기
        if not self.deactivate_detector_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Vision Manager 서비스를 찾을 수 없음 (deactivate_detector)')
            return False
        
        # DeactivateDetector 요청 생성
        request = DeactivateDetector.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'👁️ Vision Manager에게 감지기 비활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.deactivate_detector_client.call_async(request)
            future.add_done_callback(self.deactivate_detector_response_callback)
            self.get_logger().info(f'📤 감지기 비활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ Vision Manager 통신 중 오류: {e}')
            return False

    def activate_detector_response_callback(self, future):  # ActivateDetector 응답 콜백
        """ActivateDetector 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 감지기 활성화 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️  감지기 활성화 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ 감지기 활성화 응답 처리 중 오류: {e}')

    def deactivate_detector_response_callback(self, future):  # DeactivateDetector 응답 콜백
        """DeactivateDetector 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 감지기 비활성화 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️  감지기 비활성화 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ 감지기 비활성화 응답 처리 중 오류: {e}')

    def activate_qr_scanner(self, robot_id):  # Vision Manager에게 QR Scanner 활성화 요청
        """Vision Manager에게 ActivateQRScanner 서비스 요청을 보내는 메서드"""
        # Vision Manager 서비스가 준비될 때까지 대기
        if not self.activate_qr_scanner_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Vision Manager 서비스를 찾을 수 없음 (activate_qr_scanner)')
            return False
        
        # ActivateQRScanner 요청 생성
        request = ActivateQRScanner.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'👁️ Vision Manager에게 QR Scanner 활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.activate_qr_scanner_client.call_async(request)
            future.add_done_callback(self.activate_qr_scanner_response_callback)
            self.get_logger().info(f'📤 QR Scanner 활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ Vision Manager 통신 중 오류: {e}')
            return False

    def deactivate_qr_scanner(self, robot_id):  # Vision Manager에게 QR Scanner 비활성화 요청
        """Vision Manager에게 DeactivateQRScanner 서비스 요청을 보내는 메서드"""
        # Vision Manager 서비스가 준비될 때까지 대기
        if not self.deactivate_qr_scanner_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ Vision Manager 서비스를 찾을 수 없음 (deactivate_qr_scanner)')
            return False
        
        # DeactivateQRScanner 요청 생성
        request = DeactivateQRScanner.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'👁️ Vision Manager에게 QR Scanner 비활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.deactivate_qr_scanner_client.call_async(request)
            future.add_done_callback(self.deactivate_qr_scanner_response_callback)
            self.get_logger().info(f'📤 QR Scanner 비활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ Vision Manager 통신 중 오류: {e}')
            return False

    def cancel_navigation(self):  # 네비게이션 취소 요청
        """네비게이션을 취소하는 메서드"""
        # CancelNavigation 서비스가 준비될 때까지 대기
        if not self.cancel_navigation_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ CancelNavigation 서비스를 찾을 수 없음')
            return False
        
        # CancelNavigation 요청 생성 (요청은 비어있음)
        request = CancelNavigation.Request()
        
        self.get_logger().info(f'⏹️ 네비게이션 취소 요청 전송...')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.cancel_navigation_client.call_async(request)
            future.add_done_callback(self.cancel_navigation_response_callback)
            self.get_logger().info(f'📤 네비게이션 취소 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ CancelNavigation 통신 중 오류: {e}')
            return False

    def cancel_navigation_response_callback(self, future):  # CancelNavigation 응답 콜백
        """CancelNavigation 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 네비게이션 취소 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️ 네비게이션 취소 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ 네비게이션 취소 응답 처리 중 오류: {e}')

    def detection_timer_callback(self, msg):  # DetectionTimer 메시지 수신 콜백
        """DetectionTimer 메시지를 받았을 때 호출되는 콜백 함수"""
        try:
            # 단순히 DetectionTimer 메시지 수신만 로그로 표시
            self.get_logger().info(f'⏰ [DetectionTimer] 수신됨! robot_id={msg.robot_id}, command={msg.command}')
            
            # 일반적인 카운터 명령인 경우
            try:
                counter_value = int(msg.command)
                self.get_logger().info(f'📊 [DetectionTimer] 카운터: {counter_value}초 (robot: {msg.robot_id})')
                
                # 10초 초과 시 특별 처리
                if counter_value >= 10:
                    self.get_logger().warn(f'🚨 [DetectionTimer] 10초 초과! 특별 처리 시작 (robot: {msg.robot_id})')
                    
                    # 현재 활성 task 확인
                    if self.tasks and len(self.tasks) > 0:
                        current_task = self.tasks[0]  # 첫 번째 활성 task
                        
                        # 새로운 통합 시스템으로 timer 이벤트 처리
                        if counter_value == 10:
                            self.process_task_stage_logic(current_task, current_task.stage, 'timer_10s')
                        elif counter_value >= 30:
                            self.process_task_stage_logic(current_task, current_task.stage, 'timer_30s')
                    
                    else:
                        # 활성 task가 없는 경우
                        self.get_logger().warn(f'⚠️ [DetectionTimer] 10초 초과했지만 활성 task가 없음')
                
                # 5초일 때는 일반 경고만 (기존 로직 유지)
                elif counter_value == 5:
                    self.get_logger().warn(f'⚠️ [DetectionTimer] 5초 경과! 주의가 필요합니다. (robot: {msg.robot_id})')
                
            except ValueError:
                # 숫자가 아닌 다른 명령인 경우
                self.get_logger().info(f'📝 [DetectionTimer] 명령: {msg.command} (robot: {msg.robot_id})')
            
        except Exception as e:  # 예외 발생 시 처리
            self.get_logger().error(f'❌ [DetectionTimer] 처리 중 오류: {e}')  # 에러 로그

    def weight_callback(self, msg):  # 무게 데이터 수신 콜백
        """무게 데이터를 받았을 때 호출되는 콜백 함수"""
        self.current_weight = msg.data  # 무게 데이터 저장
        self.last_weight_update = time.time()  # 마지막 무게 업데이트 시간 갱신
        # self.get_logger().info(f'⚖️ [libo_a Weight] 실시간 수신: {self.current_weight:.1f}g ({self.current_weight/1000.0:.3f}kg)')  # 실시간 무게 데이터 표시
    
    def get_current_weight(self):  # 현재 무게 반환
        """현재 무게를 반환하는 메서드 (g 단위)"""
        return self.current_weight
    
    def get_current_weight_kg(self):  # 현재 무게를 kg 단위로 반환
        """현재 무게를 kg 단위로 반환하는 메서드"""
        return self.current_weight / 1000.0
    
    def is_weight_data_recent(self, timeout_seconds=5):  # 최근 무게 데이터인지 확인
        """최근 timeout_seconds 이내에 무게 데이터가 업데이트되었는지 확인"""
        if self.last_weight_update is None:
            return False
        return (time.time() - self.last_weight_update) <= timeout_seconds

    def manage_robot_states(self):  # 로봇 상태 관리
        """로봇들의 상태를 주기적으로 관리하는 메서드"""
        for robot_id, robot in self.robots.items():
            # 비상 상황 체크 (EMERGENCY 상태가 아닐 때만)
            if robot.current_state != RobotState.EMERGENCY:
                if self.check_emergency_conditions(robot_id):
                    continue  # 비상 상황 발생 시 다른 처리는 중단
            
            # 기존 상태 관리 로직
            self.process_robot_state(robot)
    
    def process_robot_state(self, robot):  # 개별 로봇 상태 처리
        """개별 로봇의 상태에 따른 처리 로직"""
        current_time = time.time()
        state_duration = current_time - robot.state_start_time  # 현재 상태 지속 시간
        
        if robot.current_state == RobotState.INIT:
            # INIT 상태에서 5초 후 CHARGING으로 변경
            if state_duration >= 5.0:
                old_state, new_state = robot.change_state(RobotState.CHARGING)
                self.get_logger().info(f'🔋 로봇 <{robot.robot_id}> 상태 변경: {old_state.value} → {new_state.value} (5초 경과)')
                
                # 초기화 완료 음성 명령 발행
                self.get_logger().info(f'🗣️ 로봇 초기화 완료 음성 명령 발행: common.initialized')
                if self.send_voice_command(robot.robot_id, 'common', 'initialized'):
                    self.get_logger().info(f'✅ 초기화 완료 음성 명령 발행 완료')
                else:
                    self.get_logger().warning(f'⚠️ 초기화 완료 음성 명령 발행 실패')
        
        elif robot.current_state == RobotState.CHARGING:
            # CHARGING 상태에서 10초 후 STANDBY로 변경 (임시)
            if state_duration >= 10.0:
                old_state, new_state = robot.change_state(RobotState.STANDBY)
                self.get_logger().info(f'⚡ 로봇 <{robot.robot_id}> 상태 변경: {old_state.value} → {new_state.value} (10초 경과)')
                
                # 배터리 충분 음성 명령 발행
                self.get_logger().info(f'🗣️ 배터리 충분 음성 명령 발행: common.battery_sufficient')
                if self.send_voice_command(robot.robot_id, 'common', 'battery_sufficient'):
                    self.get_logger().info(f'✅ 배터리 충분 음성 명령 발행 완료')
                else:
                    self.get_logger().warning(f'⚠️ 배터리 충분 음성 명령 발행 실패')
        
        # ESCORT, DELIVERY, ASSIST 상태는 Task 완료 시까지 자동 변경하지 않음
        # 이 상태들은 advance_task_stage에서만 변경됨
        elif robot.current_state in [RobotState.ESCORT, RobotState.DELIVERY, RobotState.ASSIST]:
            # Task 관련 상태는 자동 변경하지 않음 - Task 완료 시까지 유지
            pass
        
        # 다른 상태들은 나중에 추가 예정

    def send_voice_command(self, robot_id, category, action):  # VoiceCommand 메시지 발행
        """VoiceCommand 메시지를 발행하는 메서드"""
        try:
            # 유효한 카테고리와 액션인지 확인
            if category not in VOICE_COMMANDS:
                self.get_logger().error(f'❌ [VoiceCommand] 유효하지 않은 카테고리: {category}')
                return False
                
            if action not in VOICE_COMMANDS[category]:
                self.get_logger().error(f'❌ [VoiceCommand] {category} 카테고리에 {action} 액션이 없습니다')
                return False
            
            # VoiceCommand 메시지 생성
            voice_msg = VoiceCommand()
            voice_msg.robot_id = robot_id
            voice_msg.category = category
            voice_msg.action = action
            
            # 메시지 발행
            self.voice_command_publisher.publish(voice_msg)
            
            # 로그 출력
            self.get_logger().info(f'🗣️ [VoiceCommand] 발행: robot_id={robot_id}, category={category}, action={action}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ [VoiceCommand] 발행 중 오류: {e}')
            return False

    def send_voice_command_by_task_type(self, robot_id, task_type, action):  # Task 타입에 따른 VoiceCommand 발행
        """Task 타입에 따라 적절한 카테고리로 VoiceCommand를 발행하는 메서드"""
        try:
            # Task 타입을 카테고리로 매핑
            task_to_category = {
                'escort': 'escort',
                'delivery': 'delivery', 
                'assist': 'assist'
            }
            
            if task_type not in task_to_category:
                self.get_logger().error(f'❌ [VoiceCommand] 지원하지 않는 Task 타입: {task_type}')
                return False
            
            category = task_to_category[task_type]
            return self.send_voice_command(robot_id, category, action)
            
        except Exception as e:
            self.get_logger().error(f'❌ [VoiceCommand] Task 타입 기반 발행 중 오류: {e}')
            return False

    def send_led_command(self, emotion):
        """감정에 따라 LED 색상 제어"""
        try:
            msg = String()
            msg.data = emotion  # "기쁨", "슬픔", "화남"
            self.led_publisher.publish(msg)
            self.get_logger().info(f'🎨 [LED] 명령 발행 성공: {emotion}')
            return True
        except Exception as e:
            self.get_logger().warn(f'⚠️ [LED] 명령 발행 실패: {emotion} (오류: {e}) - 무시하고 계속 진행')
            return False

    def send_expression_command(self, robot_id, robot_status):
        """로봇 ID와 상태에 따라 Expression 메시지 발행"""
        try:
            msg = Expression()
            msg.robot_id = robot_id  # "libo_a", "libo_b"
            msg.robot_status = robot_status  # "escort", "assist", "delivery", "기쁨", "슬픔", "화남"
            self.expression_publisher.publish(msg)
            self.get_logger().info(f'😊 [Expression] 명령 발행 성공: {robot_id} - {robot_status}')
            return True
        except Exception as e:
            self.get_logger().warn(f'⚠️ [Expression] 명령 발행 실패: {robot_id} - {robot_status} (오류: {e}) - 무시하고 계속 진행')
            return False

    def process_task_stage_logic(self, task, stage, event_type):
        """task 타입별 stage 로직을 처리하는 통합 메서드"""
        if task.task_type in self.task_stage_logic:
            if stage in self.task_stage_logic[task.task_type]:
                if event_type in self.task_stage_logic[task.task_type][stage]:
                    self.get_logger().info(f'🔄 [{task.task_type}] Stage {stage} - {event_type} 이벤트 처리 시작')
                    for action in self.task_stage_logic[task.task_type][stage][event_type]:
                        self.execute_action(task, action)
                    self.get_logger().info(f'✅ [{task.task_type}] Stage {stage} - {event_type} 이벤트 처리 완료')
                else:
                    self.get_logger().debug(f'📝 [{task.task_type}] Stage {stage}에 {event_type} 이벤트 없음')
            else:
                self.get_logger().debug(f'📝 [{task.task_type}] Stage {stage} 로직 정의 없음')
        else:
            self.get_logger().debug(f'📝 Task 타입 {task.task_type} 로직 정의 없음')

    def execute_action(self, task, action):
        """단순한 액션 실행 메서드"""
        action_type = action.get('action')
        
        # 핵심 액션들 (자주 사용되는 것들)
        if action_type == 'voice':
            command = action.get('command')
            self.send_voice_command_by_task_type(task.robot_id, task.task_type, command)
            
        elif action_type == 'led':
            emotion = action.get('emotion')
            self.send_led_command(emotion)
            
        elif action_type == 'expression':
            robot_id = action.get('robot_id')
            status = action.get('status')
            # robot_id가 'robot_id' 문자열이면 실제 task의 robot_id 사용
            if robot_id == 'robot_id':
                robot_id = task.robot_id
            self.send_expression_command(robot_id, status)
            
        elif action_type == 'navigate':
            target = action.get('target')
            if target == 'call_location':
                x, y = LOCATION_COORDINATES[task.call_location]
            elif target == 'goal_location':
                x, y = LOCATION_COORDINATES[task.goal_location]
            elif target == 'base':
                x, y = LOCATION_COORDINATES['Base']
            elif target == 'admin_desk':
                x, y = LOCATION_COORDINATES['admin_desk']  # admin PC 좌표
            self.send_goal_to_navigator(x, y)
            
        # 특수한 액션들 (자주 사용되지 않는 것들)
        elif action_type == 'activate_detector':
            self.activate_detector(task.robot_id)
            
        elif action_type == 'deactivate_detector':
            self.deactivate_detector(task.robot_id)
            
        elif action_type == 'activate_qr_scanner':
            self.activate_qr_scanner(task.robot_id)
            
        elif action_type == 'deactivate_qr_scanner':
            self.deactivate_qr_scanner(task.robot_id)
            
        elif action_type == 'activate_tracker':
            self.activate_tracker(task.robot_id)
            
        elif action_type == 'activate_talker':
            self.activate_talker(task.robot_id)
            
        elif action_type == 'deactivate_talker':
            self.deactivate_talker(task.robot_id)
            
        elif action_type == 'deactivate_tracker':
            self.deactivate_tracker(task.robot_id)
            
        elif action_type == 'cancel_navigation':
            self.cancel_navigation()
            
        elif action_type == 'force_stage':
            target_stage = action.get('target')
            task.stage = target_stage
            self.get_logger().warn(f'🔄 [{task.task_type}] 강제 Stage 변경: {target_stage}')
            # 강제 stage 변경 후 해당 stage의 stage_start 이벤트 처리
            self.process_task_stage_logic(task, target_stage, 'stage_start')
            
        elif action_type == 'advance_stage':
            # advance_stage 메서드 호출 (기존 로직 재사용)
            self.advance_stage()
        else:
            self.get_logger().warning(f'⚠️ 알 수 없는 액션 타입: {action_type}')

    def activate_qr_scanner_response_callback(self, future):  # ActivateQRScanner 응답 콜백
        """ActivateQRScanner 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ QR Scanner 활성화 성공: {response.message}')
                
                # QR Scanner 활성화 성공을 이벤트로 발행
                if self.tasks and len(self.tasks) > 0:
                    current_task = self.tasks[0]
                    self.process_task_stage_logic(current_task, current_task.stage, 'qr_scanner_activated')
                
            else:
                self.get_logger().warning(f'⚠️  QR Scanner 활성화 실패: {response.message}')
                
                # QR Scanner 활성화 실패를 이벤트로 발행
                if self.tasks and len(self.tasks) > 0:
                    current_task = self.tasks[0]
                    self.process_task_stage_logic(current_task, current_task.stage, 'qr_scanner_failed')
                
        except Exception as e:
            self.get_logger().error(f'❌ QR Scanner 활성화 응답 처리 중 오류: {e}')

    def deactivate_qr_scanner_response_callback(self, future):  # DeactivateQRScanner 응답 콜백
        """DeactivateQRScanner 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ QR Scanner 비활성화 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️  QR Scanner 비활성화 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ QR Scanner 비활성화 응답 처리 중 오류: {e}')

    def end_task_callback(self, request, response):  # EndTask 서비스 콜백
        """EndTask 서비스 콜백"""
        self.get_logger().info(f'📥 EndTask 요청 받음!')
        self.get_logger().info(f'   - 로봇 ID: {request.robot_id}')
        self.get_logger().info(f'   - Task Type: {request.task_type}')
        
        # 해당 로봇의 활성 작업 찾기
        active_task = None
        for task in self.tasks:
            if task.robot_id == request.robot_id and task.task_type == request.task_type:
                active_task = task
                break
        
        if active_task:
            # task_stage_logic에서 end_task 이벤트 처리
            self.process_task_stage_logic(active_task, active_task.stage, 'end_task')
            response.success = True
            response.message = f"EndTask 이벤트 처리 완료: {request.robot_id} - {request.task_type}"
        else:
            response.success = False
            response.message = f"로봇 <{request.robot_id}>의 {request.task_type} 작업을 찾을 수 없습니다"
        
        return response

    def robot_qr_check_callback(self, request, response):  # RobotQRCheck 서비스 콜백
        """RobotQRCheck 서비스 콜백"""
        self.get_logger().info(f'📥 RobotQRCheck 요청 받음!')
        self.get_logger().info(f'   - 로봇 ID: {request.robot_id}')
        self.get_logger().info(f'   - 관리자 이름: {request.admin_name}')
        
        # 현재 활성 task가 있는지 확인
        if not self.tasks or len(self.tasks) == 0:
            response.success = False
            response.message = f"활성 task가 없어서 QR Check를 처리할 수 없습니다"
            self.get_logger().warning(f'❌ QR Check 실패: 활성 task 없음')
            return response
        
        current_task = self.tasks[0]
        
        # 로봇 ID 일치 여부 확인
        if current_task.robot_id != request.robot_id:
            response.success = False
            response.message = f"로봇 ID 불일치: 현재 task는 {current_task.robot_id}이지만 요청은 {request.robot_id}입니다"
            self.get_logger().warning(f'❌ QR Check 실패: 로봇 ID 불일치 (현재: {current_task.robot_id}, 요청: {request.robot_id})')
            return response
        
        # 관리자 이름 유효성 확인
        if request.admin_name not in ADMIN_NAMES:
            response.success = False
            response.message = f"유효하지 않은 관리자 이름: {request.admin_name} (등록된 관리자: {', '.join(ADMIN_NAMES)})"
            self.get_logger().warning(f'❌ QR Check 실패: 유효하지 않은 관리자 이름 ({request.admin_name})')
            return response
        
        # 모든 검증 통과 - QR Check 성공
        response.success = True
        response.message = f"Robot QR Check 완료: {request.robot_id} - {request.admin_name}"
        
        # QR Check 완료 후 qr_check_completed 이벤트 발생
        self.get_logger().info(f'✅ QR Check 완료! qr_check_completed 이벤트 발생')
        self.process_task_stage_logic(current_task, current_task.stage, 'qr_check_completed')
        
        return response

    def activate_talker(self, robot_id):  # Talker 활성화 요청
        """Talker를 활성화하는 메서드"""
        # ActivateTalker 서비스가 준비될 때까지 대기
        if not self.activate_talker_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ ActivateTalker 서비스를 찾을 수 없음')
            return False
        
        # ActivateTalker 요청 생성
        request = ActivateTalker.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'🗣️ Talker 활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.activate_talker_client.call_async(request)
            future.add_done_callback(self.activate_talker_response_callback)
            self.get_logger().info(f'📤 Talker 활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ ActivateTalker 통신 중 오류: {e}')
            return False

    def deactivate_talker(self, robot_id):  # Talker 비활성화 요청
        """Talker를 비활성화하는 메서드"""
        # DeactivateTalker 서비스가 준비될 때까지 대기
        if not self.deactivate_talker_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ DeactivateTalker 서비스를 찾을 수 없음')
            return False
        
        # DeactivateTalker 요청 생성
        request = DeactivateTalker.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'🗣️ Talker 비활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.deactivate_talker_client.call_async(request)
            future.add_done_callback(self.deactivate_talker_response_callback)
            self.get_logger().info(f'📤 Talker 비활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ DeactivateTalker 통신 중 오류: {e}')
            return False

    def activate_tracker(self, robot_id):  # Tracker 활성화 요청
        """Tracker를 활성화하는 메서드"""
        # ActivateTracker 서비스가 준비될 때까지 대기
        if not self.activate_tracker_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ ActivateTracker 서비스를 찾을 수 없음')
            return False
        
        # ActivateTracker 요청 생성
        request = ActivateTracker.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'🎯 Tracker 활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.activate_tracker_client.call_async(request)
            future.add_done_callback(self.activate_tracker_response_callback)
            self.get_logger().info(f'📤 Tracker 활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ ActivateTracker 통신 중 오류: {e}')
            return False

    def deactivate_tracker(self, robot_id):  # Tracker 비활성화 요청
        """Tracker를 비활성화하는 메서드"""
        # DeactivateTracker 서비스가 준비될 때까지 대기
        if not self.deactivate_tracker_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ DeactivateTracker 서비스를 찾을 수 없음')
            return False
        
        # DeactivateTracker 요청 생성
        request = DeactivateTracker.Request()
        request.robot_id = robot_id  # 로봇 ID 설정
        
        self.get_logger().info(f'🎯 Tracker 비활성화 요청: {robot_id}')
        
        try:
            # 비동기 서비스 호출 (응답을 콜백으로 처리)
            future = self.deactivate_tracker_client.call_async(request)
            future.add_done_callback(self.deactivate_tracker_response_callback)
            self.get_logger().info(f'📤 Tracker 비활성화 요청 전송 완료 - 응답 대기 중...')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ DeactivateTracker 통신 중 오류: {e}')
            return False

    def activate_talker_response_callback(self, future):  # ActivateTalker 응답 콜백
        """ActivateTalker 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Talker 활성화 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️ Talker 활성화 실패: {response.message}')
                
                # Talker 활성화 실패를 이벤트로 발행
                if self.tasks and len(self.tasks) > 0:
                    current_task = self.tasks[0]
                    self.process_task_stage_logic(current_task, current_task.stage, 'talker_failed')
                
        except Exception as e:
            self.get_logger().error(f'❌ Talker 활성화 응답 처리 중 오류: {e}')
            
            # 예외 발생 시에도 실패 이벤트 발행
            if self.tasks and len(self.tasks) > 0:
                current_task = self.tasks[0]
                self.process_task_stage_logic(current_task, current_task.stage, 'talker_failed')

    def deactivate_talker_response_callback(self, future):  # DeactivateTalker 응답 콜백
        """DeactivateTalker 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Talker 비활성화 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️ Talker 비활성화 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ Talker 비활성화 응답 처리 중 오류: {e}')

    def activate_tracker_response_callback(self, future):  # ActivateTracker 응답 콜백
        """ActivateTracker 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Tracker 활성화 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️ Tracker 활성화 실패: {response.message}')
                
                # Tracker 활성화 실패를 이벤트로 발행
                if self.tasks and len(self.tasks) > 0:
                    current_task = self.tasks[0]
                    self.process_task_stage_logic(current_task, current_task.stage, 'tracker_failed')
                
        except Exception as e:
            self.get_logger().error(f'❌ Tracker 활성화 응답 처리 중 오류: {e}')
            
            # 예외 발생 시에도 실패 이벤트 발행
            if self.tasks and len(self.tasks) > 0:
                current_task = self.tasks[0]
                self.process_task_stage_logic(current_task, current_task.stage, 'tracker_failed')

    def deactivate_tracker_response_callback(self, future):  # DeactivateTracker 응답 콜백
        """DeactivateTracker 서비스 응답을 처리하는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Tracker 비활성화 성공: {response.message}')
            else:
                self.get_logger().warning(f'⚠️ Tracker 비활성화 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ Tracker 비활성화 응답 처리 중 오류: {e}')

    def add_goal_location_callback(self, request, response):  # AddGoalLocation 서비스 콜백
        """AddGoalLocation 서비스 요청을 처리하는 콜백"""
        try:
            robot_id = request.robot_id  # 요청에서 로봇 ID 가져오기
            goal_location = request.goal_location  # 요청에서 목표 위치 가져오기
            
            self.get_logger().info(f'🎯 목표 위치 추가 요청: 로봇 {robot_id} -> {goal_location}')
            
            # 현재 활성 작업이 있는지 확인
            if not self.tasks:
                response.success = False
                response.message = f'활성 작업이 없습니다.'
                self.get_logger().warning(f'⚠️ 활성 작업 없음: {response.message}')
                return response
            
            current_task = self.tasks[0]  # 첫 번째 작업 가져오기
            
            # 로봇 ID가 일치하는지 확인
            if current_task.robot_id != robot_id:
                response.success = False
                response.message = f'로봇 ID 불일치: 요청된 {robot_id}, 현재 작업 {current_task.robot_id}'
                self.get_logger().warning(f'⚠️ 로봇 ID 불일치: {response.message}')
                return response
            
            # 작업 타입이 delivery인지 확인
            if current_task.task_type != 'delivery':
                response.success = False
                response.message = f'작업 타입이 delivery가 아닙니다: {current_task.task_type}'
                self.get_logger().warning(f'⚠️ 작업 타입 불일치: {response.message}')
                return response
            
            # stage가 2인지 확인
            if current_task.stage != 2:
                response.success = False
                response.message = f'현재 stage가 2가 아닙니다: {current_task.stage}'
                self.get_logger().warning(f'⚠️ stage 불일치: {response.message}')
                return response
            
            # goal_location이 유효한 위치인지 확인
            if goal_location not in LOCATION_COORDINATES:
                response.success = False
                response.message = f'유효하지 않은 목표 위치입니다: {goal_location}'
                self.get_logger().warning(f'⚠️ 유효하지 않은 위치: {response.message}')
                return response
            
            # 모든 조건을 만족하면 goal_location 업데이트
            current_task.goal_location = goal_location  # 목표 위치 업데이트
            self.get_logger().info(f'✅ 목표 위치 업데이트: {robot_id} -> {goal_location}')
            
            # goal_location_updated 이벤트 발생
            self.process_task_stage_logic(current_task, current_task.stage, 'goal_location_updated')
            
            response.success = True  # 성공 응답
            response.message = f'목표 위치 {goal_location}이 로봇 {robot_id}의 delivery stage 2 작업에 추가되었습니다.'
            
            self.get_logger().info(f'✅ 목표 위치 추가 성공: {response.message}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 목표 위치 추가 처리 중 오류: {e}')
            response.success = False  # 실패 응답
            response.message = f'목표 위치 추가 실패: {str(e)}'
        
        return response

    def emergency_stop(self, robot_id, reason="비상 상황 발생"):
        """비상 정지 - 로봇을 EMERGENCY 상태로 변경하고 모든 작업 중단"""
        if robot_id in self.robots:
            # 현재 활성 task가 있다면 중단
            if self.tasks:
                current_task = self.tasks[0]
                if current_task.robot_id == robot_id:
                    self.get_logger().error(f'🚨 비상 정지! Task[{current_task.task_id}] 중단 - {reason}')
                    # 네비게이션 취소
                    self.cancel_navigation()
                    # 모든 감지기/스캐너 비활성화
                    self.deactivate_detector(robot_id)
                    self.deactivate_qr_scanner(robot_id)
                    self.deactivate_talker(robot_id)
                    self.deactivate_tracker(robot_id)
            
            # 로봇을 EMERGENCY 상태로 변경
            old_state, _ = self.robots[robot_id].change_state(RobotState.EMERGENCY)
            self.get_logger().error(f' 로봇 <{robot_id}> 비상 정지: {old_state.value} → EMERGENCY ({reason})')
            
            # 비상 상황 음성 알림
            self.send_voice_command(robot_id, 'common', 'emergency_stop')
            
            return True
        else:
            self.get_logger().error(f'❌ 비상 정지 실패: 로봇 <{robot_id}> 찾을 수 없음')
            return False
    
    def emergency_recovery(self, robot_id):
        """비상 상황 복구 - 로봇을 STANDBY 상태로 복구"""
        if robot_id in self.robots:
            if self.robots[robot_id].current_state == RobotState.EMERGENCY:
                old_state, _ = self.robots[robot_id].change_state(RobotState.STANDBY)
                self.get_logger().info(f'✅ 로봇 <{robot_id}> 비상 상황 복구: {old_state.value} → STANDBY')
                
                # 복구 완료 음성 알림
                self.send_voice_command(robot_id, 'common', 'emergency_recovery')
                
                return True
            else:
                self.get_logger().warning(f'⚠️ 복구 실패: 로봇 <{robot_id}>이 EMERGENCY 상태가 아님')
                return False
        else:
            self.get_logger().error(f'❌ 복구 실패: 로봇 <{robot_id}> 찾을 수 없음')
            return False
    
    def check_emergency_conditions(self, robot_id):
        """비상 상황 조건 체크 (예: 배터리 부족, 통신 오류 등)"""
        if robot_id not in self.robots:
            return False
        
        robot = self.robots[robot_id]
        
        # 배터리 부족 체크 (예: 10% 이하)
        if hasattr(robot, 'battery') and robot.battery < 10:
            self.emergency_stop(robot_id, "배터리 부족")
            return True
        
        # 하트비트 타임아웃 체크 (예: 10초 이상)
        if not robot.check_timeout(timeout_seconds=10):
            self.emergency_stop(robot_id, "통신 오류")
            return True
        
        # 기타 비상 상황 조건들 추가 가능
        # - 센서 오류
        # - 모터 오류
        # - 장애물 감지
        # - 기울기 과다 등
        
        return False

    def handle_navigation_canceled(self, task):
        """네비게이션 취소 시 공통 처리 로직"""
        self.get_logger().info(f'⏹️ 네비게이션 취소 처리: Task[{task.task_id}] Stage {task.stage}')
        
        # 네비게이션 취소 음성 알림
        self.send_voice_command_by_task_type(task.robot_id, task.task_type, 'navigation_canceled')
        
        # Stage 3으로 강제 진행 (복귀)
        if task.stage < 3:
            old_stage = task.stage
            task.stage = 3
            self.get_logger().info(f' Task[{task.task_id}] 강제 Stage 변경: {old_stage} → 3')
            
            # Stage 3 시작 로직 실행
            self.process_task_stage_logic(task, 3, 'stage_start')
        else:
            self.get_logger().warning(f'⚠️ Task[{task.task_id}] 이미 Stage {task.stage} - 강제 변경 불필요')

def main(args=None):  # ROS2 노드 실행 및 종료 처리
    rclpy.init(args=args)
    
    task_manager = TaskManager()
    
    try:
        rclpy.spin(task_manager)
    except KeyboardInterrupt:
        pass
    finally:
        task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
