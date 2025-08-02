#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
from libo_interfaces.srv import SetGoal  # SetGoal 서비스 추가
from libo_interfaces.srv import NavigationResult  # NavigationResult 서비스 추가
from libo_interfaces.srv import ActivateDetector  # ActivateDetector 서비스 추가
from libo_interfaces.srv import DeactivateDetector  # DeactivateDetector 서비스 추가
from libo_interfaces.srv import CancelNavigation  # CancelNavigation 서비스 추가
from libo_interfaces.msg import Heartbeat  # Heartbeat 메시지 추가
from libo_interfaces.msg import OverallStatus  # OverallStatus 메시지 추가
from libo_interfaces.msg import TaskStatus  # TaskStatus 메시지 추가
from libo_interfaces.msg import DetectionTimer  # DetectionTimer 메시지 추가
from libo_interfaces.msg import VoiceCommand  # VoiceCommand 메시지 추가
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
    'Base': (0.05, -0.34)  # E3 좌표와 동일
}

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
        "arrived_base": "Base에 도착했습니다."
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
        
        # CancelNavigation 서비스 클라이언트 생성
        self.cancel_navigation_client = self.create_client(CancelNavigation, 'cancel_navigation')
        
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
        
        # VoiceCommand 토픽 퍼블리셔 생성
        self.voice_command_publisher = self.create_publisher(VoiceCommand, 'voice_command', 10)
        
        # 작업 목록을 저장할 리스트
        self.tasks = []  # 생성된 작업들을 저장할 리스트
        
        # 로봇 목록을 저장할 딕셔너리 (robot_id를 키로 사용)
        self.robots = {}  # 로봇들을 저장할 딕셔너리
        
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
        
        self.get_logger().info('🎯 Task Manager 시작됨 - task_request 서비스 대기 중...')
        self.get_logger().info('💓 Heartbeat 구독 시작됨 - heartbeat 토픽 모니터링 중...')
        self.get_logger().info('📡 OverallStatus 발행 시작됨 - robot_status 토픽으로 1초마다 발행...')
        self.get_logger().info('📋 TaskStatus 발행 시작됨 - task_status 토픽으로 1초마다 발행...')  # TaskStatus 로그 추가
        self.get_logger().info('🧭 Navigator 클라이언트 준비됨 - set_navigation_goal 서비스 연결...')  # Navigator 클라이언트 로그 추가
        self.get_logger().info('📍 NavigationResult 서비스 시작됨 - navigation_result 서비스 대기 중...')  # NavigationResult 서버 로그 추가
        self.get_logger().info('👁️ ActivateDetector 클라이언트 준비됨 - activate_detector 서비스 연결...')
        self.get_logger().info('👁️ DeactivateDetector 클라이언트 준비됨 - deactivate_detector 서비스 연결...')
        self.get_logger().info('⏹️ CancelNavigation 클라이언트 준비됨 - cancel_navigation 서비스 연결...')
        self.get_logger().info('⏰ DetectionTimer 구독 시작됨 - detection_timer 토픽 모니터링 중...')
        self.get_logger().info('🗣️ VoiceCommand 퍼블리셔 준비됨 - voice_command 토픽으로 이벤트 기반 발행...')
    
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
            
            # 무게 시뮬레이션 (작업 상태일 때만 무게 있음)
            if robot.current_state in [RobotState.ESCORT, RobotState.DELIVERY, RobotState.ASSIST]:
                status_msg.book_weight = 2.5  # 작업 중일 때 2.5kg
            else:
                status_msg.book_weight = 0.0  # 작업 중이 아닐 때 무게 없음
            
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
        
        # 새로운 Task의 첫 번째 스테이지 좌표 전송
        self.get_logger().info(f'🚀 새로운 Task의 Stage 1 좌표 전송 시작...')
        
        # Task 시작 시 출발 음성 명령 발행
        self.get_logger().info(f'🗣️ Task 시작 음성 명령 발행: {request.task_type}.depart_base')
        if self.send_voice_command_by_task_type(selected_robot_id, request.task_type, 'depart_base'):
            self.get_logger().info(f'✅ 출발 음성 명령 발행 완료')
        else:
            self.get_logger().warning(f'⚠️ 출발 음성 명령 발행 실패')
        
        if self.send_coordinate_for_stage(new_task):
            self.get_logger().info(f'✅ Stage 1 좌표 전송 완료')
        else:
            self.get_logger().error(f'❌ Stage 1 좌표 전송 실패')
        
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
            
            # Stage 3 시작 시 복귀 음성 명령 발행
            self.get_logger().info(f'🗣️ Stage 3 시작 - 복귀 음성 명령 발행: {task.task_type}.return')
            if self.send_voice_command_by_task_type(task.robot_id, task.task_type, 'return'):
                self.get_logger().info(f'✅ 복귀 음성 명령 발행 완료')
            else:
                self.get_logger().warning(f'⚠️ 복귀 음성 명령 발행 실패')
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
            # 현재는 단순히 로그만 출력 (나중에 task 상태 업데이트 등 추가 예정)
            if request.result == "SUCCEEDED":
                self.get_logger().info(f'✅ 네비게이션 성공!')
                # SUCCEEDED를 받으면 현재 활성 task의 stage 증가
                self.advance_task_stage()
            elif request.result == "FAILED":
                self.get_logger().warning(f'❌ 네비게이션 실패!')
            elif request.result == "CANCELED":
                self.get_logger().info(f'⏹️  네비게이션 취소됨!')
            else:
                self.get_logger().warning(f'⚠️  알 수 없는 결과: {request.result}')
            
            # 성공 응답
            response.success = True
            response.message = f"NavigationResult 처리 완료: {request.result}"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'❌ NavigationResult 처리 중 오류: {e}')
            response.success = False
            response.message = f"처리 실패: {str(e)}"
            return response
    
    def advance_task_stage(self):  # 활성 task의 stage 증가
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
            
            # Escort task의 Stage 2 시작 시점에 감지기 활성화
            if current_task.task_type == 'escort' and current_task.stage == 2:
                self.get_logger().info(f'🚶 Escort task Stage 2 시작 - 감지기 활성화 요청...')
                if self.activate_detector(current_task.robot_id):
                    self.get_logger().info(f'✅ 감지기 활성화 요청 전송 완료')
                else:
                    self.get_logger().error(f'❌ 감지기 활성화 요청 전송 실패')
            
            # 스테이지가 바뀌었으므로 해당하는 좌표를 Navigator에게 전송
            self.get_logger().info(f'🚀 새로운 스테이지에 맞는 좌표 전송 시작...')
            
            if self.send_coordinate_for_stage(current_task):
                self.get_logger().info(f'✅ 스테이지 {current_task.stage} 좌표 전송 완료')
            else:
                self.get_logger().error(f'❌ 스테이지 {current_task.stage} 좌표 전송 실패')

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
                        
                        # Escort task이고 Stage 2인 경우에만 특별 처리
                        if current_task.task_type == 'escort' and current_task.stage == 2:
                            # 10초일 때는 lost_user 음성만 발행
                            if counter_value == 10:
                                self.get_logger().warn(f'🚨 [DetectionTimer] Escort Stage 2에서 10초 경과! 사용자 분실 경고')
                                
                                # 사용자 분실 음성 명령 발행
                                self.get_logger().info(f'🗣️ [DetectionTimer] 사용자 분실 음성 명령 발행: escort.lost_user')
                                if self.send_voice_command_by_task_type(current_task.robot_id, 'escort', 'lost_user'):
                                    self.get_logger().info(f'✅ [DetectionTimer] 사용자 분실 음성 명령 발행 완료')
                                else:
                                    self.get_logger().warning(f'⚠️ [DetectionTimer] 사용자 분실 음성 명령 발행 실패')
                            
                            # 30초일 때 Stage 3으로 강제 이동
                            elif counter_value >= 30:
                                self.get_logger().warn(f'🚨 [DetectionTimer] Escort Stage 2에서 30초 초과! 자동 Stage 3 전환 시작')
                                
                                # 1. CancelNavigation 발행
                                self.get_logger().info(f'⏹️ [DetectionTimer] CancelNavigation 요청 전송...')
                                if self.cancel_navigation():
                                    self.get_logger().info(f'✅ [DetectionTimer] CancelNavigation 요청 전송 완료')
                                else:
                                    self.get_logger().error(f'❌ [DetectionTimer] CancelNavigation 요청 전송 실패')
                                
                                # 2. DeactivateDetector 발행
                                self.get_logger().info(f'👁️ [DetectionTimer] DeactivateDetector 요청 전송...')
                                if self.deactivate_detector(current_task.robot_id):
                                    self.get_logger().info(f'✅ [DetectionTimer] DeactivateDetector 요청 전송 완료')
                                else:
                                    self.get_logger().error(f'❌ [DetectionTimer] DeactivateDetector 요청 전송 실패')
                                
                                # 3. Stage 3으로 강제 이동
                                self.get_logger().warn(f'🔄 [DetectionTimer] Stage 3으로 강제 이동...')
                                current_task.stage = 3
                                self.get_logger().info(f'✅ [DetectionTimer] Stage 3으로 이동 완료')
                                
                                # 4. Stage 3 좌표 전송 (return 음성은 send_coordinate_for_stage에서 자동 발행)
                                if self.send_coordinate_for_stage(current_task):
                                    self.get_logger().info(f'✅ [DetectionTimer] Stage 3 좌표 전송 완료')
                                else:
                                    self.get_logger().error(f'❌ [DetectionTimer] Stage 3 좌표 전송 실패')
                            
                        else:
                            # Escort가 아니거나 Stage 2가 아닌 경우 일반 경고만
                            task_info = f"{current_task.task_type} (Stage {current_task.stage})" if self.tasks else "No active task"
                            self.get_logger().warn(f'⚠️ [DetectionTimer] 10초 초과했지만 Escort Stage 2가 아님: {task_info}')
                    
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

    def manage_robot_states(self):  # 로봇 상태 관리
        """각 로봇의 상태를 관리하는 메서드"""
        for robot_id, robot in self.robots.items():
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
        
        elif robot.current_state == RobotState.CHARGING:
            # CHARGING 상태에서 10초 후 STANDBY로 변경 (임시)
            if state_duration >= 10.0:
                old_state, new_state = robot.change_state(RobotState.STANDBY)
                self.get_logger().info(f'⚡ 로봇 <{robot.robot_id}> 상태 변경: {old_state.value} → {new_state.value} (10초 경과)')
        
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
