#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
from libo_interfaces.srv import SetGoal  # SetGoal 서비스 추가
from libo_interfaces.srv import NavigationResult  # NavigationResult 서비스 추가
from libo_interfaces.msg import Heartbeat  # Heartbeat 메시지 추가
from libo_interfaces.msg import OverallStatus  # OverallStatus 메시지 추가
from libo_interfaces.msg import TaskStatus  # TaskStatus 메시지 추가
import time  # 시간 관련 기능
import uuid  # 고유 ID 생성
import random  # 랜덤 좌표 생성용

# 좌표 매핑 딕셔너리 (A1~E9까지 총 45개 좌표)
LOCATION_COORDINATES = {
    # A열 좌표들
    'A1': (1.2, 3.4), 'A2': (2.1, 4.5), 'A3': (3.3, 2.8), 'A4': (4.7, 1.9), 'A5': (5.2, 6.1),
    'A6': (6.8, 3.7), 'A7': (7.4, 5.2), 'A8': (8.1, 2.3), 'A9': (9.5, 4.8),
    
    # B열 좌표들
    'B1': (1.8, 7.2), 'B2': (2.9, 8.4), 'B3': (3.6, 6.9), 'B4': (4.2, 9.1), 'B5': (5.8, 7.6),
    'B6': (6.3, 8.9), 'B7': (7.1, 6.4), 'B8': (8.7, 9.3), 'B9': (9.2, 7.8),
    
    # C열 좌표들
    'C1': (1.5, 1.2), 'C2': (2.4, 2.6), 'C3': (3.8, 1.8), 'C4': (4.5, 3.2), 'C5': (5.1, 1.5),
    'C6': (6.2, 2.9), 'C7': (7.6, 1.3), 'C8': (8.3, 3.7), 'C9': (9.8, 2.1),
    
    # D열 좌표들
    'D1': (1.9, 5.8), 'D2': (2.7, 6.3), 'D3': (3.4, 5.1), 'D4': (4.8, 6.7), 'D5': (5.3, 5.4),
    'D6': (6.1, 7.2), 'D7': (7.9, 5.9), 'D8': (8.4, 6.8), 'D9': (9.1, 5.6),
    
    # E열 좌표들
    'E1': (1.3, 8.7), 'E2': (2.6, 9.2), 'E3': (3.9, 8.1), 'E4': (4.1, 9.8), 'E5': (5.7, 8.3),
    'E6': (6.5, 9.5), 'E7': (7.2, 8.6), 'E8': (8.9, 9.7), 'E9': (9.4, 8.4),
    
    # Base 좌표 (스테이지 3 완료 후 돌아갈 위치) - E3로 고정
    'Base': (3.9, 8.1)  # E3 좌표와 동일
}

class Robot:  # 로봇 정보를 담는 클래스
    def __init__(self, robot_id):  # Robot 객체 초기화
        self.robot_id = robot_id  # 로봇 ID 저장
        self.last_heartbeat_time = time.time()  # 마지막 하트비트 수신 시간
        self.is_available = True  # 사용 가능 상태 (기본값: 사용 가능)
    
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
    
    def get_status_info(self):  # 로봇 상태 정보 반환
        """로봇의 현재 상태 정보를 문자열로 반환"""
        available_status = "사용가능" if self.is_available else "사용중"
        return f"Robot[{self.robot_id}] - 활성 | {available_status}"

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
        
        self.get_logger().info('🎯 Task Manager 시작됨 - task_request 서비스 대기 중...')
        self.get_logger().info('💓 Heartbeat 구독 시작됨 - heartbeat 토픽 모니터링 중...')
        self.get_logger().info('📡 OverallStatus 발행 시작됨 - robot_status 토픽으로 1초마다 발행...')
        self.get_logger().info('📋 TaskStatus 발행 시작됨 - task_status 토픽으로 1초마다 발행...')  # TaskStatus 로그 추가
        self.get_logger().info('🧭 Navigator 클라이언트 준비됨 - set_navigation_goal 서비스 연결...')  # Navigator 클라이언트 로그 추가
        self.get_logger().info('📍 NavigationResult 서비스 시작됨 - navigation_result 서비스 대기 중...')  # NavigationResult 서버 로그 추가
    
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
            status_msg.is_available = robot.is_available  # 실제 로봇의 사용 가능 상태 사용
            status_msg.battery = 255  # 기본값: 알 수 없음 (255로 표시)
            status_msg.book_weight = 0.0  # 기본값: 무게 없음
            status_msg.position_x = 0.0  # 기본값: 위치 알 수 없음
            status_msg.position_y = 0.0  # 기본값: 위치 알 수 없음
            status_msg.position_yaw = 0.0  # 기본값: 방향 알 수 없음
            
            self.status_publisher.publish(status_msg)  # 메시지 발행
            self.get_logger().debug(f'📡 로봇 상태 발행: {robot_id} → {"사용가능" if robot.is_available else "사용중"}')
    
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
        
        # 새로운 Task 객체 생성
        new_task = Task(request.robot_id, request.task_type, request.call_location, request.goal_location)  # Task 객체 생성
        self.tasks.append(new_task)  # 작업 목록에 추가
        
        self.get_logger().info(f'✅ 새로운 작업 생성됨: {new_task.get_info()}')  # 생성된 작업 정보 출력
        
        # Task 생성 후 자동으로 로봇을 사용중으로 설정
        if self.set_robot_unavailable_for_task(request.robot_id):
            self.get_logger().info(f'🔒 로봇 <{request.robot_id}> 자동으로 사용중 상태로 변경됨')
        else:
            self.get_logger().warning(f'⚠️  로봇 <{request.robot_id}> 상태 변경 실패 - 로봇이 존재하지 않음')
        
        # 새로운 Task의 첫 번째 스테이지 좌표 전송
        self.get_logger().info(f'🚀 새로운 Task의 Stage 1 좌표 전송 시작...')
        if self.send_coordinate_for_stage(new_task):
            self.get_logger().info(f'✅ Stage 1 좌표 전송 완료')
        else:
            self.get_logger().error(f'❌ Stage 1 좌표 전송 실패')
        
        # Navigator에게 더미 좌표 전송 테스트 (기존 코드 제거)
        # self.get_logger().info(f'🧭 Navigator 통신 테스트 시작...')
        # navigator_success = self.send_goal_to_navigator(1.5, 2.3)  # 더미 좌표 (1.5, 2.3)
        # if navigator_success:
        #     self.get_logger().info(f'📤 Navigator 요청 전송됨 - 응답은 비동기로 처리됩니다')
        # else:
        #     self.get_logger().warning(f'⚠️  Navigator 요청 전송 실패')
        
        # 응답 설정
        response.success = True
        response.message = f"Task request 잘 받았음! Task ID: {new_task.task_id}"
        
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
            
            # task 목록에서 제거
            self.tasks.remove(current_task)
            
            self.get_logger().info(f'🏁 Task[{current_task.task_id}] 완료 및 제거됨!')
            self.get_logger().info(f'📊 현재 활성 task 수: {len(self.tasks)}개')
        else:
            # Stage 3 이하일 때 현재 상태 로그
            stage_desc = {1: "시작", 2: "진행중", 3: "완료직전"}.get(current_task.stage, f"Stage {current_task.stage}")
            self.get_logger().info(f'📍 현재 상태: {stage_icons.get(current_task.stage, "⚪")} Stage {current_task.stage} ({stage_desc})')
            
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
