#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import json
import threading
import time
from libo_interfaces.msg import GestureResult

# 상수 정의
HAND_GESTURE_BRIDGE = 7023  # 손 제스처 브릿지 포트 (수신)

class HandGestureROS2Bridge(Node):
    """
    손 제스처 감지기(hand_gesture_detector.py)에서 UDP로 받은 제스처 데이터를
    ROS2 토픽으로 발행하는 브릿지 노드
    """
    def __init__(self):
        super().__init__('hand_gesture_ros2_bridge')
        
        # 로거 설정
        self.get_logger().info('손 제스처 ROS2 브릿지 노드 초기화 중...')
        
        # 제스처 결과 발행자
        self.gesture_publisher = self.create_publisher(
            GestureResult,
            '/gesture_result',
            10
        )
        
        # 디버그 설정
        self.show_debug_window = False  # 디버그 화면 표시하지 않음
        self.current_gesture = "none"
        self.robot_id = "libo_a"
        
        # 통계 및 모니터링
        self.last_message_time = time.time()
        self.message_count = 0
        self.error_count = 0
        self.success_count = 0
        
        # 상태 모니터링 타이머 설정 (10초마다)
        self.create_timer(10.0, self.report_status)
        
        # 제스처별 로그 스타일 정의 
        self.gesture_symbols = {
            "go": "↑",
            "back": "↓",
            "left": "←", 
            "right": "→",
            "stop": "■",
            "none": "•"
        }
        
        # UDP 수신 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # 버퍼 크기 증가 (대용량 패킷 처리)
        recv_buffer_size = 4 * 1024 * 1024  # 4MB 버퍼
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buffer_size)
            self.get_logger().info(f'수신 버퍼 크기 설정: {recv_buffer_size/1024:.0f}KB')
        except Exception as e:
            self.get_logger().warning(f'버퍼 크기 설정 실패: {str(e)}')
        
        try:
            self.sock.bind(('0.0.0.0', HAND_GESTURE_BRIDGE))
            self.get_logger().info(f'UDP 소켓이 포트 {HAND_GESTURE_BRIDGE}에 바인딩됨')
        except Exception as e:
            self.get_logger().error(f'포트 {HAND_GESTURE_BRIDGE}에 바인딩 실패: {str(e)}')
            raise
            
        self.sock.settimeout(0.5)  # 0.5초 타임아웃 설정 (종료 처리용)
        
        # 스레드 실행 상태
        self.is_running = True
        
        # 손 제스처 감지기의 데이터 형식 안내
        self.get_logger().info(f'손 제스처 감지기(hand_gesture_detector3.py)의 형식으로 수신합니다')
        self.get_logger().info(f'예상 형식: JSON + "|" + 데이터 + "\\n"')
        
        # 수신 스레드 시작
        self.receive_thread = threading.Thread(target=self.receive_and_publish)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        self.get_logger().info(f'손 제스처 ROS2 브릿지 노드 초기화 완료! (포트: {HAND_GESTURE_BRIDGE})')
        
    def report_status(self):
        """주기적으로 노드의 상태를 보고하는 메서드"""
        current_time = time.time()
        elapsed = current_time - self.last_message_time
        
        if self.message_count > 0:
            success_rate = (self.success_count / self.message_count) * 100
            self.get_logger().info(f"===== 상태 보고 =====")
            self.get_logger().info(f"처리된 메시지: {self.message_count}")
            self.get_logger().info(f"성공: {self.success_count}, 오류: {self.error_count}")
            self.get_logger().info(f"성공률: {success_rate:.1f}%")
            self.get_logger().info(f"마지막 메시지 수신: {elapsed:.1f}초 전")
            self.get_logger().info(f"현재 제스처: {self.current_gesture}")
            self.get_logger().info(f"====================")
        else:
            self.get_logger().warning(f"아직 수신된 메시지가 없습니다. (대기 시간: {elapsed:.1f}초)")
    
    def receive_and_publish(self):
        """UDP 패킷을 수신하여 ROS2 토픽으로 발행하는 스레드"""
        self.get_logger().info('UDP 패킷 수신 시작')
        
        buffer_size = 65536  # UDP 수신 버퍼 크기 (64KB)
        
        while self.is_running:
            try:
                # UDP 패킷 수신
                data, addr = self.sock.recvfrom(buffer_size)
                self.last_message_time = time.time()
                self.message_count += 1
                
                # 디버깅을 위해 수신된 데이터 크기 기록
                self.get_logger().debug(f'수신된 데이터 크기: {len(data)} 바이트')
                
                # hand_gesture_detector3.py는 데이터를 json_data + b'|' + b'' + b'\n' 형식으로 전송
                # 파이프(|) 구분자가 있는지 확인하고 처리
                split_pos = data.find(b'|')
                
                if split_pos > 0:
                    # '|' 앞부분만 JSON으로 파싱
                    json_str = data[:split_pos].decode('utf-8').strip()
                    self.get_logger().debug(f'파이프(|) 구분자 발견: 앞부분만 파싱 (길이: {len(json_str)})')
                else:
                    # 전체 데이터를 JSON으로 파싱 (하위 호환성)
                    json_str = data.decode('utf-8').strip()
                    # 줄바꿈 문자 제거 (일부 메시지 형식에 \n이 포함될 수 있음)
                    if json_str.endswith('\n'):
                        json_str = json_str[:-1]
                        self.get_logger().debug('줄바꿈 문자 제거됨')
                
                try:
                    # JSON 데이터 파싱
                    gesture_data = json.loads(json_str)
                    
                    # 필수 필드 확인
                    if 'robot_id' in gesture_data and 'gesture' in gesture_data:
                        # GestureResult 메시지 생성
                        msg = GestureResult()
                        msg.robot_id = gesture_data['robot_id']
                        msg.gesture = gesture_data['gesture']
                        
                        # ROS2 토픽 발행
                        self.gesture_publisher.publish(msg)
                        self.get_logger().info(f'제스처 발행: {msg.gesture} (로봇: {msg.robot_id})')
                        
                        # 현재 제스처 정보 업데이트 및 디버그 화면 갱신
                        self.current_gesture = msg.gesture
                        self.robot_id = msg.robot_id
                        self.update_debug_window()
                        
                        # 성공 카운트 증가
                        self.success_count += 1
                    else:
                        self.get_logger().warning('수신된 제스처 데이터 형식이 잘못되었습니다: 필수 필드 없음')
                        self.error_count += 1
                        
                except json.JSONDecodeError as je:
                    self.get_logger().error(f'JSON 파싱 오류: {str(je)}')
                    self.error_count += 1
                    
                    # 디버깅을 위해 문제가 된 데이터 출력
                    try:
                        if split_pos > 0:
                            self.get_logger().error(f'파싱 실패한 데이터: {json_str[:100]}...')
                            # 더 자세한 정보 보기
                            json_bytes = data[:split_pos]
                            self.get_logger().error(f'바이트 형식 (16진수): {json_bytes[:50].hex()}')
                        else:
                            self.get_logger().error(f'파싱 실패한 데이터: {json_str[:100]}...')
                            self.get_logger().error(f'바이트 형식 (16진수): {data[:50].hex()}')
                    except Exception as ex:
                        self.get_logger().error(f'디버깅 데이터 출력 중 오류: {str(ex)}')
                    
                    time.sleep(0.1)
                    
            except socket.timeout:
                # 타임아웃은 무시 (스레드 종료 확인용)
                pass
            except Exception as e:
                self.get_logger().error(f'패킷 수신/처리 오류: {str(e)}')
                self.error_count += 1
                
                # 오류 추적을 위한 로깅 추가
                import traceback
                self.get_logger().error(f'상세 오류: {traceback.format_exc()}')
                # 오류 추적을 위한 로깅 추가
                import traceback
                self.get_logger().error(traceback.format_exc())
                time.sleep(0.1)
    
    def update_debug_window(self):
        """쉘에 제스처 정보 출력"""
        # 제스처에 해당하는 심볼 가져오기
        symbol = self.gesture_symbols.get(self.current_gesture, "•")
        
        # 제스처 타입에 따라 다른 로그 메시지 출력
        if self.current_gesture == "go":
            self.get_logger().info(f"{symbol} GO 제스처 감지: 전진 [로봇: {self.robot_id}]")
        elif self.current_gesture == "back":
            self.get_logger().info(f"{symbol} BACK 제스처 감지: 후진 [로봇: {self.robot_id}]")
        elif self.current_gesture == "left":
            self.get_logger().info(f"{symbol} LEFT 제스처 감지: 좌회전 [로봇: {self.robot_id}]")
        elif self.current_gesture == "right":
            self.get_logger().info(f"{symbol} RIGHT 제스처 감지: 우회전 [로봇: {self.robot_id}]")
        elif self.current_gesture == "stop":
            self.get_logger().info(f"{symbol} STOP 제스처 감지: 정지 [로봇: {self.robot_id}]")
        else:  # "none"
            self.get_logger().debug(f"{symbol} 제스처 없음 [로봇: {self.robot_id}]")
            
        # 제스처 정보 요약 로그 (콘솔에서 한눈에 보기 쉽게)
        if self.current_gesture != "none":
            self.get_logger().info("-" * 50)
            self.get_logger().info(f"현재 제스처: {self.current_gesture.upper()}")
            self.get_logger().info(f"로봇 ID: {self.robot_id}")
            self.get_logger().info("-" * 50)
            
    def destroy_node(self):
        """노드 종료 시 정리 작업"""
        self.get_logger().info('손 제스처 ROS2 브릿지 노드 종료 중...')
        
        # 최종 통계 출력
        if self.message_count > 0:
            success_rate = (self.success_count / self.message_count) * 100
            self.get_logger().info(f"===== 최종 통계 =====")
            self.get_logger().info(f"처리된 총 메시지: {self.message_count}")
            self.get_logger().info(f"성공: {self.success_count}, 오류: {self.error_count}")
            self.get_logger().info(f"최종 성공률: {success_rate:.1f}%")
            self.get_logger().info(f"====================")
        
        self.is_running = False
        
        if self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
            
        self.sock.close()
        self.get_logger().info('소켓이 정상적으로 닫혔습니다.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureROS2Bridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
