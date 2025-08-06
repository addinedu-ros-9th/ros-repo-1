import pyaudio
import socket
import os
import io
from datetime import datetime
import pytz
import numpy as np
import threading
import queue
import time
import rclpy
from rclpy.node import Node
from google.cloud import texttospeech
from libo_interfaces.msg import VoiceCommand

# ======================== 상수 정의 =========================
# AI_SERVICE_IP = "192.168.1.7"            # AI Service IP (TCP 서버)
AI_SERVICE_IP = "127.0.0.1"            # AI Service IP (TCP 서버)
SPEAKER_PORT = 7002                     # 스피커 출력용 포트 (TCP)

CHANNELS = 1                           # 모노 채널
RATE = 24000                          # 샘플링 레이트
CHUNK = 1024                          # 청크 크기
FORMAT = pyaudio.paFloat32            # 32비트 부동소수점
MAX_RETRY_COUNT = 5                   # 최대 재연결 시도 횟수
RETRY_INTERVAL = 2                    # 재연결 시도 간격(초)
# ===========================================================

# ================== 프로젝트 루트 경로 세팅 ==================
# 현재 실행 경로에서 ros-repo-1 위치 찾기
current_path = os.path.abspath(os.path.dirname(__file__))
print(f"현재 경로: {current_path}")

# ros_ws의 위치를 찾아서 그 상위 디렉토리를 프로젝트 루트로 설정
ros_ws_index = current_path.find('/ros_ws/')
if ros_ws_index != -1:
    PROJECT_ROOT = current_path[:ros_ws_index]
else:
    # 백업 방법: 현재 위치에서 상위로 올라가며 찾기
    PROJECT_ROOT = os.path.abspath(os.path.join(current_path, '../../../../../../'))
    
# MP3 효과음 디렉토리 설정
MP3_EFFECTS_DIR = os.path.join(PROJECT_ROOT, "data", "mp3_effect_files")

# Google Cloud 인증 파일 경로 설정
GOOGLE_CREDS_PATH = os.path.join(PROJECT_ROOT, "data", "fleet-unison-452704-j5-31aaeff5ac33.json")
# ===========================================================

def get_kr_time():
    """한국 시간 ISO 포맷으로 반환"""
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')
        
        self.get_logger().info('Speaker Node 초기화 중...')
        
        # PyAudio 초기화
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            output=True,
            frames_per_buffer=CHUNK
        )
        
        self.audio_queue = queue.Queue()
        self.is_running = True
        self.tcp_socket = None
        
        # Google TTS 클라이언트 초기화
        if os.path.exists(GOOGLE_CREDS_PATH):
            self.tts_client = texttospeech.TextToSpeechClient.from_service_account_file(GOOGLE_CREDS_PATH)
            self.get_logger().info('Google TTS 클라이언트 초기화 완료')
        else:
            self.get_logger().error(f'Google Cloud 인증 파일이 없습니다: {GOOGLE_CREDS_PATH}')
            self.tts_client = None
        
        # 음성 명령어 사전 정의
        self.voice_commands = {
            # 공통 음성 명령
            "common": {
                "power_on": {"type": "mp3", "value": "power_on.mp3"},                   # (전원 켜지는 소리 - 삐빅)
                "initialized": {"type": "mp3", "value": "robot_initialized.mp3"},       # (초기화 완료 소리 - 따리리리링)
                "charging": {"type": "tts", "value": "충전을 시작하겠습니다."},
                "battery_sufficient": {"type": "tts", "value": "배터리가 충분합니다. 대기모드로 전환합니다."},
                "depart_base": {"type": "tts", "value": "출발합니다~ (충전기를 뽑고)"},
                "high_weigh": {"type": "tts", "value": "바구니가 가득 찼습니다. 정리해주세요."},
                "obstacle_detected": {"type": "mp3", "value": "honk.mp3"},              # (장애물이 감지됐습니다. 잠시합니다. / 빵!!!!!!!!!!)
                "reroute": {"type": "tts", "value": "새로운 경로로 안내합니다."},
                "return": {"type": "mp3", "value": "complete.mp3"},                     # (복귀하겠습니다. / (북귀음 소리 - 빠빕))
                "arrived_base": {"type": "tts", "value": "Base에 도착했습니다."},
                "navigation_canceled": {"type": "tts", "value": "주행이 취소되었습니다"},
                "emergency_stop": {"type": "tts", "value": "비상 정지! 안전을 위해 모든 작업을 중단합니다."},
                "emergency_recovery": {"type": "tts", "value": "비상 상황이 해결되었습니다. 정상 상태로 복구합니다"}
            },
            
            # 안내 관련 음성 명령
            "escort": {
                "depart_base": {"type": "tts", "value": "출발합니다~"},
                "arrived_kiosk": {"type": "tts", "value": "잭 위치까지 에스코팅을 시작하겠습니다. 뒤로 따라와주시길 바랍니다."},
                "lost_user": {"type": "tts", "value": "손님이 보이지 않습니다. 5초 후에 자동종료 됩니다."},
                "user_reconnected": {"type": "mp3", "value": "reconnected.mp3"},        # (다시 연결된 소리, 뿌루루? 빠빅?)
                "arrived_destination": {"type": "tts", "value": "도착했습니다. 더 필요한 것이 있으면 키오스크에서 불러주세요."},
                "return": {"type": "mp3", "value": "complete.mp3"},                     # 복귀하겠습니다. / (북귀음 소리 - 빠빕)
                "arrived_base": {"type": "tts", "value": "Base에 도착했습니다."}
            },
            
            # 배송 관련 음성 명령
            "delivery": {
                "depart_base": {"type": "tts", "value": "출발합니다~"},
                "arrived_admin_desk": {"type": "tts", "value": "딜리버리 준비가 완료되었습니다. 다음 목적지를 선택해주세요."},
                "receive_next_goal": {"type": "tts", "value": "목적지를 수신하였습니다. 출발하겠습니다."},
                "arrived_destination": {"type": "tts", "value": "도착했습니다. 작업이 완료되면 말해주세요."},
                "called_by_staff": {"type": "mp3", "value": "ribo_response.mp3"},        # 네? / (삐빅)
                "return": {"type": "mp3", "value": "complete.mp3"},                      # 복귀하겠습니다. / (북귀음 소리 - 빠빕)
                "arrived_base": {"type": "tts", "value": "Base에 도착했습니다."}
            },
            
            # 도움 관련 음성 명령
            "assist": {
                "depart_base": {"type": "tts", "value": "출발합니다~"},
                "arrived_kiosk": {"type": "tts", "value": "어시스트를 시작하시려면 QR 코드를 카메라 앞에 대주세요"},
                "qr_authenticated": {"type": "tts", "value": "QR 인증 완료! 어시스트를 시작하기 위해 카메라 앞에 서주세요."},
                "no_person_5s": {"type": "tts", "value": "감지 실패!"},
                "person_detected": {"type": "tts", "value": "감지 성공!"},
                "called_by_staff": {"type": "mp3", "value": "ribo_response.mp3"},       # 네? / (삐빅)
                "pause": {"type": "tts", "value": "일시정지합니다."},
                "resume": {"type": "tts", "value": "어시스트를 재개합니다."},
                "return": {"type": "mp3", "value": "complete.mp3"},                     # 복귀하겠습니다. / (북귀음 소리 - 빠빕)
                "arrived_base": {"type": "tts", "value": "Base에 도착했습니다."}
            }
        }
        
        # VoiceCommand 토픽 구독 설정
        self.voice_cmd_sub = self.create_subscription(
            VoiceCommand,
            '/voice_command',
            self.voice_command_callback,
            10
        )
        self.get_logger().info('VoiceCommand 토픽 구독 시작')
        
    def connect_to_server(self):
        """AI 서비스에 TCP 연결"""
        retry_count = 0
        print(f"[{get_kr_time()}][TCP] 🔌 연결 시도: {AI_SERVICE_IP}:{SPEAKER_PORT} → AI 서비스 TCP 서버")
        while retry_count < MAX_RETRY_COUNT:
            try:
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.connect((AI_SERVICE_IP, SPEAKER_PORT))
                print(f"[{get_kr_time()}][TCP] ✅ 연결 성공: {AI_SERVICE_IP}:{SPEAKER_PORT} → AI 서비스")
                return True
            except ConnectionRefusedError:
                retry_count += 1
                print(f"[{get_kr_time()}][TCP] ❌ 연결 실패 ({retry_count}/{MAX_RETRY_COUNT}): {AI_SERVICE_IP}:{SPEAKER_PORT}. {RETRY_INTERVAL}초 후 재시도...")
                time.sleep(RETRY_INTERVAL)
                
        print(f"[{get_kr_time()}][ERROR] ⛔ 최대 재시도 횟수 초과. 프로그램을 종료합니다.")
        return False

    def receive_audio(self):
        """TCP로 오디오 데이터 수신"""
        print(f"[{get_kr_time()}][INIT] 🎧 오디오 수신 대기 중... ({AI_SERVICE_IP}:{SPEAKER_PORT})")
        connection_alive = True
        
        while self.is_running and connection_alive:
            try:
                # 소켓에 1초 타임아웃 설정
                self.tcp_socket.settimeout(1.0)
                
                # 데이터 크기 수신 (4바이트)
                size_data = self.tcp_socket.recv(4)
                if not size_data:
                    print(f"[{get_kr_time()}][TCP] 서버와의 연결이 종료되었습니다.")
                    connection_alive = False
                    break
                    
                total_size = int.from_bytes(size_data, byteorder='big')
                received_size = 0
                
                # 데이터 크기에 따른 메시지 유형 분류
                if total_size < 1024:  # 1KB 미만은 무시
                    # 매우 작은 데이터는 로그 출력하지 않고 무시
                    while received_size < total_size and self.is_running:
                        chunk_size = min(CHUNK * 4, total_size - received_size)
                        data = self.tcp_socket.recv(chunk_size)
                        if not data:
                            break
                        self.audio_queue.put(data)
                        received_size += len(data)
                    continue
                
                # 메시지 유형을 크기에 따라 더 세분화
                if total_size < 10240:  # 10KB 미만
                    message_type = "짧은 음성 응답"
                    estimated_time = total_size / (RATE * CHANNELS * 4) 
                elif total_size < 51200:  # 50KB 미만
                    message_type = "중간 길이 음성 응답"
                    estimated_time = total_size / (RATE * CHANNELS * 4)
                elif total_size < 204800:  # 200KB 미만
                    message_type = "일반 음성 응답"
                    estimated_time = total_size / (RATE * CHANNELS * 4)
                else:
                    message_type = "긴 음성 응답"
                    estimated_time = total_size / (RATE * CHANNELS * 4)
                
                # 전체 데이터 수신
                while received_size < total_size and self.is_running:
                    chunk_size = min(CHUNK * 4, total_size - received_size)
                    data = self.tcp_socket.recv(chunk_size)
                    if not data:
                        break
                    self.audio_queue.put(data)
                    received_size += len(data)
                
                # 데이터 크기가 일정 이상일 때만 수신 완료 메시지 출력
                kb_size = received_size / 1024
                print(f"[{get_kr_time()}][AUDIO] 📢 {message_type} 수신 완료: {kb_size:.1f}KB | 예상 재생 시간: {estimated_time:.2f}초")
                    
            except socket.timeout:
                # 타임아웃은 정상 - 조용히 넘어감
                pass
            except ConnectionResetError:
                print(f"[{get_kr_time()}][TCP] 서버에 의해 연결이 재설정되었습니다.")
                connection_alive = False
                break
            except Exception as e:
                print(f"[{get_kr_time()}][ERROR] 수신 오류: {str(e)}")
                connection_alive = False
                break
                
        # 연결이 끊어진 경우 재연결 시도
        if self.is_running:
            print(f"[{get_kr_time()}][TCP] 🔄 서버({AI_SERVICE_IP}:{SPEAKER_PORT})와의 연결이 끊어졌습니다. 재연결을 시도합니다.")
            if self.connect_to_server():
                self.receive_audio()

    def play_audio(self):
        """수신된 오디오 재생"""
        print(f"[{get_kr_time()}][INIT] 오디오 재생 스레드 시작")
        audio_chunk_count = 0  # 재생된 청크 수 카운터
        total_bytes_played = 0  # 총 재생된 바이트 수
        start_time = None  # 재생 시작 시간
        prev_queue_empty = True  # 이전 큐 상태 (빈 상태였는지)
        
        while self.is_running:
            try:
                if not self.audio_queue.empty():
                    # 재생 시작 시간 기록
                    if prev_queue_empty:
                        start_time = time.time()
                        prev_queue_empty = False
                    
                    data = self.audio_queue.get()
                    audio_data = np.frombuffer(data, dtype=np.float32)
                    self.stream.write(audio_data.tobytes())
                    
                    # 디버깅 정보 업데이트
                    audio_chunk_count += 1
                    total_bytes_played += len(data)
                    
                    # 데이터 크기가 10KB 이상이고, 20개 청크마다 디버깅 정보 출력
                    if audio_chunk_count % 20 == 0 and total_bytes_played > 10240:
                        elapsed = time.time() - start_time if start_time else 0
                        print(f"[{get_kr_time()}][AUDIO] 🔊 재생 중: {audio_chunk_count}개 청크 | "
                              f"{total_bytes_played/1024:.1f}KB | 경과 시간: {elapsed:.2f}초")
                else:
                    # 큐가 빈 상태로 변경된 경우 (재생 완료)
                    if not prev_queue_empty and audio_chunk_count > 0 and total_bytes_played > 1024:
                        elapsed = time.time() - start_time if start_time else 0
                        print(f"[{get_kr_time()}][AUDIO] ✅ 재생 완료: {audio_chunk_count}개 청크 | "
                              f"{total_bytes_played/1024:.1f}KB | 소요 시간: {elapsed:.2f}초")
                        audio_chunk_count = 0
                        total_bytes_played = 0
                    
                    prev_queue_empty = True
                    time.sleep(0.01)  # 큐가 비었을 때 CPU 사용량 감소
                    
            except Exception as e:
                print(f"[{get_kr_time()}][ERROR] 재생 오류: {str(e)}")

    def run(self):
        """메인 실행 함수"""
        print(f"[{get_kr_time()}][INIT] speaker_node 시작")
        
        # AI 서비스에 연결
        if not self.connect_to_server():
            return
            
        # 수신 및 재생 스레드 시작
        receive_thread = threading.Thread(target=self.receive_audio)
        play_thread = threading.Thread(target=self.play_audio)
        
        receive_thread.start()
        play_thread.start()
        
        try:
            receive_thread.join()
            play_thread.join()
        except KeyboardInterrupt:
            print(f"[{get_kr_time()}][SYSTEM] 사용자에 의해 중지됨")
        finally:
            self.cleanup()

    def voice_command_callback(self, msg):
        """
        VoiceCommand 메시지 처리 콜백
        
        Args:
            msg.robot_id: 로봇 ID (예: "libo_a", "libo_b")
            msg.category: 명령 카테고리 (예: "escort", "delivery")
            msg.action: 명령 액션 (예: "arrived", "return")
        """
        robot_id = msg.robot_id
        category = msg.category
        action = msg.action
        
        print(f"[{get_kr_time()}][COMMAND] {'=' * 30} VoiceCommand 수신 {'=' * 30}")
        self.get_logger().info(f'VoiceCommand 수신: 로봇={robot_id}, 카테고리={category}, 액션={action}')
        
        # 카테고리와 액션으로 음성 명령 재생
        self.play_voice_command(category, action)
        
    def play_voice_command(self, category, action):
        """카테고리와 액션에 따라 음성 명령을 재생
        
        Args:
            category (str): 명령 카테고리 ('common', 'escort', 'delivery', 'assist')
            action (str): 카테고리 내 액션 이름
            
        Returns:
            bool: 명령 재생 성공 여부
        """
        try:
            # 카테고리에 해당하는 명령이 있는지 확인
            if category in self.voice_commands:
                command_dict = self.voice_commands[category]
                
                # 액션에 해당하는 명령이 있는지 확인
                if action in command_dict:
                    command = command_dict[action]
                    
                    # 명령 타입에 따른 처리
                    if command['type'] == 'mp3':
                        return self.play_mp3_effect(command['value'])
                    elif command['type'] == 'tts':
                        return self.play_tts_response(command['value'])
                    else:
                        print(f"[{get_kr_time()}][WARNING] 알 수 없는 명령 타입: {command['type']}")
                        return False
                else:
                    print(f"[{get_kr_time()}][WARNING] 액션 '{action}'을(를) 찾을 수 없습니다 (카테고리: {category})")
                    return False
            else:
                print(f"[{get_kr_time()}][WARNING] 카테고리 '{category}'을(를) 찾을 수 없습니다")
                return False
                
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] 음성 명령 재생 오류: {str(e)}")
            return False
    
    def play_mp3_effect(self, file_name):
        """MP3 효과음 재생"""
        try:
            # MP3 파일 경로 구성
            file_path = os.path.join(MP3_EFFECTS_DIR, file_name)
            
            if not os.path.exists(file_path):
                print(f"[{get_kr_time()}][ERROR] MP3 파일이 존재하지 않습니다: {file_path}")
                return False
                
            print(f"[{get_kr_time()}][AUDIO] MP3 효과음 재생 시작: {file_name}")
            
            # TODO: MP3 파일을 오디오 데이터로 변환하여 스피커에 출력하는 코드 구현
            # 현재는 구현되지 않았으므로 TTS로 대체
            return self.play_tts_response(f"효과음 {file_name}을 재생하려 했으나 기능이 구현되지 않았습니다.")
            
        except Exception as e:
            print(f"[{get_kr_time()}][WARNING] MP3 파일 로드 실패, TTS로 대체: {str(e)}")
            return self.play_tts_response(f"효과음 {file_name}을 재생하려 했으나 실패했습니다.")
    
    def play_tts_response(self, text):
        """텍스트를 TTS로 변환하여 재생"""
        try:
            if not self.tts_client:
                print(f"[{get_kr_time()}][ERROR] TTS 클라이언트가 초기화되지 않았습니다")
                return False
                
            print(f"[{get_kr_time()}][TTS] 음성 응답 생성 중: {text}")
                
            synthesis_input = texttospeech.SynthesisInput(text=text)
            
            voice = texttospeech.VoiceSelectionParams(
                language_code="ko-KR",
                name="ko-KR-Standard-A",
                ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
            )
            
            audio_config = texttospeech.AudioConfig(
                audio_encoding=texttospeech.AudioEncoding.LINEAR16,
                sample_rate_hertz=RATE,
            )
            
            tts_response = self.tts_client.synthesize_speech(
                input=synthesis_input,
                voice=voice,
                audio_config=audio_config
            )
            
            # 오디오 데이터를 float32로 변환 및 처리
            audio_data = np.frombuffer(tts_response.audio_content, dtype=np.int16)
            audio_float32 = audio_data.astype(np.float32) / 32768.0
            
            # 볼륨 3dB 증가 (약 1.4배)
            audio_float32 *= 1.4
            
            # 클리핑 방지 (값이 1.0을 넘지 않도록)
            audio_float32 = np.clip(audio_float32, -1.0, 1.0)
            
            # 오디오 데이터를 큐에 추가하여 재생
            print(f"[{get_kr_time()}][AUDIO] TTS 오디오 데이터 추가 중... (볼륨 3dB 증가)")
            
            # 데이터를 바이트로 변환하여 큐에 추가
            for i in range(0, len(audio_float32), CHUNK):
                chunk = audio_float32[i:i + CHUNK]
                if len(chunk) < CHUNK:
                    chunk = np.pad(chunk, (0, CHUNK - len(chunk)))
                self.audio_queue.put(chunk.tobytes())
            
            print(f"[{get_kr_time()}][AUDIO] TTS 데이터 큐 추가 완료 (길이: {len(audio_float32)})")
            return True
            
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] TTS 생성/전송 오류: {str(e)}")
            return False

    def cleanup(self):
        """리소스 정리"""
        print(f"[{get_kr_time()}][CLEANUP] 리소스 정리 중...")
        self.is_running = False
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.pa:
            self.pa.terminate()
        if self.tcp_socket:
            self.tcp_socket.close()
            
        print(f"[{get_kr_time()}][SYSTEM] 프로그램 종료")

def main(args=None):
    rclpy.init(args=args)
    speaker = SpeakerNode()
    
    # TCP 서버 연결 및 스레드 시작
    if speaker.connect_to_server():
        # 수신 및 재생 스레드 시작
        receive_thread = threading.Thread(target=speaker.receive_audio)
        play_thread = threading.Thread(target=speaker.play_audio)
        
        receive_thread.start()
        play_thread.start()
        
        try:
            rclpy.spin(speaker)
        except KeyboardInterrupt:
            print(f"[{get_kr_time()}][SYSTEM] 사용자에 의해 중지됨")
        finally:
            speaker.cleanup()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
