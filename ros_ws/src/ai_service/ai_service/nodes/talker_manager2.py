import os
import sys
import io
import time
import socket
import threading
import queue
import struct
import wave
import pytz
import openai
import numpy as np
import resampy
import pvporcupine
import re
import rclpy
import speech_recognition as sr
import os
import time
import pytz
import traceback
import numpy as np
import re
import speech_recognition as sr
from datetime import datetime
from dotenv import load_dotenv
from pydub import AudioSegment
from google.cloud import texttospeech
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from libo_interfaces.msg import TalkCommand, FaceExpression, VoiceCommand
from libo_interfaces.srv import EndTask, ActivateTalker, DeactivateTalker, ActivateGesture, DeactivateGesture, ActivateTracker, DeactivateTracker
from std_msgs.msg import Float32  # ESP에서 발행하는 weight_data 토픽 사용


# ================== 네트워크/오디오 기본 설정 ==================
# 네트워크 설정
HARDWARE_HANDLER_IP = "0.0.0.0"      # 🖥️ Hardware Handler IP (UDP 서버 주소)
MIC_STREAM_PORT = 7010                 # 🎤 마이크 스트림 포트 (UDP 수신)

# 오디오 설정
NATIVE_RATE = 48000                    # 🎵 원본 샘플링 레이트 (마이크용)
# NATIVE_RATE = 44100                    # 🎵 원본 샘플링 레이트 (웹캠 마이크용)
TARGET_RATE = 16000                    # 🎯 웨이크워드 처리용 레이트

print(f"[NETWORK CONFIG] 📡 UDP 서버: {HARDWARE_HANDLER_IP}:{MIC_STREAM_PORT} - 마이크 스트림 수신")

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

# MP3 효과음 디렉토리가 없으면 생성
if not os.path.exists(MP3_EFFECTS_DIR):
    try:
        os.makedirs(MP3_EFFECTS_DIR)
        print(f"MP3 효과음 디렉토리 생성됨: {MP3_EFFECTS_DIR}")
    except Exception as e:
        print(f"MP3 효과음 디렉토리 생성 오류: {str(e)}")

print(f"프로젝트 루트 경로: {PROJECT_ROOT}")
env_path = os.path.join(PROJECT_ROOT, '.env')
print(f".env 파일 경로: {env_path}")
print(f"MP3 효과음 디렉토리 경로: {MP3_EFFECTS_DIR}")

# 파일 존재 확인
if not os.path.exists(env_path):
    raise FileNotFoundError(f".env 파일을 찾을 수 없습니다: {env_path}")

load_dotenv(dotenv_path=env_path)

# ================== 환경 변수 및 모델 경로 ==================
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
PICOVOICE_ACCESS_KEY = os.getenv("PICOVOICE_ACCESS_KEY")

if not all([OPENAI_API_KEY, PICOVOICE_ACCESS_KEY]):
    raise ValueError("OPENAI_API_KEY 또는 PICOVOICE_ACCESS_KEY가 .env에 없습니다!")

DATA_DIR = os.path.join(PROJECT_ROOT, 'data')

# Google Cloud 인증 키 설정 (직접 파일 사용)
GOOGLE_CREDS_PATH = os.path.join(DATA_DIR, 'fleet-unison-452704-j5-31aaeff5ac33.json')
if not os.path.exists(GOOGLE_CREDS_PATH):
    raise FileNotFoundError(f"Google Cloud 인증 파일이 존재하지 않습니다: {GOOGLE_CREDS_PATH}")

# Google TTS 클라이언트 초기화 (인증 파일 직접 사용)
tts_client = texttospeech.TextToSpeechClient.from_service_account_file(GOOGLE_CREDS_PATH)

PORCUPINE_MODEL_PATH = os.path.join(DATA_DIR, 'porcupine_params_ko.pv')
PORCUPINE_KEYWORD_PATH = os.path.join(DATA_DIR, 'riboya_ko_linux_v3_0_0.ppn')

for path in [PORCUPINE_MODEL_PATH, PORCUPINE_KEYWORD_PATH]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"필요한 모델 파일이 존재하지 않습니다: {path}")

# ================== 오디오/모드 관련 상수 정의 ==================
# 모드 설정
FOLLOW_MODE = "follow"
GESTURE_MODE = "gesture"
CURRENT_MODE = FOLLOW_MODE  # 기본 모드는 Follow 모드

# 오디오 관련 상수
CHANNELS = 1
CHUNK = 2048                           # mic_streamer와 동일
CHANNELS = 1
CHUNK = 2048                           # mic_streamer와 동일
# ================== 유틸리티 함수들 ==================
def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시

def log(tag, message):
    """일관된 형식으로 로그 출력"""
    print(f"[{get_kr_time()}][{tag}] {message}")

def recognize_speech(recognizer, audio_file_path):
    """음성을 텍스트로 변환"""
    transcript = None
    try:
        with sr.AudioFile(audio_file_path) as source:
            log("STT", "음성 파일 로드 완료. 구글 STT API 호출 중...")
            audio = recognizer.record(source)
            try:
                transcript = recognizer.recognize_google(audio, language="ko-KR")
                log("STT", f"사용자 발화: {transcript}")
            except sr.UnknownValueError:
                log("STT", "❌ 음성 인식 실패 (음성을 감지할 수 없음)")
            except Exception as e:
                log("STT", f"❌ STT 오류: {e}")
    except Exception as e:
        log("ERROR", f"STT 처리 중 예외 발생: {str(e)}")
    
    return transcript

def analyze_intent(client, transcript):
    """OpenAI API를 사용하여 텍스트에서 의도 추출"""
    intent = "ignore"  # 기본값
    system_prompt = """
    당신은 로봇의 음성 명령을 분석하는 AI입니다.
    사용자의 발화를 듣고, 아래 의도 중 하나로 분류하세요.

    - pause_assist: '잠깐 멈춰', '잠깐만 기다려봐' 등 작업 일시중지 명령
    - resume_assist: '다시 따라와', '계속하자' 등 작업 재개 명령
    - start_gesture: '제스쳐모드 시작', '내 동작 보고 따라와' 등 제스처 모드 시작 명령
    - start_follow: '팔로우 모드 시작' 등 팔로우 모드 시작 명령
    - get_mode: '지금 어떤 모드야?' 등 현재 모드 확인 명령
    - get_weight: '지금 무게 얼마나 돼?', '지금 책 무게는?' 등 책 무게 확인 명령
    - stop_assist: '이제 그만하고 복귀하자', '제스쳐 모드 중지', '어시스트 중지' 등 작업 중지 명령
    - get_weather: '오늘 날씨 어때?' 등 날씨 확인 명령
    - ignore: '아니야 잘못불렀어', '오늘 날씨 어때' 등 무시할만한 명령

    결과는 반드시 다음 JSON 형식으로만 출력해야 합니다:
    {"intent": "..."}
    """
    
    try:
        completion = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": transcript}
            ]
        )
        ai_text = completion.choices[0].message.content
        log("GPT", f"AI 응답: {ai_text}")
        match = re.search(r'{"intent":\s*"(\w+)"}', ai_text)
        if match:
            intent = match.group(1)
            log("INTENT", f"감지된 의도: {intent}")
        else:
            log("INTENT", "⚠️ 의도 분석 실패, 기본값 'ignore' 사용")
    except Exception as e:
        log("ERROR", f"의도 분석 중 오류 발생: {str(e)}")
    
    return intent

def collect_audio(comm_manager, max_time=15.0, silence_threshold=300, silence_duration=1.5):
    """
    마이크에서 오디오 데이터 수집 (침묵 감지 기능 포함)
    
    Args:
        comm_manager: 통신 관리자 인스턴스
        max_time: 최대 녹음 시간(초)
        silence_threshold: 침묵 감지 임계값(RMS)
        silence_duration: 침묵으로 간주할 시간(초)
        
    Returns:
        tuple: (collected_data, duration)
    """
    collected = b''
    start = time.time()
    last_active_time = time.time()
    has_speech_started = False
    
    try:
        while time.time() - start < max_time:
            if not comm_manager.buffer_queue.empty():
                data = comm_manager.buffer_queue.get()
                collected += data
                
                # 현재 청크의 소리 크기 측정 (RMS)
                if len(data) >= CHUNK * 2:  # 최소 1개 청크 이상
                    pcm = struct.unpack_from("h" * (len(data) // 2), data)
                    rms = np.sqrt(np.mean(np.square(pcm)))
                    
                    # 소리가 임계값보다 크면 활동으로 간주
                    if rms > silence_threshold:
                        last_active_time = time.time()
                        if not has_speech_started and len(collected) > CHUNK * 10:  # 처음 몇 청크는 노이즈일 수 있으므로 건너뜀
                            has_speech_started = True
                            log("RECORD", f"🗣️ 음성 감지됨 (RMS: {rms:.1f})")
                
                # 로그 출력
                if len(collected) % (CHUNK * 10) == 0:
                    elapsed = time.time() - start
                    log("RECORD", f"수집된 데이터: {len(collected)} bytes (경과 시간: {elapsed:.1f}초)")
                
                # 침묵 감지 로직: 음성이 시작된 후 일정 시간동안 침묵이 계속되면 녹음 종료
                if has_speech_started and time.time() - last_active_time > silence_duration:
                    log("RECORD", f"⏹️ 침묵 감지: {silence_duration}초 동안 소리가 없어 녹음 종료")
                    break
                    
            else:
                time.sleep(0.01)
    except Exception as e:
        log("ERROR", f"음성 수집 중 오류: {str(e)}")
    
    duration = time.time() - start
    return collected, duration

def save_wav_file(filepath, audio_data, channels=CHANNELS, sample_width=2, framerate=NATIVE_RATE):
    """오디오 데이터를 WAV 파일로 저장"""
    try:
        with wave.open(filepath, 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(sample_width)  # 16-bit
            wf.setframerate(framerate)
            wf.writeframes(audio_data)
        return True
    except Exception as e:
        log("ERROR", f"WAV 파일 저장 중 오류: {str(e)}")
        return False

class CommunicationManager:
    def __init__(self):
        self.udp_sock = None
        self.buffer_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.is_active = False  # 웨이크워드 및 명령 처리 활성화 상태 변수 - 기본값 False로 변경
        self.current_robot_id = "unknown"  # 현재 활성화/비활성화 요청한 로봇 ID
        self.last_status_report_time = 0  # 마지막 상태 출력 시간
        self.last_status = False  # 마지막 상태 기록
        self.talker_node = None  # TalkerNode 참조 (나중에 설정됨)
        print(f"[{get_kr_time()}][CONFIG] 토커매니저 기본 상태: 비활성화됨 (웨이크워드 감지 불가능)")

    def start_udp_receiver(self):
        """UDP 수신기 초기화 및 시작"""
        def _udp_receiver():
            try:
                print(f"[{get_kr_time()}][UDP] 🎤 마이크 스트림 UDP 서버 초기화 중...")
                self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # SO_REUSEADDR 설정으로 포트 재사용 허용
                self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                
                # UDP 소켓 바인딩 시도
                try:
                    self.udp_sock.bind((HARDWARE_HANDLER_IP, MIC_STREAM_PORT))
                    print(f"[{get_kr_time()}][UDP] 📡 마이크 스트림 수신 대기 중... ({HARDWARE_HANDLER_IP}:{MIC_STREAM_PORT})")
                except OSError as e:
                    if e.errno == 98:  # Address already in use
                        print(f"[{get_kr_time()}][UDP] ⚠️ 포트 {MIC_STREAM_PORT}가 이미 사용 중입니다. 바인딩 없이 계속 진행합니다.")
                        # 바인딩 없이 계속 진행 - 다른 프로세스가 이미 수신 중이므로 오디오 데이터는 처리될 것임
                    else:
                        print(f"[{get_kr_time()}][UDP] ❌ UDP 바인딩 오류: {str(e)}")
                        print(f"[{get_kr_time()}][UDP] ⚠️ 바인딩 없이 계속 진행합니다.")
                        
                print(f"[{get_kr_time()}][UDP] ⚙️  설정: CHUNK={CHUNK}, NATIVE_RATE={NATIVE_RATE}Hz, TARGET_RATE={TARGET_RATE}Hz")
                print(f"[{get_kr_time()}][UDP] ⚙️  설정: CHUNK={CHUNK}, NATIVE_RATE={NATIVE_RATE}Hz, TARGET_RATE={TARGET_RATE}Hz")
                
                received_count = 0
                start_time = time.time()
                
                # 소켓이 바인딩되었는지 확인
                is_socket_bound = True
                try:
                    # 소켓 바인딩 확인 (getpeername 또는 getsockname으로)
                    local_addr = self.udp_sock.getsockname()
                    if not local_addr:
                        is_socket_bound = False
                except:
                    is_socket_bound = False
                
                # 바인딩 여부에 따라 동작 분리
                if is_socket_bound:
                    print(f"[{get_kr_time()}][UDP] ✅ UDP 소켓 바인딩 상태: 정상")
                    while not self.stop_event.is_set():
                        try:
                            data, _ = self.udp_sock.recvfrom(CHUNK * 2)
                            # 활성화 상태인 경우에만 버퍼에 넣음 - 비활성화 상태에서는 오디오 데이터 처리하지 않음
                            if self.is_active:
                                self.buffer_queue.put(data)
                            received_count += 1
                            
                            if received_count % 1000 == 0:
                                elapsed = time.time() - start_time
                                rate = received_count / elapsed
                                data_rate = (rate * CHUNK * 2) / 1024  # KB/s
                                active_status = "활성화" if self.is_active else "비활성화"
                                robot_id = self.current_robot_id
                                print(f"[{get_kr_time()}][UDP] 📊 수신 통계: {received_count}개 패킷, {rate:.2f} packets/sec, {data_rate:.1f} KB/s, 상태: {active_status}, 로봇: {robot_id}")
                                print(f"[{get_kr_time()}][UDP] 🔄 활성 연결: {HARDWARE_HANDLER_IP}:{MIC_STREAM_PORT} ← 오디오 스트림 수신 중")
                        except Exception as recv_e:
                            print(f"[{get_kr_time()}][UDP] ⚠️ 데이터 수신 오류: {str(recv_e)}")
                            time.sleep(0.1)  # 오류 시 잠시 대기
                else:
                    print(f"[{get_kr_time()}][UDP] ℹ️ 포트 {MIC_STREAM_PORT}에 바인딩되지 않았습니다. 마이크 스트리머가 별도 프로세스로 실행 중인 것으로 간주합니다.")
                    print(f"[{get_kr_time()}][UDP] 🔄 웨이크워드 감지는 계속 진행됩니다.")
                    
                    # 바인딩 없이 계속 실행 - 다른 방식으로 오디오 스트림을 받거나 필요한 처리 수행
                    while not self.stop_event.is_set():
                        time.sleep(0.5)  # 주기적으로 상태 확인
            except Exception as e:
                print(f"[{get_kr_time()}][UDP] ❌ 오류 발생: {str(e)}")
            finally:
                if self.udp_sock:
                    try:
                        self.udp_sock.close()
                        print(f"[{get_kr_time()}][UDP] 🛑 UDP 소켓 닫힘 ({HARDWARE_HANDLER_IP}:{MIC_STREAM_PORT})")
                    except Exception as close_e:
                        print(f"[{get_kr_time()}][UDP] ⚠️ 소켓 닫기 오류: {str(close_e)}")
                    
        thread = threading.Thread(target=_udp_receiver)
        thread.daemon = True
        thread.start()
        return thread

    def cleanup(self):
        """모든 리소스 정리"""
        self.stop_event.set()
        if self.udp_sock:
            self.udp_sock.close()

class TalkerNode(Node):
    """
    ROS2 노드 클래스 - 음성 명령 토픽 구독자 및 제어 명령 발행자
    """
    def __init__(self, comm_manager):
        global CURRENT_MODE
        CURRENT_MODE = "follow"  # 초기 모드 설정
        
        # 무게 관련 변수 초기화
        self.current_weight = 0.0
        self.weight_unit = "g"  # 기본 단위는 그램
        self.robot_id = "libo_a"  # 기본 로봇 ID
        super().__init__('talker_node')
        
        # 통신 관리자 참조 저장
        self.comm_manager = comm_manager
        
        # 로깅
        self.get_logger().info('TalkerNode 초기화 중...')
        
        # 콜백 그룹 생성 - 동시에 여러 콜백을 처리하기 위함
        self.callback_group = ReentrantCallbackGroup()
        
        # TalkCommand 토픽 발행자
        self.talk_cmd_pub = self.create_publisher(
            TalkCommand,
            '/talk_command',
            10
        )
        
        # VoiceCommand 토픽 발행자
        self.voice_cmd_pub = self.create_publisher(
            VoiceCommand,
            '/voice_command',
            10
        )
        
        # FaceExpression 토픽 발행자
        self.face_expr_pub = self.create_publisher(
            FaceExpression,
            '/face_expression',
            10
        )
        
        # Weight 데이터 토픽 구독자 (ESP에서 발행하는 Float32 타입)
        self.weight_sub = self.create_subscription(
            Float32,
            '/weight_data',
            self.weight_callback,
            10
        )
        self.get_logger().info('ESP의 /weight_data 토픽 구독 시작')
        
        # EndTask 서비스 클라이언트
        self.end_task_client = self.create_client(
            EndTask, 
            '/end_task',
            callback_group=self.callback_group
        )
        
        # Gesture/Tracker 서비스 클라이언트들
        self.activate_tracker_client = self.create_client(
            ActivateTracker,
            '/activate_tracker',
            callback_group=self.callback_group
        )
        
        self.deactivate_tracker_client = self.create_client(
            DeactivateTracker,
            '/deactivate_tracker',
            callback_group=self.callback_group
        )
        
        self.activate_gesture_client = self.create_client(
            ActivateGesture,
            '/activate_gesture',
            callback_group=self.callback_group
        )
        
        self.deactivate_gesture_client = self.create_client(
            DeactivateGesture,
            '/deactivate_gesture',
            callback_group=self.callback_group
        )
        
        # ActivateTalker 서비스 서버
        self.activate_service = self.create_service(
            ActivateTalker,
            '/activate_talker',
            self.activate_talker_callback,
            callback_group=self.callback_group
        )
        
        # DeactivateTalker 서비스 서버
        self.deactivate_service = self.create_service(
            DeactivateTalker,
            '/deactivate_talker',
            self.deactivate_talker_callback,
            callback_group=self.callback_group
        )
        
        # 서비스 가용성 확인 (비차단식)
        if not self.end_task_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('EndTask 서비스가 아직 활성화되지 않았습니다. 필요할 때 다시 시도합니다.')
        
        # 서비스 서버 로그
        log("SERVICE", "✅ ActivateTalker 서비스 서버 등록됨: /activate_talker")
        log("SERVICE", "✅ DeactivateTalker 서비스 서버 등록됨: /deactivate_talker")
        
        self.get_logger().info('TalkerNode 초기화 완료!')
    
    def call_end_task(self, robot_id):
        """작업 종료 서비스 호출
        
        Args:
            robot_id (str): 로봇 ID (예: "libo_a")
        """
        # 서비스 가용성 확인
        if not self.end_task_client.service_is_ready():
            self.get_logger().warning('EndTask 서비스가 준비되지 않았습니다. 5초간 대기 후 시도합니다.')
            if not self.end_task_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('EndTask 서비스를 사용할 수 없습니다.')
                return
        
        request = EndTask.Request()
        request.robot_id = robot_id
        
        self.get_logger().info(f"EndTask 서비스 호출 중 (robot_id: {robot_id})")
        future = self.end_task_client.call_async(request)
        future.add_done_callback(self.on_end_task_response)
        
    def on_end_task_response(self, future):
        """EndTask 서비스 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f"EndTask 서비스 성공: {response.message if response.message else 'No message'}")
                log("SERVICE", "✅ EndTask 서비스 호출 성공")
                
            else:
                self.get_logger().warning(f"EndTask 서비스 실패: {response.message if response.message else 'No message'}")
                log("SERVICE", f"⚠️ EndTask 서비스 호출 실패: {response.message}")
        except Exception as e:
            self.get_logger().error(f"EndTask 서비스 호출 중 오류 발생: {e}")
            log("SERVICE", f"❌ EndTask 서비스 호출 예외: {str(e)}")
    
    def publish_talk_command(self, robot_id, action):
        """TalkCommand 메시지 발행"""
        msg = TalkCommand()
        msg.robot_id = robot_id
        msg.action = action
        
        self.get_logger().info(f"TalkCommand 발행: robot_id={robot_id}, action={action}")
        self.talk_cmd_pub.publish(msg)
        
    def publish_voice_command(self, robot_id, category, action):
        """VoiceCommand 메시지 발행"""
        msg = VoiceCommand()
        msg.robot_id = robot_id
        msg.category = category
        msg.action = action
        
        self.get_logger().info(f"VoiceCommand 발행: robot_id={robot_id}, category={category}, action={action}")
        self.voice_cmd_pub.publish(msg)
        
    def activate_tracker(self, robot_id):
        """Tracker 활성화 서비스 호출"""
        try:
            req = ActivateTracker.Request()
            req.robot_id = robot_id
            self.activate_tracker_client.call_async(req)
            self.get_logger().info(f'ActivateTracker 서비스 호출: 로봇={robot_id}')
        except Exception as e:
            self.get_logger().error(f'ActivateTracker 서비스 호출 실패: {str(e)}')
    
    def deactivate_tracker(self, robot_id):
        """Tracker 비활성화 서비스 호출"""
        try:
            req = DeactivateTracker.Request()
            req.robot_id = robot_id
            self.deactivate_tracker_client.call_async(req)
            self.get_logger().info(f'DeactivateTracker 서비스 호출: 로봇={robot_id}')
        except Exception as e:
            self.get_logger().error(f'DeactivateTracker 서비스 호출 실패: {str(e)}')
    
    def activate_gesture(self, robot_id):
        """Gesture 활성화 서비스 호출"""
        try:
            req = ActivateGesture.Request()
            req.robot_id = robot_id
            self.activate_gesture_client.call_async(req)
            self.get_logger().info(f'ActivateGesture 서비스 호출: 로봇={robot_id}')
        except Exception as e:
            self.get_logger().error(f'ActivateGesture 서비스 호출 실패: {str(e)}')
    
    def deactivate_gesture(self, robot_id):
        """Gesture 비활성화 서비스 호출"""
        try:
            req = DeactivateGesture.Request()
            req.robot_id = robot_id
            self.deactivate_gesture_client.call_async(req)
            self.get_logger().info(f'DeactivateGesture 서비스 호출: 로봇={robot_id}')
        except Exception as e:
            self.get_logger().error(f'DeactivateGesture 서비스 호출 실패: {str(e)}')
            
    def weight_callback(self, msg):
        """ESP에서 발행한 weight_data(Float32) 토픽 메시지 수신 콜백"""
        # Float32 메시지에서 값 추출
        weight_value = msg.data  # Float32 메시지는 .data 필드에 값이 있음
        
        # 무게 정보 저장
        self.current_weight = weight_value
        
        # 무게에 따른 단위 자동 조정 (가독성 향상)
        if weight_value >= 1000:
            self.weight_unit = "kg"
            self.current_weight = weight_value / 1000.0
        else:
            self.weight_unit = "g"
        
        # 현재 로봇 ID와 함께 로그 출력
        self.get_logger().debug(f'Weight 데이터 수신: 로봇={self.robot_id}, 무게={self.current_weight}{self.weight_unit}')
        
    def activate_talker_callback(self, request, response):
        """
        토커매니저 활성화 서비스 콜백
        
        Args:
            request: 서비스 요청 (robot_id 포함)
            response: 서비스 응답
        
        Returns:
            response: 성공 여부와 메시지가 포함된 응답
        """
        robot_id = request.robot_id
        self.get_logger().info(f'\nActivateTalker 서비스 호출됨 (robot_id: {robot_id})')
        log("SERVICE", f"🔊 토커매니저 활성화 요청 수신 (robot_id: {robot_id})")
        
        try:
            # 토커매니저 활성화 및 로봇 ID 설정
            self.comm_manager.is_active = True
            self.comm_manager.current_robot_id = robot_id
            
            response.success = True
            response.message = f"토커매니저가 활성화되었습니다. 로봇 {robot_id}의 웨이크워드 감지를 시작합니다."
            log("SERVICE", f"✅ 토커매니저 활성화 완료 (robot_id: {robot_id})")
            return response
        except Exception as e:
            response.success = False
            response.message = f"토커매니저 활성화 중 오류 발생: {str(e)}"
            log("SERVICE", f"❌ 토커매니저 활성화 실패: {str(e)}")
            return response
    
    def deactivate_talker_callback(self, request, response):
        """
        토커매니저 비활성화 서비스 콜백
        
        Args:
            request: 서비스 요청 (robot_id 포함)
            response: 서비스 응답
        
        Returns:
            response: 성공 여부와 메시지가 포함된 응답
        """
        robot_id = request.robot_id
        self.get_logger().info(f'DeactivateTalker 서비스 호출됨 (robot_id: {robot_id})')
        log("SERVICE", f"🔇 토커매니저 비활성화 요청 수신 (robot_id: {robot_id})")
        
        try:
            # 토커매니저 비활성화
            self.comm_manager.is_active = False
            self.comm_manager.current_robot_id = robot_id
            
            response.success = True
            response.message = f"토커매니저가 비활성화되었습니다. 로봇 {robot_id}의 웨이크워드 감지를 중지합니다."
            log("SERVICE", f"✅ 토커매니저 비활성화 완료 (robot_id: {robot_id})")
            return response
        except Exception as e:
            response.success = False
            response.message = f"토커매니저 비활성화 중 오류 발생: {str(e)}"
            log("SERVICE", f"❌ 토커매니저 비활성화 실패: {str(e)}")
            return response
    
    def publish_face_expression(self, robot_id, expression_type):
        """
        얼굴 표정 메시지 발행
        
        Args:
            robot_id (str): 로봇 ID (예: "libo_a")
            expression_type (str): 표정 타입 ("normal", "listening", "speaking" 등)
        """
        msg = FaceExpression()
        msg.robot_id = robot_id
        msg.expression_type = expression_type
        
        # 표정 타입에 맞는 이모지 선택
        emoji = "😐"  # 기본 이모지
        if expression_type == "normal":
            emoji = "😊"
        elif expression_type == "listening":
            emoji = "👂"
        elif expression_type == "speaking":
            emoji = "🗣️"
            
        self.get_logger().info(f"FaceExpression 발행: robot_id={robot_id}, expression_type={expression_type}")
        log("FACE", f"{emoji} 얼굴 표정 변경: {robot_id} → {expression_type}")
        self.face_expr_pub.publish(msg)


def process_voice_command(comm_manager, talker_node, recognizer, client, robot_id):
    """
    웨이크워드 감지 후 음성 명령 처리 로직
    
    이 함수는 웨이크워드('리보야') 감지 후 실행되는 전체 음성 처리 로직을 구현합니다:
    1. 노이즈 레벨 조정
    2. 음성 수집 (침묵 감지 기능 포함)
    3. STT를 통한 음성->텍스트 변환
    4. LLM(OpenAI)을 통한 의도 분석
    5. 의도에 따른 적절한 액션 실행
    
    지원하는 의도:
    - pause_assist: 작업 일시중지
    - resume_assist: 작업 재개
    - start_gesture: 제스처 모드 시작
    - start_follow: 팔로우 모드 시작
    - get_mode: 현재 모드 확인
    - get_weight: 책 무게 확인
    - stop_assist: 작업 중지 및 복귀
    - ignore: 무시할 명령
    
    Args:
        comm_manager: 통신 관리자 인스턴스
        talker_node: TalkerNode 인스턴스
        recognizer: SpeechRecognition 인식기
        client: OpenAI API 클라이언트
        robot_id: 로봇 ID
        
    Returns:
        None
    """
    global CURRENT_MODE
    
    # 음성 수집 및 텍스트 변환 로직
    try:
        # 웨이크워드 응답 재생 없이 바로 리스닝 모드로 진행
        # 얼굴 표정은 이미 웨이크워드 감지 시 listening으로 설정되어 있음
        
        # 음성 수집 (침묵 감지)
        log("AUDIO", "음성 수집 시작...")
        audio_data, duration = collect_audio(comm_manager)
        if audio_data is None or len(audio_data) == 0:
            log("AUDIO", "유효한 음성 데이터가 수집되지 않았습니다.")
            return
        
        # 임시 WAV 파일 저장
        tmp_dir = "/tmp"
        os.makedirs(tmp_dir, exist_ok=True)
        tmp_wav = os.path.join(tmp_dir, f"voice_command_{int(time.time())}.wav")
        save_wav_file(tmp_wav, audio_data)
        log("AUDIO", f"음성 데이터 저장됨: {tmp_wav} ({len(audio_data)} bytes, {duration:.2f} sec)")
        
        # STT로 음성을 텍스트로 변환
        transcript = recognize_speech(recognizer, tmp_wav)
        if transcript is None:
            log("STT", "음성 인식 실패")
            talker_node.publish_voice_command(robot_id, "voice_command", "ignore")
            return
        
        log("STT", f"인식된 텍스트: '{transcript}'")
        
        # 의도 분석
        intent = analyze_intent(client, transcript)
        log("INTENT", f"분석된 의도: {intent}")
        
        # 로봇 응답을 위해 speaking 모드로 변경
        talker_node.publish_face_expression(robot_id, "speaking")
        
        # 의도에 따른 액션 실행
        if intent == "pause_assist":
            # 일시중지 명령 처리
            log("ACTION", "일시중지 명령 처리")
            talker_node.publish_talk_command(robot_id, "stop")
            talker_node.publish_voice_command(robot_id, "voice_command", "pause_assist")
            
        elif intent == "resume_assist":
            # 재개 명령 처리
            log("ACTION", "재개 명령 처리")
            talker_node.publish_talk_command(robot_id, "activate")
            talker_node.publish_voice_command(robot_id, "voice_command", "resume_assist")
            
        elif intent == "start_gesture":
            # 제스처 모드 시작
            log("ACTION", "제스처 모드 시작")
            CURRENT_MODE = GESTURE_MODE
            talker_node.publish_talk_command(robot_id, "activate")
            
            # DeactivateTracker.srv, ActivateGesture.srv 호출
            talker_node.deactivate_tracker(robot_id)
            talker_node.activate_gesture(robot_id)
            
            talker_node.publish_voice_command(robot_id, "voice_command", "start_gesture")
            
        elif intent == "start_follow":
            # 팔로우 모드 시작
            log("ACTION", "팔로우 모드 시작")
            CURRENT_MODE = FOLLOW_MODE
            talker_node.publish_talk_command(robot_id, "activate")
            
            # ActivateTracker.srv, DeactivateGesture.srv 호출
            talker_node.activate_tracker(robot_id)
            talker_node.deactivate_gesture(robot_id)
            
            talker_node.publish_voice_command(robot_id, "voice_command", "start_follow")
            
        elif intent == "get_mode":
            # 현재 모드 확인
            log("ACTION", "현재 모드 확인 명령 처리")
            
            # 현재 모드에 따라 동적 액션 전송
            if CURRENT_MODE == GESTURE_MODE:
                talker_node.publish_voice_command(robot_id, "voice_command", "mode_gesture")
            else:
                talker_node.publish_voice_command(robot_id, "voice_command", "mode_follow")
            
        elif intent == "get_weight":
            # 책 무게 확인 - ESP에서 발행하는 /weight_data 토픽에서 값 가져오기
            log("ACTION", "책 무게 확인 명령 처리")
            
            # 현재 저장된 무게 정보를 소수점 3자리까지 포맷팅
            weight_value = round(talker_node.current_weight, 3)
            weight_action = f"get_weight_{weight_value}"
            
            # VoiceCommand로 무게 정보 전송 (동적 액션)
            talker_node.publish_voice_command(robot_id, "voice_command", weight_action)
            
            # 현재 저장된 무게 정보 로그에 출력
            log("INFO", f"현재 무게 정보: {weight_value}{talker_node.weight_unit}")
            
        elif intent == "stop_assist":
            # 작업 중지 및 복귀
            log("ACTION", "작업 중지 및 복귀 명령 처리")
            talker_node.call_end_task(robot_id)
            talker_node.publish_voice_command(robot_id, "voice_command", "stop_assist")
            
        elif intent == "get_weather":
            # 날씨 정보 확인
            log("ACTION", "날씨 정보 확인 명령 처리")
            
            try:
                # OpenAI API를 사용하여 가산디지털단지역 날씨 정보 생성
                weather_prompt = """
                오늘 가산디지털단지역의 날씨 정보를 간단하게 전달하는 내용을 작성해주세요.
                실제 날씨 정보가 아닌 일반적인 날씨 안내 멘트로 작성하되, 
                "오늘 가산디지털단지 날씨는 맑고 기온은 25도 정도입니다. 외출하기 좋은 날씨네요!"와 같은 형식으로 
                자연스럽고 친근한 톤으로 작성해주세요.
                
                반드시 다음 JSON 형식으로만 출력해야 합니다:
                {"weather_info": "날씨 정보 내용"}
                """
                
                completion = client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": weather_prompt},
                        {"role": "user", "content": "가산디지털단지역 날씨 알려줘"}
                    ]
                )
                
                weather_response = completion.choices[0].message.content
                log("GPT", f"날씨 응답: {weather_response}")
                
                # JSON 파싱하여 weather_info 추출
                import json
                try:
                    weather_data = json.loads(weather_response)
                    weather_info = weather_data.get("weather_info", "날씨 정보를 가져올 수 없습니다.")
                except json.JSONDecodeError:
                    # JSON 파싱 실패 시 정규식으로 추출 시도
                    match = re.search(r'{"weather_info":\s*"([^"]+)"}', weather_response)
                    if match:
                        weather_info = match.group(1)
                    else:
                        weather_info = "날씨 정보를 처리할 수 없습니다."
                
                # 동적 TTS로 날씨 정보 전송
                talker_node.publish_voice_command(robot_id, "dynamic_tts", weather_info)
                log("WEATHER", f"날씨 정보 전송: {weather_info}")
                
            except Exception as e:
                log("ERROR", f"날씨 정보 처리 중 오류: {str(e)}")
                talker_node.publish_voice_command(robot_id, "voice_command", "ignore")
            
        elif intent == "ignore":
            # 무시할 명령
            log("ACTION", "무시 가능한 명령")
            talker_node.publish_voice_command(robot_id, "voice_command", "ignore")
            
        else:
            # 알 수 없는 의도
            log("ACTION", f"알 수 없는 의도: {intent}")
            talker_node.publish_voice_command(robot_id, "voice_command", "ignore")

        # 로봇 응답을 위해 5초 후 normal 모드로 변경 (비동기 처리)
        def delayed_normal_expression():
            try:
                time.sleep(5.0)  # 5초 대기
                talker_node.publish_face_expression(robot_id, "normal")
                log("FACE", f"😊 5초 후 얼굴 표정 복귀: {robot_id} → normal")
            except Exception as e:
                log("ERROR", f"지연된 얼굴 표정 변경 중 오류: {str(e)}")
        
        # 백그라운드 스레드로 실행 (시스템에 영향 주지 않음)
        normal_thread = threading.Thread(target=delayed_normal_expression)
        normal_thread.daemon = True  # 메인 프로그램 종료 시 함께 종료
        normal_thread.start()

    except Exception as e:
        log("ERROR", f"음성 명령 처리 중 오류 발생: {str(e)}")
        traceback.print_exc()
        try:
            talker_node.publish_voice_command(robot_id, "voice_command", "ignore")
        except:
            pass
    # 함수 종료 - 이제 명확한 처리 흐름으로 중복된 코드를 제거했습니다
    return


def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    # ========== 1. 통신 관리자 초기화 ==========
    log("INIT", "🚀 통신 관리자 초기화 중...")
    log("NETWORK", "📡 네트워크 설정 요약:")
    log("NETWORK", f"🎤 UDP 서버: {HARDWARE_HANDLER_IP}:{MIC_STREAM_PORT} - 마이크 스트림")
    comm_manager = CommunicationManager()
    log("STATUS", "⚠️ 토커매니저 초기 상태: 비활성화 (웨이크워드 감지 불가능 - ActivateTalker 서비스 호출 필요)")
    
    # UDP 수신기 시작 (비동기)
    udp_thread = comm_manager.start_udp_receiver()

    # ========== 2. ROS2 노드 생성 (VoiceCommand 메시지 발행용) ==========
    log("INIT", "ROS2 노드 생성 중...")
    talker_node = TalkerNode(comm_manager)
    
    # CommunicationManager에 TalkerNode 참조 설정
    comm_manager.talker_node = talker_node
    
    # ROS2 노드와 웨이크워드 감지를 병렬로 실행하기 위한 스레드 생성
    def ros_spin_thread():
        try:
            log("ROS", "ROS2 스핀 루프 시작")
            rclpy.spin(talker_node)
        except Exception as e:
            log("ERROR", f"ROS2 스핀 루프 오류: {str(e)}")
        finally:
            log("ROS", "ROS2 스핀 루프 종료")
    
    # ROS2 스핀 스레드 시작
    ros_thread = threading.Thread(target=ros_spin_thread)
    ros_thread.daemon = True
    ros_thread.start()
    log("ROS", "ROS2 스핀 스레드 시작됨 - 이제 '/voice_command' 토픽을 구독합니다")
    
    # ========== 3. Porcupine 웨이크워드 엔진 ==========
    log("INIT", "Porcupine 웨이크워드 엔진 초기화 중...")
    # 민감도 높게 설정 (0.0~1.0 사이, 기본값 0.5, 높을수록 더 민감함)
    SENSITIVITY = 0.7  # 민감도 증가
    log("CONFIG", f"웨이크워드 감지 민감도: {SENSITIVITY} (0.0~1.0, 높을수록 더 민감)")
    
    porcupine = pvporcupine.create(
        access_key=PICOVOICE_ACCESS_KEY,
        keyword_paths=[PORCUPINE_KEYWORD_PATH],
        model_path=PORCUPINE_MODEL_PATH,
        sensitivities=[SENSITIVITY]  # 키워드 감지 민감도 설정
    )
    mic_frame_length = int(porcupine.frame_length * (NATIVE_RATE / TARGET_RATE))
    log("CONFIG", f"프레임 길이: {mic_frame_length}, 원본 레이트: {NATIVE_RATE}Hz, 타겟 레이트: {TARGET_RATE}Hz")
    if comm_manager.is_active:
        log("WAKEWORD", "Ready for wakeword detection: '리보야'")
    else:
        log("WAKEWORD", "Wakeword detection DISABLED. Use ActivateTalker service to enable.")
    buffer = b''

    # ========== 4. OpenAI 클라이언트 ==========
    client = openai.OpenAI(api_key=OPENAI_API_KEY)
    recognizer = sr.Recognizer()

    try:
        while True:
            # --- UDP로부터 데이터 누적 ---
            while not comm_manager.buffer_queue.empty():
                buffer += comm_manager.buffer_queue.get()

            # --- mic_frame_length 단위로 처리 ---
            while len(buffer) >= mic_frame_length * 2:  # int16은 2bytes
                frame_bytes = buffer[:mic_frame_length * 2]
                buffer = buffer[mic_frame_length * 2:]

                # 전체 프레임이 있는지 확인
                if len(frame_bytes) >= mic_frame_length * 2:  # int16은 2bytes
                    pcm_native = struct.unpack_from("h" * mic_frame_length, frame_bytes)
                    audio_np = np.array(pcm_native, dtype=np.float32)
                    audio_resampled = resampy.resample(audio_np, NATIVE_RATE, TARGET_RATE)
                    # 길이가 512의 배수가 되도록 패딩 또는 잘라내기
                    target_length = 512 * ((len(audio_resampled) + 511) // 512)
                    if len(audio_resampled) < target_length:
                        # 패딩: 부족한 샘플을 0으로 채우기
                        audio_resampled = np.pad(audio_resampled, (0, target_length - len(audio_resampled)))
                    elif len(audio_resampled) > target_length:
                        # 잘라내기: 초과 샘플 제거
                        audio_resampled = audio_resampled[:target_length]
                    pcm_resampled = audio_resampled.astype(np.int16)
                
                # 상태가 변경되었을 때만 메시지 출력 (또는 60초마다 한 번)
                current_time = time.time()
                if (comm_manager.last_status != comm_manager.is_active) or (current_time - comm_manager.last_status_report_time > 60):
                    status = "활성화" if comm_manager.is_active else "비활성화"
                    robot_id = comm_manager.current_robot_id
                    log("STATUS", f"토커매니저 상태: {status} (웨이크워드 감지: {'켜짐' if comm_manager.is_active else '꺼짐'}, 로봇: {robot_id})")
                    comm_manager.last_status = comm_manager.is_active
                    comm_manager.last_status_report_time = current_time
                
                # ========== 4. 웨이크워드 검출 ==========
                # 활성화 상태일 때만 웨이크워드 감지 수행
                if comm_manager.is_active:
                    keyword_index = porcupine.process(pcm_resampled)
                    # 웨이크워드가 감지된 경우
                    if keyword_index >= 0:
                        robot_id = "libo_a"  # 기본 로봇 ID
                        log("WAKE", "🟢 Wakeword('리보야') 감지됨!")
                        
                        # 웨이크워드 감지 시 VoiceCommand 발행 (스피커에게 응답음 재생 요청)
                        talker_node.publish_voice_command(robot_id, "voice_command", "wake_response")
                        
                        # 웨이크워드 감지 시 'stop' 명령 바로 발행
                        log("COMMAND", f"TalkCommand 발행: robot_id={robot_id}, action=stop")
                        talker_node.publish_talk_command(robot_id, "stop")
                        
                        # 웨이크워드 감지 시 얼굴 표정을 바로 'listening'으로 변경
                        talker_node.publish_face_expression(robot_id, "listening")
                        
                        # 웨이크워드 이후 음성 명령 처리 함수 호출 (응답 재생 없이)
                        log("STT", "다음 명령을 말씀하세요... (최대 15초)")
                        process_voice_command(comm_manager, talker_node, recognizer, client, robot_id)
                        
                        # 웨이크워드 감지 후 음성 명령 처리 로직은 process_voice_command 함수로 이동되었음
                else:
                    # 비활성화 상태일 때는 웨이크워드 감지를 수행하지 않음
                    keyword_index = -1
                    # 30초마다 한 번씩만 디버깅 메시지 출력 (버퍼가 있을 경우)
                    if int(current_time) % 30 == 0 and len(buffer) > 0:
                        log("STATUS", "ℹ️ 토커매니저 비활성화 상태 - 웨이크워드 감지 중지됨 (ActivateTalker 서비스 필요)")
                    
                    # 비활성화 상태에서도 음성 명령 처리 가능 (직접 호출)
                    # 참고: 이 블록은 필요한 경우에만 추가 (예: 비활성화 상태에서도 특정 명령 인식 필요한 경우)
                    # process_voice_command(comm_manager, talker_node, recognizer, client, "libo_a")

            time.sleep(0.01)
            
    except KeyboardInterrupt:
        log("SYSTEM", "사용자에 의한 종료 요청됨.")
    except Exception as e:
        log("ERROR", f"예외 발생: {str(e)}")
    finally:
        log("CLEANUP", "프로그램 종료 중...")
        
        # 통신 관리자 정리
        if 'comm_manager' in locals():
            log("CLEANUP", "통신 관리자 리소스 정리 중...")
            comm_manager.cleanup()
            
            # 스레드 종료 대기
            if 'udp_thread' in locals() and udp_thread.is_alive():
                udp_thread.join(timeout=2.0)
                
        # ROS2 종료
        if 'talker_node' in locals():
            log("CLEANUP", "ROS2 노드 정리 중...")
            talker_node.destroy_node()
            
        if rclpy.ok():
            log("CLEANUP", "ROS2 종료 중...")
            rclpy.shutdown()
        
        # Porcupine 리소스 정리
        if 'porcupine' in locals():
            log("CLEANUP", "Porcupine 웨이크워드 엔진 정리 중...")
            porcupine.delete()
            
        log("SYSTEM", "프로그램이 안전하게 종료되었습니다.")

if __name__ == '__main__':
    main()
