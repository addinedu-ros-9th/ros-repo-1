import pyaudio
import os
import io
from datetime import datetime
import pytz
import numpy as np
import time
import rclpy
from rclpy.node import Node
from google.cloud import texttospeech
from libo_interfaces.msg import VoiceCommand
try:
    from pydub import AudioSegment
    PYDUB_AVAILABLE = True
except ImportError:
    print("pydub 라이브러리를 찾을 수 없습니다. MP3 재생이 제한될 수 있습니다.")
    PYDUB_AVAILABLE = False

# ======================== 상수 정의 =========================
CHANNELS = 1                           # 모노 채널
RATE = 24000                          # 샘플링 레이트
CHUNK = 1024                          # 청크 크기
FORMAT = pyaudio.paFloat32            # 32비트 부동소수점
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

# MP3 효과음 디렉토리 설정 (여러 가능성 검사)
possible_mp3_paths = [
    os.path.join(PROJECT_ROOT, "data", "mp3_effect_files"),  # 표준 경로
    os.path.join(PROJECT_ROOT, "ros_ws", "data", "mp3_effect_files"),  # ros_ws 내부 경로
    os.path.join(current_path, "../../../..", "data", "mp3_effect_files"),  # 상대 경로
    "/home/addinedu/Github_Repository/ros-repo-1/data/mp3_effect_files"  # 절대 경로
]

# 존재하는 첫 번째 경로 사용
MP3_EFFECTS_DIR = None
for path in possible_mp3_paths:
    if os.path.exists(path):
        MP3_EFFECTS_DIR = path
        print(f"MP3 효과음 디렉토리 발견: {MP3_EFFECTS_DIR}")
        break

if MP3_EFFECTS_DIR is None:
    print(f"[ERROR] MP3 효과음 디렉토리를 찾을 수 없습니다. 기본 경로 사용.")
    MP3_EFFECTS_DIR = os.path.join(PROJECT_ROOT, "data", "mp3_effect_files")
    # 디렉토리가 없으면 생성
    try:
        os.makedirs(MP3_EFFECTS_DIR, exist_ok=True)
        print(f"MP3 효과음 디렉토리 생성: {MP3_EFFECTS_DIR}")
    except Exception as e:
        print(f"MP3 효과음 디렉토리 생성 실패: {str(e)}")

# Google Cloud 인증 파일 경로 설정 (여러 가능성 검사)
possible_creds_paths = [
    os.path.join(PROJECT_ROOT, "data", "fleet-unison-452704-j5-31aaeff5ac33.json"),
    os.path.join(PROJECT_ROOT, "ros_ws", "data", "fleet-unison-452704-j5-31aaeff5ac33.json"),
    "/home/addinedu/Github_Repository/ros-repo-1/data/fleet-unison-452704-j5-31aaeff5ac33.json"
]

# 존재하는 첫 번째 경로 사용
GOOGLE_CREDS_PATH = None
for path in possible_creds_paths:
    if os.path.exists(path):
        GOOGLE_CREDS_PATH = path
        print(f"Google Cloud 인증 파일 발견: {GOOGLE_CREDS_PATH}")
        break

if GOOGLE_CREDS_PATH is None:
    print(f"[ERROR] Google Cloud 인증 파일을 찾을 수 없습니다.")
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
        
        self.is_running = True
        
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
            },
            
            # 음성 명령 응답 (리보야 부른 후 처리)
            "voice_command": {
                "wake_response": {"type": "mp3", "value": "ribo_response.mp3"},
                "pause_assist": {"type": "tts", "value": "네. 일시정지합니다."},
                "resume_assist": {"type": "tts", "value": "네, 다시 작업 하겠습니다."},
                "start_gesture": {"type": "tts", "value": "모션 제스쳐 모드를 시작합니다."},
                "start_follow": {"type": "tts", "value": "Follow 모드를 시작합니다."},
                "mode_gesture": {"type": "tts", "value": "현재 모션 제스쳐 모드 입니다."},
                "mode_follow": {"type": "tts", "value": "현재 휴먼 팔로우 모드 입니다."},
                "stop_assist": {"type": "tts", "value": "네, 복귀합니다."},
                "ignore": {"type": "tts", "value": "등록되지 않은 명령입니다."}
            },
            
            # 오류 처리 관련 명령
            "error": {
                "mp3_file_failed": {"type": "tts", "value": "효과음 파일을 재생할 수 없습니다."},
                "tts_failed": {"type": "tts", "value": "음성 합성에 실패했습니다."},
                "connection_failed": {"type": "tts", "value": "연결에 실패했습니다."},
                "general_error": {"type": "tts", "value": "오류가 발생했습니다."}
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
            category (str): 명령 카테고리 ('common', 'escort', 'delivery', 'assist', 'voice_command', 'error', 'mp3_effect', 'dynamic_tts')
            action (str): 카테고리 내 액션 이름 또는 파일명/텍스트
            
        Returns:
            bool: 명령 재생 성공 여부
        """
        try:
            # 특별 카테고리 처리
            if category == "mp3_effect":
                # action을 파일명으로 사용하여 MP3 재생
                print(f"[{get_kr_time()}][MP3] 동적 MP3 효과음 재생: {action}")
                return self.play_mp3_effect(action)
            
            elif category == "dynamic_tts":
                # action을 텍스트로 사용하여 TTS 재생
                print(f"[{get_kr_time()}][TTS] 동적 텍스트 TTS: {action}")
                return self.play_tts_response(action)
            
            # 일반 카테고리 처리
            elif category in self.voice_commands:
                command_dict = self.voice_commands[category]
                
                # get_weight_{무게} 동적 처리
                if category == "voice_command" and action.startswith("get_weight_"):
                    weight = action.replace("get_weight_", "")
                    return self.play_tts_response(f"현재 책 무게는 {weight}킬로그램 입니다.")
                
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
            
            # 파일이 존재하는지 확인
            if not os.path.exists(file_path):
                print(f"[{get_kr_time()}][ERROR] MP3 파일이 존재하지 않습니다: {file_path}")
                
                # 파일이 없는 경우 가능한 전체 경로 목록 출력
                print(f"[{get_kr_time()}][DEBUG] MP3_EFFECTS_DIR: {MP3_EFFECTS_DIR}")
                if os.path.exists(MP3_EFFECTS_DIR):
                    files = os.listdir(MP3_EFFECTS_DIR)
                    print(f"[{get_kr_time()}][DEBUG] 디렉토리 내 파일 목록: {files}")
                
                # TTS로 대체
                return self.play_tts_response(f"효과음 {file_name}을 찾을 수 없습니다.")
                
            print(f"[{get_kr_time()}][AUDIO] MP3 효과음 재생 시작: {file_name}")
            
            # pydub 라이브러리가 있으면 MP3 재생
            if PYDUB_AVAILABLE:
                try:
                    # MP3 파일 로드
                    sound = AudioSegment.from_mp3(file_path)
                    
                    # 24kHz로 리샘플링 (RATE와 일치하도록)
                    if sound.frame_rate != RATE:
                        sound = sound.set_frame_rate(RATE)
                    
                    # 모노로 변환 (채널 수가 다른 경우)
                    if sound.channels != CHANNELS:
                        sound = sound.set_channels(CHANNELS)
                        
                    # 볼륨 약간 증가 (3dB)
                    sound = sound + 3
                    
                    # float32 데이터로 변환
                    samples = np.array(sound.get_array_of_samples())
                    audio_float32 = samples.astype(np.float32) / 32768.0
                    
                    # 클리핑 방지
                    audio_float32 = np.clip(audio_float32, -1.0, 1.0)
                    
                    # 오디오 큐에 추가하여 재생
                    for i in range(0, len(audio_float32), CHUNK):
                        chunk = audio_float32[i:i + CHUNK]
                        if len(chunk) < CHUNK:
                            chunk = np.pad(chunk, (0, CHUNK - len(chunk)))
                        self.audio_queue.put(chunk.tobytes())
                    
                    print(f"[{get_kr_time()}][AUDIO] MP3 효과음 '{file_name}' 큐에 추가 완료 (길이: {len(audio_float32)} 샘플)")
                    return True
                except Exception as e:
                    print(f"[{get_kr_time()}][ERROR] MP3 처리 중 오류 발생: {str(e)}")
                    # 오류 발생 시 TTS로 대체
                    return self.play_tts_response(f"효과음 재생 중 오류가 발생했습니다.")
            else:
                # pydub이 없으면 TTS로 대체
                print(f"[{get_kr_time()}][WARNING] pydub 라이브러리 없음, TTS로 대체합니다.")
                return self.play_tts_response(f"효과음 {file_name}을 재생하려 했으나 pydub 라이브러리가 없습니다.")
            
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
            
        print(f"[{get_kr_time()}][SYSTEM] 프로그램 종료")

def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    speaker = SpeakerNode()
    
    try:
        rclpy.spin(speaker)
    except KeyboardInterrupt:
        print(f"[{get_kr_time()}][SYSTEM] 사용자에 의해 중지됨")
    finally:
        speaker.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
