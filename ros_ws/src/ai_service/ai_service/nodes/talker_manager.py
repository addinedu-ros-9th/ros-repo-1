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
from datetime import datetime
from dotenv import load_dotenv
from pydub import AudioSegment
from google.cloud import texttospeech
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from libo_interfaces.msg import VoiceCommand, TalkCommand
from libo_interfaces.srv import EndTask, ActivateTalker, DeactivateTalker


# ================== 네트워크/오디오 기본 설정 ==================
# 네트워크 설정
HARDWARE_HANDLER_IP = "0.0.0.0"      # 🖥️ Hardware Handler IP (UDP/TCP 서버 주소)
MIC_STREAM_PORT = 7010                 # 🎤 마이크 스트림 포트 (UDP 수신)
SPEAKER_PORT = 7002                    # 🔊 스피커 출력 포트 (TCP 서버)

# 오디오 설정
NATIVE_RATE = 48000                    # 🎵 원본 샘플링 레이트 (mic_streamer와 동일 int16)
TARGET_RATE = 16000                    # 🎯 웨이크워드 처리용 레이트
TTS_RATE = 24000                       # 🗣️ TTS 출력 레이트

print(f"[NETWORK CONFIG] 📡 UDP 서버: {HARDWARE_HANDLER_IP}:{MIC_STREAM_PORT} - 마이크 스트림 수신")
print(f"[NETWORK CONFIG] 🔌 TCP 서버: {HARDWARE_HANDLER_IP}:{SPEAKER_PORT} - 스피커 노드 연결 수신")

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

# ================== 음성 명령 응답 매핑 ==================
# 음성 명령 응답 매핑 - 카테고리별로 구성
VOICE_COMMANDS = {
    # 공통 음성 명령
    "common": {
        "power_on": {"type": "mp3", "value": "power_on.mp3"},                   # (전원 켜지는 소리 - 삐빅)
        "initialized": {"type": "mp3", "value": "robot_initialized.mp3"},       # (초기화 완료 소리 - 따리리리링)
        "charging": {"type": "tts", "value": "충전을 시작하겠습니다."},
        "battery_sufficient": {"type": "tts", "value": "배터리가 충분합니다. 대기모드로 전환합니다."},
        "depart_base": {"type": "tts", "value": "출발합니다~ (충전기를 뽑고)"},
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
        "lost_user": {"type": "tts", "value": "손님이 보이지 않습니다. 20초 후에 자동종료 됩니다."},
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
        "arrived_kiosk": {"type": "tts", "value": "어시스트를 시작하시면 QR 코드를 카메라 앞에 대주세요"},
        "qr_authenticated": {"type": "tts", "value": "QR 인증 완료! 어시스트를 시작하면 카메라 앞에서 대기 해주시길 바랍니다."},
        "no_person_5s": {"type": "tts", "value": "감지 실패!"},
        "person_detected": {"type": "tts", "value": "감지 성공!"},
        "called_by_staff": {"type": "mp3", "value": "ribo_response.mp3"},       # 네? / (삐빅)
        "pause": {"type": "tts", "value": "일시정지합니다."},
        "resume": {"type": "tts", "value": "어시스트를 재개합니다."},
        "return": {"type": "mp3", "value": "complete.mp3"},                     # 복귀하겠습니다. / (북귀음 소리 - 빠빕)
        "arrived_base": {"type": "tts", "value": "Base에 도착했습니다."}
    }
}
CHANNELS = 1
CHUNK = 2048                           # mic_streamer와 동일

# TCP 서버 설정
tcp_server = None
tcp_client = None

# ================== UDP/TCP 통신 관리 ==================
def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시

class CommunicationManager:
    def __init__(self):
        self.udp_sock = None
        self.tcp_server = None
        self.tcp_client = None
        self.buffer_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.tcp_ready = threading.Event()
        self.is_active = False  # 웨이크워드 및 명령 처리 활성화 상태 변수 - 기본값 False로 변경
        self.current_robot_id = "unknown"  # 현재 활성화/비활성화 요청한 로봇 ID
        self.last_status_report_time = 0  # 마지막 상태 출력 시간
        self.last_status = False  # 마지막 상태 기록
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

    def start_tcp_server(self):
        """TCP 서버 초기화 및 시작"""
        def _tcp_server():
            try:
                print(f"[{get_kr_time()}][TCP] 🔊 스피커 TCP 서버 초기화 중...")
                self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.tcp_server.bind((HARDWARE_HANDLER_IP, SPEAKER_PORT))
                self.tcp_server.listen(1)
                
                print(f"[{get_kr_time()}][TCP] 🎧 스피커 노드 연결 대기 중... ({HARDWARE_HANDLER_IP}:{SPEAKER_PORT})")
                print(f"[{get_kr_time()}][TCP] ⚙️  설정: TTS_RATE={TTS_RATE}Hz, CHUNK={CHUNK}")
                
                while not self.stop_event.is_set():
                    self.tcp_server.settimeout(1.0)  # 1초 타임아웃 설정
                    try:
                        self.tcp_client, addr = self.tcp_server.accept()
                        print(f"[{get_kr_time()}][TCP] ✅ 스피커 노드 연결됨: {addr} → {HARDWARE_HANDLER_IP}:{SPEAKER_PORT}")
                        self.tcp_ready.set()  # TCP 연결 완료 신호
                        
                        # 클라이언트 연결이 끊어질 때까지 대기
                        while not self.stop_event.is_set():
                            time.sleep(1)
                            try:
                                # 연결 상태 확인
                                self.tcp_client.send(b'')
                            except:
                                print(f"[{get_kr_time()}][TCP] 클라이언트 연결이 끊어짐")
                                self.tcp_ready.clear()
                                break
                                
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[{get_kr_time()}][TCP] 연결 오류: {str(e)}")
                        self.tcp_ready.clear()
                        time.sleep(1)  # 재시도 전 대기
                        
            except Exception as e:
                print(f"[{get_kr_time()}][TCP] 서버 오류: {str(e)}")
            finally:
                if self.tcp_client:
                    self.tcp_client.close()
                if self.tcp_server:
                    self.tcp_server.close()
                    
        thread = threading.Thread(target=_tcp_server)
        thread.daemon = True
        thread.start()
        return thread

    def send_audio_data(self, audio_data):
        """TTS 오디오 데이터를 TCP로 전송"""
        if not self.tcp_ready.is_set():
            print(f"[{get_kr_time()}][TCP] ⚠️ 스피커 노드가 연결되어 있지 않습니다.")
            return False
            
        try:
            # 데이터 크기 전송 (4바이트)
            total_size = len(audio_data) * 4  # float32는 4바이트
            kb_size = total_size / 1024
            print(f"[{get_kr_time()}][TCP] 📤 오디오 데이터 전송 시작: {kb_size:.2f}KB")
            self.tcp_client.send(total_size.to_bytes(4, byteorder='big'))
            
            # 청크 단위로 전송
            chunks_sent = 0
            for i in range(0, len(audio_data), CHUNK):
                chunk = audio_data[i:i + CHUNK]
                if len(chunk) < CHUNK:
                    chunk = np.pad(chunk, (0, CHUNK - len(chunk)))
                self.tcp_client.send(chunk.tobytes())
                chunks_sent += 1
                
                # 큰 오디오 데이터인 경우에만 진행 상황 표시
                if total_size > 100000 and chunks_sent % 20 == 0:
                    progress = min(100, int((i+CHUNK) * 100 / len(audio_data)))
                    print(f"[{get_kr_time()}][TCP] 🔄 오디오 전송 중: {progress}% 완료")
            
            print(f"[{get_kr_time()}][TCP] ✅ 오디오 데이터 전송 완료: {chunks_sent}개 청크")
            return True
            
        except Exception as e:
            print(f"[{get_kr_time()}][TCP] ❌ 전송 오류: {str(e)}")
            print(f"[{get_kr_time()}][TCP] 🔄 연결 상태 초기화 ({HARDWARE_HANDLER_IP}:{SPEAKER_PORT})")
            self.tcp_ready.clear()
            return False

    def play_mp3_file(self, file_name):
        """MP3 파일을 재생하여 TCP로 전송"""
        try:
            file_path = os.path.join(MP3_EFFECTS_DIR, file_name)
            if not os.path.exists(file_path):
                print(f"[{get_kr_time()}][ERROR] MP3 파일을 찾을 수 없습니다: {file_path}")
                return False
                
            print(f"[{get_kr_time()}][MP3] 파일 로드 중: {file_name}")
            
            try:
                # MP3 파일을 pydub로 직접 로드
                sound = AudioSegment.from_mp3(file_path)
                
                # 모노 변환 (필요시)
                if sound.channels > 1:
                    sound = sound.set_channels(1)
                
                # 샘플링 레이트 변환 (필요시)
                if sound.frame_rate != TTS_RATE:
                    sound = sound.set_frame_rate(TTS_RATE)
                
                # 16비트로 설정 (필요시)
                sound = sound.set_sample_width(2)
                
                # 오디오 데이터를 numpy 배열로 변환
                samples = np.array(sound.get_array_of_samples())
                audio_float32 = samples.astype(np.float32) / 32768.0  # int16 범위에서 float32로 변환
                
                # 볼륨 증가 (약 3데시벨 증가 = 약 1.4배 볼륨)
                volume_factor = 1.4  # 약 3dB 증가
                audio_float32 = audio_float32 * volume_factor
                
                # 클리핑 방지 (값이 1.0을 넘지 않도록)
                audio_float32 = np.clip(audio_float32, -1.0, 1.0)
                
                # float32 형식으로 오디오 데이터 전송
                print(f"[{get_kr_time()}][AUDIO] MP3 오디오 데이터 전송 중... (볼륨 3dB 증가)")
                success = self.send_audio_data(audio_float32)
                
                if success:
                    print(f"[{get_kr_time()}][AUDIO] MP3 전송 완료: {file_name}")
                else:
                    print(f"[{get_kr_time()}][AUDIO] ❌ MP3 전송 실패: {file_name}")
                
                return success
            
            except Exception as inner_e:
                # MP3 파일 로드 실패 시 TTS로 대체
                print(f"[{get_kr_time()}][WARNING] MP3 파일 로드 실패, TTS로 대체: {str(inner_e)}")
                return self.play_tts_response(f"효과음 {file_name}을 재생하려 했으나 실패했습니다.")
            
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] MP3 재생 오류: {str(e)}")
            return False
    
    def play_tts_response(self, text):
        """텍스트를 TTS로 변환하여 TCP로 전송"""
        try:
            print(f"[{get_kr_time()}][TTS] 음성 응답 생성 중: {text}")
            synthesis_input = texttospeech.SynthesisInput(text=text)
            
            voice = texttospeech.VoiceSelectionParams(
                language_code="ko-KR",
                name="ko-KR-Standard-A",
                ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
            )
            
            audio_config = texttospeech.AudioConfig(
                audio_encoding=texttospeech.AudioEncoding.LINEAR16,
                sample_rate_hertz=TTS_RATE,
            )
            
            tts_response = tts_client.synthesize_speech(
                input=synthesis_input,
                voice=voice,
                audio_config=audio_config
            )
            
            # 오디오 데이터를 float32로 변환
            audio_data = np.frombuffer(tts_response.audio_content, dtype=np.int16)
            audio_float32 = audio_data.astype(np.float32) / 32768.0
            
            # 볼륨 증가 (약 3데시벨 증가 = 약 1.4배 볼륨)
            # 3dB 증가는 약 1.4배(10^(3/20))의 amplitude 증가에 해당
            volume_factor = 1.4  # 약 3dB 증가
            audio_float32 = audio_float32 * volume_factor
            
            # 클리핑 방지 (값이 1.0을 넘지 않도록)
            audio_float32 = np.clip(audio_float32, -1.0, 1.0)
            
            # TCP를 통해 스피커 노드로 전송
            print(f"[{get_kr_time()}][AUDIO] TTS 오디오 데이터 전송 중... (볼륨 3dB 증가)")
            success = self.send_audio_data(audio_float32)
            
            if success:
                print(f"[{get_kr_time()}][AUDIO] TTS 전송 완료")
            else:
                print(f"[{get_kr_time()}][AUDIO] ❌ TTS 전송 실패")
            
            return success
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] TTS 생성/전송 오류: {str(e)}")
            return False
            
    def play_voice_command(self, category, action):
        """카테고리와 액션에 따라 음성 명령을 재생
        
        Args:
            category (str): 명령 카테고리 ('common', 'escort', 'delivery', 'assist')
            action (str): 카테고리 내 액션 이름
            
        Returns:
            bool: 성공 여부
        """
        try:
            if category not in VOICE_COMMANDS:
                print(f"[{get_kr_time()}][ERROR] 유효하지 않은 카테고리: {category}")
                return False
                
            if action not in VOICE_COMMANDS[category]:
                print(f"[{get_kr_time()}][ERROR] '{category}' 카테고리에 '{action}' 액션이 없습니다")
                return False
                
            command = VOICE_COMMANDS[category][action]
            print(f"[{get_kr_time()}][VOICE] 명령 실행: {category}.{action} ({command['type']})")
            
            if command['type'] == 'mp3':
                return self.play_mp3_file(command['value'])
            elif command['type'] == 'tts':
                return self.play_tts_response(command['value'])
            else:
                print(f"[{get_kr_time()}][ERROR] 알 수 없는 명령 타입: {command['type']}")
                return False
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] 음성 명령 실행 오류: {str(e)}")
            return False

    def cleanup(self):
        """모든 리소스 정리"""
        self.stop_event.set()
        if self.tcp_client:
            self.tcp_client.close()
        if self.tcp_server:
            self.tcp_server.close()
        if self.udp_sock:
            self.udp_sock.close()

class TalkerNode(Node):
    """
    ROS2 노드 클래스 - 음성 명령 토픽 구독자 및 제어 명령 발행자
    """
    def __init__(self, comm_manager):
        super().__init__('talker_node')
        
        # 통신 관리자 참조 저장
        self.comm_manager = comm_manager
        
        # 로깅
        self.get_logger().info('TalkerNode 초기화 중...')
        
        # 콜백 그룹 생성 - 동시에 여러 콜백을 처리하기 위함
        self.callback_group = ReentrantCallbackGroup()
        
        # VoiceCommand 토픽 구독
        self.voice_cmd_sub = self.create_subscription(
            VoiceCommand,
            '/voice_command',
            self.voice_command_callback,
            10
        )
        
        # TalkCommand 토픽 발행자
        self.talk_cmd_pub = self.create_publisher(
            TalkCommand,
            '/talk_command',
            10
        )
        
        # EndTask 서비스 클라이언트
        self.end_task_client = self.create_client(
            EndTask, 
            '/end_task',
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
        print(f"[{get_kr_time()}][SERVICE] ✅ ActivateTalker 서비스 서버 등록됨: /activate_talker")
        print(f"[{get_kr_time()}][SERVICE] ✅ DeactivateTalker 서비스 서버 등록됨: /deactivate_talker")
        
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
                print(f"[{get_kr_time()}][SERVICE] ✅ EndTask 서비스 호출 성공")
            else:
                self.get_logger().warning(f"EndTask 서비스 실패: {response.message if response.message else 'No message'}")
                print(f"[{get_kr_time()}][SERVICE] ⚠️ EndTask 서비스 호출 실패: {response.message}")
        except Exception as e:
            self.get_logger().error(f"EndTask 서비스 호출 중 오류 발생: {e}")
            print(f"[{get_kr_time()}][SERVICE] ❌ EndTask 서비스 호출 예외: {str(e)}")
    
    def publish_talk_command(self, robot_id, action):
        """TalkCommand 메시지 발행"""
        msg = TalkCommand()
        msg.robot_id = robot_id
        msg.action = action
        
        self.get_logger().info(f"TalkCommand 발행: robot_id={robot_id}, action={action}")
        self.talk_cmd_pub.publish(msg)
        
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
        self.get_logger().info(f'ActivateTalker 서비스 호출됨 (robot_id: {robot_id})')
        print(f"[{get_kr_time()}][SERVICE] 🔊 토커매니저 활성화 요청 수신 (robot_id: {robot_id})")
        
        try:
            # 토커매니저 활성화 및 로봇 ID 설정
            self.comm_manager.is_active = True
            self.comm_manager.current_robot_id = robot_id
            
            response.success = True
            response.message = f"토커매니저가 활성화되었습니다. 로봇 {robot_id}의 웨이크워드 감지를 시작합니다."
            print(f"[{get_kr_time()}][SERVICE] ✅ 토커매니저 활성화 완료 (robot_id: {robot_id})")
            return response
        except Exception as e:
            response.success = False
            response.message = f"토커매니저 활성화 중 오류 발생: {str(e)}"
            print(f"[{get_kr_time()}][SERVICE] ❌ 토커매니저 활성화 실패: {str(e)}")
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
        print(f"[{get_kr_time()}][SERVICE] 🔇 토커매니저 비활성화 요청 수신 (robot_id: {robot_id})")
        
        try:
            # 토커매니저 비활성화
            self.comm_manager.is_active = False
            self.comm_manager.current_robot_id = robot_id
            
            response.success = True
            response.message = f"토커매니저가 비활성화되었습니다. 로봇 {robot_id}의 웨이크워드 감지를 중지합니다."
            print(f"[{get_kr_time()}][SERVICE] ✅ 토커매니저 비활성화 완료 (robot_id: {robot_id})")
            return response
        except Exception as e:
            response.success = False
            response.message = f"토커매니저 비활성화 중 오류 발생: {str(e)}"
            print(f"[{get_kr_time()}][SERVICE] ❌ 토커매니저 비활성화 실패: {str(e)}")
            return response
        
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
        
        print(f"{'=' * 30} VoiceCommand 수신 {'=' * 30}")
        self.get_logger().info(f'VoiceCommand 수신: 로봇={robot_id}, 카테고리={category}, 액션={action}')
        
        # 카테고리와 액션으로 음성 명령 재생
        success = self.comm_manager.play_voice_command(category, action)
        
        if success:
            self.get_logger().info(f"음성 명령 '{category}.{action}' 성공적으로 실행됨")
        else:
            self.get_logger().warning(f"음성 명령 '{category}.{action}' 실행 실패")


def init_tcp_server():
    """TCP 서버 초기화 및 클라이언트 대기"""
    global tcp_server, tcp_client
    
    # TCP 서버 소켓 생성
    print(f"[{get_kr_time()}][TCP] 🔊 글로벌 TCP 서버 초기화 중...")
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind((HARDWARE_HANDLER_IP, SPEAKER_PORT))
    tcp_server.listen(1)
    
    print(f"[{get_kr_time()}][TCP] 🎧 스피커 노드 연결 대기 중... ({HARDWARE_HANDLER_IP}:{SPEAKER_PORT})")
    tcp_client, addr = tcp_server.accept()
    print(f"[{get_kr_time()}][TCP] ✅ 스피커 노드 연결됨: {addr} → {HARDWARE_HANDLER_IP}:{SPEAKER_PORT}")

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    # ========== 1. 통신 관리자 초기화 ==========
    print(f"[{get_kr_time()}][INIT] 🚀 통신 관리자 초기화 중...")
    print(f"[{get_kr_time()}][NETWORK] 📡 네트워크 설정 요약:")
    print(f"[{get_kr_time()}][NETWORK] 🎤 UDP 서버: {HARDWARE_HANDLER_IP}:{MIC_STREAM_PORT} - 마이크 스트림")
    print(f"[{get_kr_time()}][NETWORK] 🔊 TCP 서버: {HARDWARE_HANDLER_IP}:{SPEAKER_PORT} - 스피커 출력")
    comm_manager = CommunicationManager()
    print(f"[{get_kr_time()}][STATUS] ⚠️ 토커매니저 초기 상태: 비활성화 (웨이크워드 감지 불가능 - ActivateTalker 서비스 호출 필요)")
    
    # UDP 수신기 및 TCP 서버 시작 (비동기)
    udp_thread = comm_manager.start_udp_receiver()
    tcp_thread = comm_manager.start_tcp_server()

    # ========== 2. ROS2 노드 생성 (VoiceCommand 메시지 구독용) ==========
    print(f"[{get_kr_time()}][INIT] ROS2 노드 생성 중...")
    talker_node = TalkerNode(comm_manager)
    
    # ROS2 노드와 웨이크워드 감지를 병렬로 실행하기 위한 스레드 생성
    def ros_spin_thread():
        try:
            print(f"[{get_kr_time()}][ROS] ROS2 스핀 루프 시작")
            rclpy.spin(talker_node)
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] ROS2 스핀 루프 오류: {str(e)}")
        finally:
            print(f"[{get_kr_time()}][ROS] ROS2 스핀 루프 종료")
    
    # ROS2 스핀 스레드 시작
    ros_thread = threading.Thread(target=ros_spin_thread)
    ros_thread.daemon = True
    ros_thread.start()
    print(f"[{get_kr_time()}][ROS] ROS2 스핀 스레드 시작됨 - 이제 '/voice_command' 토픽을 구독합니다")
    
    # ========== 3. Porcupine 웨이크워드 엔진 ==========
    print(f"[{get_kr_time()}][INIT] Porcupine 웨이크워드 엔진 초기화 중...")
    porcupine = pvporcupine.create(
        access_key=PICOVOICE_ACCESS_KEY,
        keyword_paths=[PORCUPINE_KEYWORD_PATH],
        model_path=PORCUPINE_MODEL_PATH
    )
    mic_frame_length = int(porcupine.frame_length * (NATIVE_RATE / TARGET_RATE))
    print(f"[{get_kr_time()}][CONFIG] 프레임 길이: {mic_frame_length}, 원본 레이트: {NATIVE_RATE}Hz, 타겟 레이트: {TARGET_RATE}Hz")
    if comm_manager.is_active:
        print(f"[{get_kr_time()}][talker_manager] Ready for wakeword detection: '리보야'")
    else:
        print(f"[{get_kr_time()}][talker_manager] Wakeword detection DISABLED. Use ActivateTalker service to enable.")
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

                pcm_native = struct.unpack_from("h" * mic_frame_length, frame_bytes)
                audio_np = np.array(pcm_native, dtype=np.float32)
                audio_resampled = resampy.resample(audio_np, NATIVE_RATE, TARGET_RATE)
                pcm_resampled = audio_resampled.astype(np.int16)
                
                # 상태가 변경되었을 때만 메시지 출력 (또는 60초마다 한 번)
                current_time = time.time()
                if (comm_manager.last_status != comm_manager.is_active) or (current_time - comm_manager.last_status_report_time > 60):
                    status = "활성화" if comm_manager.is_active else "비활성화"
                    robot_id = comm_manager.current_robot_id
                    print(f"[{get_kr_time()}][STATUS] 토커매니저 상태: {status} (웨이크워드 감지: {'켜짐' if comm_manager.is_active else '꺼짐'}, 로봇: {robot_id})")
                    comm_manager.last_status = comm_manager.is_active
                    comm_manager.last_status_report_time = current_time
                
                # ========== 4. 웨이크워드 검출 ==========
                # 활성화 상태일 때만 웨이크워드 감지 수행
                if comm_manager.is_active:
                    keyword_index = porcupine.process(pcm_resampled)
                    # 웨이크워드가 감지된 경우
                    if keyword_index >= 0:
                        print(f"\n[{get_kr_time()}][WAKE] 🟢 Wakeword('리보야') 감지됨!")
                        
                        # 웨이크워드 감지 시 'stop' 명령 바로 발행
                        print(f"[{get_kr_time()}][COMMAND] TalkCommand 발행: robot_id=libo_a, action=stop")
                        talker_node.publish_talk_command("libo_a", "stop")
                        
                        # 웨이크워드 감지 시 응답 출력
                        print(f"[{get_kr_time()}][TTS] 웨이크워드 확인 응답 생성 중...")
                        
                        # 'called_by_staff' 액션을 사용하여 응답
                        if comm_manager.play_voice_command("assist", "called_by_staff"):
                            print(f"[{get_kr_time()}][AUDIO] 웨이크워드 응답 전송 완료 (MP3)")
                        else:
                            # MP3 파일 재생 실패 시 기본 TTS 사용
                                wake_response = "네? 무엇을 도와드릴까요?"
                                print(f"[{get_kr_time()}][TTS] TTS로 대체 응답 생성 중...")
                                
                                try:
                                    # TTS 직접 생성
                                    synthesis_input = texttospeech.SynthesisInput(text=wake_response)
                                    
                                    voice = texttospeech.VoiceSelectionParams(
                                        language_code="ko-KR",
                                        name="ko-KR-Standard-A",
                                        ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
                                    )
                                    
                                    audio_config = texttospeech.AudioConfig(
                                        audio_encoding=texttospeech.AudioEncoding.LINEAR16,
                                        sample_rate_hertz=TTS_RATE,
                                    )
                                    
                                    wake_tts_response = tts_client.synthesize_speech(
                                        input=synthesis_input,
                                        voice=voice,
                                        audio_config=audio_config
                                    )
                                    
                                    # 오디오 데이터를 float32로 변환
                                    wake_audio_data = np.frombuffer(wake_tts_response.audio_content, dtype=np.int16)
                                    wake_audio_float32 = wake_audio_data.astype(np.float32) / 32768.0
                                    
                                    # 볼륨 증가 (약 3데시벨 증가 = 약 1.4배 볼륨)
                                    volume_factor = 1.4  # 약 3dB 증가
                                    wake_audio_float32 = wake_audio_float32 * volume_factor
                                    
                                    # 클리핑 방지 (값이 1.0을 넘지 않도록)
                                    wake_audio_float32 = np.clip(wake_audio_float32, -1.0, 1.0)
                                    
                                    # TCP를 통해 스피커 노드로 전송
                                    print(f"[{get_kr_time()}][AUDIO] 웨이크워드 응답 전송 중... (볼륨 3dB 증가)")
                                    
                                    if comm_manager.send_audio_data(wake_audio_float32):
                                        print(f"[{get_kr_time()}][AUDIO] 웨이크워드 응답 전송 완료 (TTS)")
                                    else:
                                        print(f"[{get_kr_time()}][AUDIO] ❌ 웨이크워드 응답 전송 실패")
                                except Exception as e:
                                    print(f"[{get_kr_time()}][ERROR] 웨이크워드 TTS 오류: {str(e)}")
                                    
                            # 여기서 마이크 스트림 닫거나 모드 전환 필요 없음 (이미 UDP 입력중)
                        # 이후: 명령어 인식 단계
                        # ---- 명령어 인식(구글 STT API 활용) ----
                        print(f"[{get_kr_time()}][STT] 다음 명령을 말씀하세요... (최대 15초)")
                        
                        # [1] 음성 수집 및 노이즈 레벨 조정 (원본 샘플링 레이트 사용)
                        print(f"[{get_kr_time()}][AUDIO] 주변 소음 분석 중... (0.5초)")
                        print(f"[{get_kr_time()}][CONFIG] 음성 인식을 위해 원본 레이트({NATIVE_RATE}Hz) 사용")
                        
                        # WAV 파일로 현재 버퍼의 데이터 저장 (임시)
                        noise_wav = os.path.join(PROJECT_ROOT, "temp_noise.wav")
                        collected = b''
                        
                        # 노이즈 분석을 위한 데이터 수집 (0.5초)
                        start = time.time()
                        while time.time() - start < 0.5:  # 0.5초 동안 데이터 수집
                            if not comm_manager.buffer_queue.empty():
                                collected += comm_manager.buffer_queue.get()
                        
                        # WAV 파일로 저장 - 원본 샘플링 레이트(NATIVE_RATE) 사용
                        with wave.open(noise_wav, 'wb') as wf:
                            wf.setnchannels(CHANNELS)
                            wf.setsampwidth(2)  # 16-bit
                            wf.setframerate(NATIVE_RATE)
                            wf.writeframes(collected)
                        
                        # 노이즈 레벨 조정
                        with sr.AudioFile(noise_wav) as source:
                            recognizer.adjust_for_ambient_noise(source, duration=0.5)
                            print(f"[{get_kr_time()}][AUDIO] 노이즈 레벨 조정 완료")
                        
                        os.remove(noise_wav)  # 임시 파일 삭제
                        
                        # [2] 실제 음성 수집 시작 (침묵 감지 기능 추가)
                        print(f"[{get_kr_time()}][RECORD] 음성 수집 시작... (최대 15초, 침묵 감지시 자동 종료)")
                        collected = b''
                        start = time.time()
                        MAX_RECORD_TIME = 15.0  # 최대 15초
                        SILENCE_THRESHOLD = 300  # 침묵 감지 임계값 (RMS)
                        SILENCE_DURATION = 1.5  # 침묵이 지속되어야 하는 시간(초)
                        
                        last_active_time = time.time()  # 마지막으로 소리가 감지된 시간
                        has_speech_started = False  # 음성이 시작되었는지 여부
                        
                        try:
                            while time.time() - start < MAX_RECORD_TIME:
                                if not comm_manager.buffer_queue.empty():
                                    data = comm_manager.buffer_queue.get()
                                    collected += data
                                    
                                    # 현재 청크의 소리 크기 측정 (RMS)
                                    if len(data) >= CHUNK * 2:  # 최소 1개 청크 이상
                                        pcm = struct.unpack_from("h" * (len(data) // 2), data)
                                        rms = np.sqrt(np.mean(np.square(pcm)))
                                        
                                        # 소리가 임계값보다 크면 활동으로 간주
                                        if rms > SILENCE_THRESHOLD:
                                            last_active_time = time.time()
                                            if not has_speech_started and len(collected) > CHUNK * 10:  # 처음 몇 청크는 노이즈일 수 있으므로 건너뜀
                                                has_speech_started = True
                                                print(f"[{get_kr_time()}][RECORD] 🗣️ 음성 감지됨 (RMS: {rms:.1f})")
                                    
                                    # 로그 출력
                                    if len(collected) % (CHUNK * 10) == 0:
                                        elapsed = time.time() - start
                                        print(f"[{get_kr_time()}][RECORD] 수집된 데이터: {len(collected)} bytes (경과 시간: {elapsed:.1f}초)")
                                    
                                    # 침묵 감지 로직: 음성이 시작된 후 일정 시간동안 침묵이 계속되면 녹음 종료
                                    if has_speech_started and time.time() - last_active_time > SILENCE_DURATION:
                                        print(f"[{get_kr_time()}][RECORD] ⏹️ 침묵 감지: {SILENCE_DURATION}초 동안 소리가 없어 녹음 종료")
                                        break
                                        
                                else:
                                    time.sleep(0.01)
                        except Exception as e:
                            print(f"[{get_kr_time()}][ERROR] 음성 수집 중 오류: {str(e)}")
                        
                        duration = time.time() - start
                        print(f"[{get_kr_time()}][RECORD] 음성 수집 완료. 총 {len(collected)} bytes, 소요 시간: {duration:.1f}초")

                        # [2] WAV파일로 저장 → SpeechRecognition에서 로드
                        tmp_wav = os.path.join(PROJECT_ROOT, "temp_cmd.wav")
                        wf = wave.open(tmp_wav, 'wb')
                        wf.setnchannels(CHANNELS)
                        wf.setsampwidth(2)
                        wf.setframerate(NATIVE_RATE)
                        wf.writeframes(collected)
                        wf.close()

                        with sr.AudioFile(tmp_wav) as source:
                            print(f"[{get_kr_time()}][STT] 음성 파일 로드 완료. 구글 STT API 호출 중...")
                            audio = recognizer.record(source)
                            try:
                                transcript = recognizer.recognize_google(audio, language="ko-KR")
                                print(f"[{get_kr_time()}][STT] 사용자 발화: {transcript}")
                            except sr.UnknownValueError:
                                transcript = None
                                print(f"[{get_kr_time()}][STT] ❌ 음성 인식 실패 (음성을 감지할 수 없음)")
                                # 음성이 감지되지 않았을 때 사용자에게 TTS로 알림
                                comm_manager.play_tts_response("음성이 감지되지 않았습니다. 다시 불러주세요.")
                                print(f"[{get_kr_time()}][TTS] 음성 감지 실패 안내 메시지 재생")
                            except Exception as e:
                                transcript = None
                                print(f"[{get_kr_time()}][STT] ❌ STT 오류: {e}")
                                # 기타 오류 발생 시에도 안내
                                comm_manager.play_tts_response("음성 인식 중 오류가 발생했습니다. 다시 시도해주세요.")
                                print(f"[{get_kr_time()}][TTS] 음성 인식 오류 안내 메시지 재생")

                        os.remove(tmp_wav)

                        # [3] OpenAI로 의도 분석
                        if transcript:
                            system_prompt = (
                                "당신은 로봇의 음성 명령을 분석하는 AI입니다.\n"
                                "사용자의 발화를 듣고, 아래 4가지 의도 중 하나로 분류하세요.\n\n"
                                "- pause_follow: '잠깐 멈춰', '멈춰봐' 등 일시정지 명령\n"
                                "- resume_follow: '다시 따라와', '다시 시작해' 등 팔로윙 재개 명령\n"
                                "- stop_follow: '어시스트 그만하고 복귀해', '그만' 등 어시스트 종료 명령\n"
                                "- ignore: '고마워', '아니야' 등 기타 대화나 무시해도 되는 표현\n\n"
                                "결과는 반드시 다음 JSON 형식으로만 출력해야 합니다:\n"
                                '{"intent": "..."}'
                            )
                            completion = client.chat.completions.create(
                                model="gpt-3.5-turbo",
                                messages=[
                                    {"role": "system", "content": system_prompt},
                                    {"role": "user", "content": transcript}
                                ]
                            )
                            ai_text = completion.choices[0].message.content
                            print(f"[{get_kr_time()}][GPT] AI 응답: {ai_text}")
                            match = re.search(r'{"intent":\s*"(\w+)"}', ai_text)
                            if match:
                                intent = match.group(1)
                                print(f"[{get_kr_time()}][INTENT] 감지된 의도: {intent}")
                            else:
                                intent = "ignore"
                                print(f"[{get_kr_time()}][INTENT] ⚠️ 의도 분석 실패, 기본값 'ignore' 사용")

                            # 명령어에 따라 적절한 액션 실행
                            robot_id = "libo_a"  # 기본 로봇 ID
                            
                            # 의도에 따라 음성 응답 및 명령 발행
                            if intent == "resume_follow":
                                # 다시 따라와: TalkCommand 메시지 발행 (robot_id, "follow")
                                print(f"[{get_kr_time()}][RESPONSE] '다시 따라와' 명령 처리")
                                success = comm_manager.play_tts_response("네, 다시 따라가겠습니다.")
                                if success:
                                    # Talk Command 발행
                                    talker_node.publish_talk_command(robot_id, "follow")
                                    
                            elif intent == "pause_follow" or intent == "ignore":
                                # 멈춰 또는 무시: TalkCommand 메시지 발행 (robot_id, "stop")
                                print(f"[{get_kr_time()}][RESPONSE] '{intent}' 명령 처리")
                                if intent == "pause_follow":
                                    success = comm_manager.play_tts_response("네, 잠시 멈추겠습니다.")
                                else:
                                    success = comm_manager.play_tts_response("등록되지 않은 명령어 입니다.")
                                if success:
                                    talker_node.publish_talk_command(robot_id, "stop")
                                    
                            elif intent == "stop_follow":
                                # 어시스트 종료: EndTask 서비스 호출
                                print(f"[{get_kr_time()}][RESPONSE] '어시스트 종료' 명령 처리")
                                success = comm_manager.play_tts_response("네, 어시스트를 종료하고 복귀하겠습니다.")
                                if success:
                                    # EndTask 서비스 호출
                                    talker_node.call_end_task(robot_id)
                            
                            # 성공 여부에 따른 로그
                            if success:
                                print(f"[{get_kr_time()}][AUDIO] 음성 응답 전송 완료")
                            else:
                                print(f"[{get_kr_time()}][AUDIO] ❌ 음성 응답 전송 실패")
                                
                            print(f"[{get_kr_time()}][SYSTEM] '리보야' 이후 명령 처리 완료, 다시 웨이크워드 대기 중...")
                else:
                    # 비활성화 상태일 때는 웨이크워드 감지를 수행하지 않음
                    keyword_index = -1
                    # 30초마다 한 번씩만 디버깅 메시지 출력 (버퍼가 있을 경우)
                    if int(current_time) % 30 == 0 and len(buffer) > 0:
                        print(f"[{get_kr_time()}][STATUS] ℹ️ 토커매니저 비활성화 상태 - 웨이크워드 감지 중지됨 (ActivateTalker 서비스 필요)")
                    
                    # 노이즈 분석을 위한 데이터 수집 (0.5초)
                    start = time.time()
                    while time.time() - start < 0.5:  # 0.5초 동안 데이터 수집
                        if not comm_manager.buffer_queue.empty():
                            collected += comm_manager.buffer_queue.get()
                    
                    # WAV 파일로 저장 - 원본 샘플링 레이트(NATIVE_RATE) 사용
                    with wave.open(noise_wav, 'wb') as wf:
                        wf.setnchannels(CHANNELS)
                        wf.setsampwidth(2)  # 16-bit
                        wf.setframerate(NATIVE_RATE)
                        wf.writeframes(collected)
                    
                    # 노이즈 레벨 조정
                    with sr.AudioFile(noise_wav) as source:
                        recognizer.adjust_for_ambient_noise(source, duration=0.5)
                        print(f"[{get_kr_time()}][AUDIO] 노이즈 레벨 조정 완료")
                    
                    os.remove(noise_wav)  # 임시 파일 삭제
                    
                    # [2] 실제 음성 수집 시작 (침묵 감지 기능 추가)
                    print(f"[{get_kr_time()}][RECORD] 음성 수집 시작... (최대 15초, 침묵 감지시 자동 종료)")
                    collected = b''
                    start = time.time()
                    MAX_RECORD_TIME = 15.0  # 최대 15초
                    SILENCE_THRESHOLD = 300  # 침묵 감지 임계값 (RMS)
                    SILENCE_DURATION = 1.5  # 침묵이 지속되어야 하는 시간(초)
                    
                    last_active_time = time.time()  # 마지막으로 소리가 감지된 시간
                    has_speech_started = False  # 음성이 시작되었는지 여부
                    
                    try:
                        while time.time() - start < MAX_RECORD_TIME:
                            if not comm_manager.buffer_queue.empty():
                                data = comm_manager.buffer_queue.get()
                                collected += data
                                
                                # 현재 청크의 소리 크기 측정 (RMS)
                                if len(data) >= CHUNK * 2:  # 최소 1개 청크 이상
                                    pcm = struct.unpack_from("h" * (len(data) // 2), data)
                                    rms = np.sqrt(np.mean(np.square(pcm)))
                                    
                                    # 소리가 임계값보다 크면 활동으로 간주
                                    if rms > SILENCE_THRESHOLD:
                                        last_active_time = time.time()
                                        if not has_speech_started and len(collected) > CHUNK * 10:  # 처음 몇 청크는 노이즈일 수 있으므로 건너뜀
                                            has_speech_started = True
                                            print(f"[{get_kr_time()}][RECORD] 🗣️ 음성 감지됨 (RMS: {rms:.1f})")
                                
                                # 로그 출력
                                if len(collected) % (CHUNK * 10) == 0:
                                    elapsed = time.time() - start
                                    print(f"[{get_kr_time()}][RECORD] 수집된 데이터: {len(collected)} bytes (경과 시간: {elapsed:.1f}초)")
                                
                                # 침묵 감지 로직: 음성이 시작된 후 일정 시간동안 침묵이 계속되면 녹음 종료
                                if has_speech_started and time.time() - last_active_time > SILENCE_DURATION:
                                    print(f"[{get_kr_time()}][RECORD] ⏹️ 침묵 감지: {SILENCE_DURATION}초 동안 소리가 없어 녹음 종료")
                                    break
                                    
                            else:
                                time.sleep(0.01)
                    except Exception as e:
                        print(f"[{get_kr_time()}][ERROR] 음성 수집 중 오류: {str(e)}")
                    
                    duration = time.time() - start
                    print(f"[{get_kr_time()}][RECORD] 음성 수집 완료. 총 {len(collected)} bytes, 소요 시간: {duration:.1f}초")

                    # [2] WAV파일로 저장 → SpeechRecognition에서 로드
                    tmp_wav = os.path.join(PROJECT_ROOT, "temp_cmd.wav")
                    wf = wave.open(tmp_wav, 'wb')
                    wf.setnchannels(CHANNELS)
                    wf.setsampwidth(2)
                    wf.setframerate(NATIVE_RATE)
                    wf.writeframes(collected)
                    wf.close()

                    with sr.AudioFile(tmp_wav) as source:
                        print(f"[{get_kr_time()}][STT] 음성 파일 로드 완료. 구글 STT API 호출 중...")
                        audio = recognizer.record(source)
                        try:
                            transcript = recognizer.recognize_google(audio, language="ko-KR")
                            print(f"[{get_kr_time()}][STT] 사용자 발화: {transcript}")
                        except sr.UnknownValueError:
                            transcript = None
                            print(f"[{get_kr_time()}][STT] ❌ 음성 인식 실패 (음성을 감지할 수 없음)")
                            # 음성이 감지되지 않았을 때 사용자에게 TTS로 알림
                            comm_manager.play_tts_response("음성이 감지되지 않았습니다. 다시 불러주세요.")
                            print(f"[{get_kr_time()}][TTS] 음성 감지 실패 안내 메시지 재생")
                        except Exception as e:
                            transcript = None
                            print(f"[{get_kr_time()}][STT] ❌ STT 오류: {e}")
                            # 기타 오류 발생 시에도 안내
                            comm_manager.play_tts_response("음성 인식 중 오류가 발생했습니다. 다시 시도해주세요.")
                            print(f"[{get_kr_time()}][TTS] 음성 인식 오류 안내 메시지 재생")

                    os.remove(tmp_wav)

                    # [3] OpenAI로 의도 분석
                    if transcript:
                        system_prompt = (
                            "당신은 로봇의 음성 명령을 분석하는 AI입니다.\n"
                            "사용자의 발화를 듣고, 아래 4가지 의도 중 하나로 분류하세요.\n\n"
                            "- pause_follow: '잠깐 멈춰', '멈춰봐' 등 일시정지 명령\n"
                            "- resume_follow: '다시 따라와', '다시 시작해' 등 팔로윙 재개 명령\n"
                            "- stop_follow: '작업 그만하고 복귀해', '작업그만' 등 어시스트 또는 에스코팅 종료 명령\n"
                            "- ignore: '고마워', '아니야' 등 기타 대화나 무시해도 되는 표현\n\n"
                            "결과는 반드시 다음 JSON 형식으로만 출력해야 합니다:\n"
                            '{"intent": "..."}'
                        )
                        completion = client.chat.completions.create(
                            model="gpt-3.5-turbo",
                            messages=[
                                {"role": "system", "content": system_prompt},
                                {"role": "user", "content": transcript}
                            ]
                        )
                        ai_text = completion.choices[0].message.content
                        print(f"[{get_kr_time()}][GPT] AI 응답: {ai_text}")
                        match = re.search(r'{"intent":\s*"(\w+)"}', ai_text)
                        if match:
                            intent = match.group(1)
                            print(f"[{get_kr_time()}][INTENT] 감지된 의도: {intent}")
                        else:
                            intent = "ignore"
                            print(f"[{get_kr_time()}][INTENT] ⚠️ 의도 분석 실패, 기본값 'ignore' 사용")

                        # 명령어에 따라 적절한 액션 실행
                        robot_id = "libo_a"  # 기본 로봇 ID
                        
                        # 의도에 따라 음성 응답 및 명령 발행
                        if intent == "resume_follow":
                            # 다시 따라와: TalkCommand 메시지 발행 (robot_id, "follow")
                            print(f"[{get_kr_time()}][RESPONSE] '다시 따라와' 명령 처리")
                            success = comm_manager.play_tts_response("네, 다시 따라가겠습니다.")
                            if success:
                                # Talk Command 발행
                                talker_node.publish_talk_command(robot_id, "follow")
                                
                        elif intent == "pause_follow" or intent == "ignore":
                            # 멈춰 또는 무시: TalkCommand 메시지 발행 (robot_id, "stop")
                            print(f"[{get_kr_time()}][RESPONSE] '{intent}' 명령 처리")
                            if intent == "pause_follow":
                                success = comm_manager.play_tts_response("네, 잠시 멈추겠습니다.")
                            else:
                                success = comm_manager.play_tts_response("등록되지 않은 명령어 입니다.")
                            if success:
                                talker_node.publish_talk_command(robot_id, "stop")
                                
                        elif intent == "stop_follow":
                            # 어시스트 종료: EndTask 서비스 호출
                            print(f"[{get_kr_time()}][RESPONSE] '어시스트 종료' 명령 처리")
                            success = comm_manager.play_tts_response("네, 어시스트를 종료하고 복귀하겠습니다.")
                            if success:
                                # EndTask 서비스 호출
                                talker_node.call_end_task(robot_id)
                        
                        # 성공 여부에 따른 로그
                        if success:
                            print(f"[{get_kr_time()}][AUDIO] 음성 응답 전송 완료")
                        else:
                            print(f"[{get_kr_time()}][AUDIO] ❌ 음성 응답 전송 실패")
                            
                        print(f"[{get_kr_time()}][SYSTEM] '리보야' 이후 명령 처리 완료, 다시 웨이크워드 대기 중...")

            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print(f"[{get_kr_time()}][SYSTEM] 사용자에 의한 종료 요청됨.")
    except Exception as e:
        print(f"[{get_kr_time()}][ERROR] 예외 발생: {str(e)}")
    finally:
        print(f"[{get_kr_time()}][CLEANUP] 프로그램 종료 중...")
        
        # 통신 관리자 정리
        if 'comm_manager' in locals():
            print(f"[{get_kr_time()}][CLEANUP] 통신 관리자 리소스 정리 중...")
            comm_manager.cleanup()
            
            # 스레드 종료 대기
            if 'udp_thread' in locals() and udp_thread.is_alive():
                udp_thread.join(timeout=2.0)
            if 'tcp_thread' in locals() and tcp_thread.is_alive():
                tcp_thread.join(timeout=2.0)
                
        # ROS2 종료
        if 'talker_node' in locals():
            print(f"[{get_kr_time()}][CLEANUP] ROS2 노드 정리 중...")
            talker_node.destroy_node()
            
        if rclpy.ok():
            print(f"[{get_kr_time()}][CLEANUP] ROS2 종료 중...")
            rclpy.shutdown()
        
        # Porcupine 리소스 정리
        if 'porcupine' in locals():
            print(f"[{get_kr_time()}][CLEANUP] Porcupine 웨이크워드 엔진 정리 중...")
            porcupine.delete()
            
        print(f"[{get_kr_time()}][SYSTEM] 프로그램이 안전하게 종료되었습니다.")

if __name__ == '__main__':
    main()
