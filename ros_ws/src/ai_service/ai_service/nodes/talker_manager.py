import os
from dotenv import load_dotenv
import socket
import threading
import queue
import struct
import numpy as np
import pvporcupine
import resampy
import openai
import time
import speech_recognition as sr
from datetime import datetime
import pytz
from google.cloud import texttospeech
import wave
import sys

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

print(f"프로젝트 루트 경로: {PROJECT_ROOT}")
env_path = os.path.join(PROJECT_ROOT, '.env')
print(f".env 파일 경로: {env_path}")

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

# ================== 네트워크/오디오 기본 설정 ==================
HAEDWARE_HANDLER_IP = "127.0.0.1"       # Hardware Handler IP
AI_SERVICE_IP = "127.0.0.1"            # AI Service IP
MIC_STREAM_PORT = 7000                 # 마이크 스트림 포트 (UDP)
SPEAKER_PORT = 7002                    # 스피커 출력 포트 (TCP)
NATIVE_RATE = 48000                    # mic_streamer와 동일 int16
TARGET_RATE = 16000                    # 웨이크워드 처리용
TTS_RATE = 24000                      # TTS 출력 레이트
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

    def start_udp_receiver(self):
        """UDP 수신기 초기화 및 시작"""
        def _udp_receiver():
            try:
                self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock.bind((HAEDWARE_HANDLER_IP, MIC_STREAM_PORT))
                print(f"[{get_kr_time()}][UDP] 마이크 스트림 수신 대기 중... ({HAEDWARE_HANDLER_IP}:{MIC_STREAM_PORT})")
                
                received_count = 0
                start_time = time.time()
                
                while not self.stop_event.is_set():
                    data, _ = self.udp_sock.recvfrom(CHUNK * 2)
                    self.buffer_queue.put(data)
                    received_count += 1
                    
                    if received_count % 1000 == 0:
                        elapsed = time.time() - start_time
                        rate = received_count / elapsed
                        print(f"[{get_kr_time()}][UDP] 수신 통계: {received_count}개 패킷, {rate:.2f} packets/sec")
            except Exception as e:
                print(f"[{get_kr_time()}][UDP] 오류 발생: {str(e)}")
            finally:
                if self.udp_sock:
                    self.udp_sock.close()
                    
        thread = threading.Thread(target=_udp_receiver)
        thread.daemon = True
        thread.start()
        return thread

    def start_tcp_server(self):
        """TCP 서버 초기화 및 시작"""
        def _tcp_server():
            try:
                self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.tcp_server.bind((AI_SERVICE_IP, SPEAKER_PORT))
                self.tcp_server.listen(1)
                
                print(f"[{get_kr_time()}][TCP] 스피커 노드 연결 대기 중... (포트: {SPEAKER_PORT})")
                
                while not self.stop_event.is_set():
                    self.tcp_server.settimeout(1.0)  # 1초 타임아웃 설정
                    try:
                        self.tcp_client, addr = self.tcp_server.accept()
                        print(f"[{get_kr_time()}][TCP] 스피커 노드 연결됨: {addr}")
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
            self.tcp_client.send(total_size.to_bytes(4, byteorder='big'))
            
            # 청크 단위로 전송
            for i in range(0, len(audio_data), CHUNK):
                chunk = audio_data[i:i + CHUNK]
                if len(chunk) < CHUNK:
                    chunk = np.pad(chunk, (0, CHUNK - len(chunk)))
                self.tcp_client.send(chunk.tobytes())
                
            return True
            
        except Exception as e:
            print(f"[{get_kr_time()}][TCP] 전송 오류: {str(e)}")
            self.tcp_ready.clear()
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

def init_tcp_server():
    """TCP 서버 초기화 및 클라이언트 대기"""
    global tcp_server, tcp_client
    
    # TCP 서버 소켓 생성
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind((AI_SERVICE_IP, SPEAKER_PORT))
    tcp_server.listen(1)
    
    print(f"[{get_kr_time()}][TCP] 스피커 노드 연결 대기 중... (포트: {SPEAKER_PORT})")
    tcp_client, addr = tcp_server.accept()
    print(f"[{get_kr_time()}][TCP] 스피커 노드 연결됨: {addr}")

def main():
    # ========== 1. 통신 관리자 초기화 ==========
    print(f"[{get_kr_time()}][INIT] 통신 관리자 초기화 중...")
    comm_manager = CommunicationManager()
    
    # UDP 수신기 및 TCP 서버 시작 (비동기)
    udp_thread = comm_manager.start_udp_receiver()
    tcp_thread = comm_manager.start_tcp_server()

    # ========== 2. Porcupine 웨이크워드 엔진 ==========
    print(f"[{get_kr_time()}][INIT] Porcupine 웨이크워드 엔진 초기화 중...")
    porcupine = pvporcupine.create(
        access_key=PICOVOICE_ACCESS_KEY,
        keyword_paths=[PORCUPINE_KEYWORD_PATH],
        model_path=PORCUPINE_MODEL_PATH
    )
    mic_frame_length = int(porcupine.frame_length * (NATIVE_RATE / TARGET_RATE))
    print(f"[{get_kr_time()}][CONFIG] 프레임 길이: {mic_frame_length}, 원본 레이트: {NATIVE_RATE}Hz, 타겟 레이트: {TARGET_RATE}Hz")
    print(f"[{get_kr_time()}][talker_manager] Ready for wakeword detection: '리보야'")
    buffer = b''

    # ========== 3. OpenAI 클라이언트 ==========
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

                # ========== 4. 웨이크워드 검출 ==========
                keyword_index = porcupine.process(pcm_resampled)
                if keyword_index >= 0:
                    print(f"\n[{get_kr_time()}][WAKE] 🟢 Wakeword('리보야') 감지됨!")
                    print(f"[{get_kr_time()}][AUDIO] 현재 버퍼 크기: {len(buffer)} bytes")
                    
                    # 웨이크워드 감지 시 "네? 무엇을 도와드릴까요?" TTS 출력
                    print(f"[{get_kr_time()}][TTS] 웨이크워드 확인 응답 생성 중...")
                    
                    wake_response = "네? 무엇을 도와드릴까요?"
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
                    
                    try:
                        wake_tts_response = tts_client.synthesize_speech(
                            input=synthesis_input,
                            voice=voice,
                            audio_config=audio_config
                        )
                        
                        # 오디오 데이터를 float32로 변환
                        wake_audio_data = np.frombuffer(wake_tts_response.audio_content, dtype=np.int16)
                        wake_audio_float32 = wake_audio_data.astype(np.float32) / 32768.0
                        
                        # TCP를 통해 스피커 노드로 전송
                        print(f"[{get_kr_time()}][AUDIO] 웨이크워드 응답 전송 중...")
                        
                        if comm_manager.send_audio_data(wake_audio_float32):
                            print(f"[{get_kr_time()}][AUDIO] 웨이크워드 응답 전송 완료")
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
                        except Exception as e:
                            transcript = None
                            print(f"[{get_kr_time()}][STT] ❌ STT 오류: {e}")

                    os.remove(tmp_wav)

                    # [3] OpenAI로 의도 분석
                    if transcript:
                        system_prompt = (
                            "당신은 로봇의 음성 명령을 분석하는 AI입니다.\n"
                            "사용자의 발화를 듣고, 아래 4가지 의도 중 하나로 분류하세요.\n\n"
                            "- pause_navigation: '잠깐 멈춰', '멈춰봐' 등 일시정지 명령\n"
                            "- resume_follow: '다시 따라와', '다시 시작해' 등 팔로윙 재개 명령\n"
                            "- end_follow: '그만 따라와', '팔로윙 종료' 등 팔로윙 종료 명령\n"
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
                        import re
                        match = re.search(r'{"intent":\s*"(\w+)"}', ai_text)
                        if match:
                            intent = match.group(1)
                            print(f"[{get_kr_time()}][INTENT] 감지된 의도: {intent}")
                        else:
                            intent = "ignore"
                            print(f"[{get_kr_time()}][INTENT] ⚠️ 의도 분석 실패, 기본값 'ignore' 사용")

                        intent_responses = {
                            "pause_navigation": "네, 일시정지 하겠습니다.",
                            "resume_follow": "네, 팔로윙을 다시 시작하겠습니다.",
                            "end_follow": "네, 팔로윙을 종료하겠습니다.",
                            "ignore": "등록되지 않은 명령입니다. 무시하겠습니다."
                        }
                        response = intent_responses.get(intent, "등록되지 않은 명령입니다.")
                        print(f"[{get_kr_time()}][RESPONSE] {response}")
                        
                        # TTS로 응답 생성
                        print(f"[{get_kr_time()}][TTS] 음성 응답 생성 중...")
                        synthesis_input = texttospeech.SynthesisInput(text=response)
                        
                        voice = texttospeech.VoiceSelectionParams(
                            language_code="ko-KR",
                            name="ko-KR-Standard-A",
                            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
                        )
                        
                        audio_config = texttospeech.AudioConfig(
                            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
                            sample_rate_hertz=TTS_RATE,
                        )
                        
                        try:
                            tts_response = tts_client.synthesize_speech(
                                input=synthesis_input,
                                voice=voice,
                                audio_config=audio_config
                            )
                            
                            # 오디오 데이터를 float32로 변환
                            audio_data = np.frombuffer(tts_response.audio_content, dtype=np.int16)
                            audio_float32 = audio_data.astype(np.float32) / 32768.0
                            
                            # TCP를 통해 스피커 노드로 전송
                            print(f"[{get_kr_time()}][AUDIO] 오디오 데이터 전송 중...")
                            
                            if comm_manager.send_audio_data(audio_float32):
                                print(f"[{get_kr_time()}][AUDIO] 전송 완료")
                            else:
                                print(f"[{get_kr_time()}][AUDIO] ❌ 전송 실패")
                            
                        except Exception as e:
                            print(f"[{get_kr_time()}][ERROR] TTS/전송 오류: {str(e)}")
                            
                    print(f"[{get_kr_time()}][SYSTEM] '리보야' 이후 명령 처리 완료, 다시 웨이크워드 대기 중...")

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("[talker_manager] 종료 요청됨.")
    finally:
        print(f"[{get_kr_time()}][CLEANUP] 프로그램 종료 중...")
        comm_manager.cleanup()
        udp_thread.join()
        tcp_thread.join()
        if 'porcupine' in locals():
            porcupine.delete()
        print(f"[{get_kr_time()}][SYSTEM] 프로그램이 안전하게 종료되었습니다.")

if __name__ == '__main__':
    main()
