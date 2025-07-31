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

if not OPENAI_API_KEY or not PICOVOICE_ACCESS_KEY:
    raise ValueError("OPENAI_API_KEY 또는 PICOVOICE_ACCESS_KEY가 .env에 없습니다!")

DATA_DIR = os.path.join(PROJECT_ROOT, 'data')
PORCUPINE_MODEL_PATH = os.path.join(DATA_DIR, 'porcupine_params_ko.pv')
PORCUPINE_KEYWORD_PATH = os.path.join(DATA_DIR, 'riboya_ko_linux_v3_0_0.ppn')

for path in [PORCUPINE_MODEL_PATH, PORCUPINE_KEYWORD_PATH]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"필요한 모델 파일이 존재하지 않습니다: {path}")

# ================== 네트워크/오디오 기본 설정 ==================
HAEDWARE_HANDLER_IP = "127.0.0.1"       # 추후 HAEDWARE_HANDLER IP 로 변경해야 함
MIC_STREAM_PORT = 7000
NATIVE_RATE = 48000                     # mic_streamer와 동일 int16
TARGET_RATE = 16000                     # 웨이크워드 처리용
CHANNELS = 1
CHUNK = 2048                            # mic_streamer와 동일

# ================== UDP 수신 스레드 ==================
def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시

def udp_receiver(buffer_queue, stop_event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HAEDWARE_HANDLER_IP, MIC_STREAM_PORT))
    print(f"[{get_kr_time()}][talker_manager] Listening UDP {HAEDWARE_HANDLER_IP}:{MIC_STREAM_PORT} ...")
    received_count = 0
    start_time = time.time()
    
    while not stop_event.is_set():
        data, _ = sock.recvfrom(CHUNK * 2)
        buffer_queue.put(data)
        received_count += 1
        
        # 매 1000개 패킷마다 통계 출력
        if received_count % 1000 == 0:
            elapsed = time.time() - start_time
            rate = received_count / elapsed
            print(f"[{get_kr_time()}][UDP] 수신 통계: {received_count}개 패킷, {rate:.2f} packets/sec")

def main():
    # ========== 1. UDP 오디오 데이터 수신 ==========
    print(f"[{get_kr_time()}][INIT] UDP 수신 스레드 초기화 중...")
    buffer_queue = queue.Queue()
    stop_event = threading.Event()
    udp_thread = threading.Thread(target=udp_receiver, args=(buffer_queue, stop_event))
    udp_thread.start()

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
            while not buffer_queue.empty():
                buffer += buffer_queue.get()

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
                    # 여기서 마이크 스트림 닫거나 모드 전환 필요 없음 (이미 UDP 입력중)
                    # 이후: 명령어 인식 단계
                    # ---- 명령어 인식(구글 STT API 활용) ----
                    print(f"[{get_kr_time()}][STT] 다음 명령을 말씀하세요... (최대 5초)")

                    # [1] 버퍼에서 약 10초 분량(48000Hz * 2byte * 10초) 데이터 쌓기
                    RECORD_TIME = 10.0  # 녹음 시간을 10초로 설정
                    collected = b''
                    start = time.time()
                    print(f"[{get_kr_time()}][RECORD] 음성 수집 시작... (제한 시간: {RECORD_TIME}초)")
                    
                    while time.time() - start < RECORD_TIME:
                        current_time = time.time() - start
                        remaining_time = RECORD_TIME - current_time
                        
                        while not buffer_queue.empty():
                            data = buffer_queue.get()
                            collected += data
                            if len(collected) % (CHUNK * 10) == 0:  # 매 10 청크마다 로그
                                print(f"[{get_kr_time()}][RECORD] 수집된 데이터: {len(collected)} bytes (남은 시간: {remaining_time:.1f}초)")
                        time.sleep(0.05)
                    
                    duration = time.time() - start
                    print(f"[{get_kr_time()}][RECORD] 음성 수집 완료. 총 {len(collected)} bytes, 소요 시간: {duration:.1f}초")

                    # [2] WAV파일로 저장 → SpeechRecognition에서 로드
                    import wave
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
                    print(f"[{get_kr_time()}][SYSTEM] '리보야' 이후 명령 처리 완료, 다시 웨이크워드 대기 중...")

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("[talker_manager] 종료 요청됨.")
    finally:
        stop_event.set()
        udp_thread.join()
        if 'porcupine' in locals():
            porcupine.delete()

if __name__ == '__main__':
    main()
