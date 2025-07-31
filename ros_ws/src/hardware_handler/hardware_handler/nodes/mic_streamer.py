import pyaudio
import socket
from datetime import datetime
import pytz
import time

# ======================== 상수 정의 =========================
AI_SERVICE = "127.0.0.1"                # 추후 AI 서비스 서버 IP
MIC_STREAM_PORT = 7000           

MIC_INDEX = None                        # 자동 선택
NATIVE_RATE = 48000                    
CHANNELS = 1                            # 모노 채널
CHUNK = 2048                            

# 선호하는 마이크 키워드 (우선순위 순)
PREFERRED_MICS = [
    "MATA STUDIO",    # MATA STUDIO C10
    "USB",            # USB 마이크
    "pulse",          # PulseAudio
    "default"         # 기본 장치
]
# ===========================================================

def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시

def find_best_mic(pa):
    """사용 가능한 마이크 중 가장 적합한 것을 찾음"""
    print(f"[{get_kr_time()}][AUDIO] 사용 가능한 오디오 입력 장치:")
    
    available_mics = []
    for i in range(pa.get_device_count()):
        dev_info = pa.get_device_info_by_index(i)
        if dev_info['maxInputChannels'] > 0:  # 입력 장치만 출력
            print(f"[{get_kr_time()}][AUDIO] [{i}] {dev_info['name']}")
            available_mics.append((i, dev_info))
    
    # 선호하는 마이크 찾기
    for keyword in PREFERRED_MICS:
        for idx, dev_info in available_mics:
            if keyword.lower() in dev_info['name'].lower():
                print(f"[{get_kr_time()}][AUDIO] 선택된 마이크: {dev_info['name']} (index: {idx})")
                return idx
    
    # 선호 마이크를 찾지 못한 경우 기본 입력 장치 사용
    default_input = pa.get_default_input_device_info()
    print(f"[{get_kr_time()}][AUDIO] 기본 마이크 사용: {default_input['name']} (index: {default_input['index']})")
    return default_input['index']

def main():
    print(f"[{get_kr_time()}][INIT] mic_streamer 초기화 중...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pa = pyaudio.PyAudio()
    
    # 최적의 마이크 찾기
    mic_index = find_best_mic(pa)
    
    try:
        # 선택된 마이크의 지원 포맷 확인
        device_info = pa.get_device_info_by_index(mic_index)
        supported_channels = min(device_info['maxInputChannels'], CHANNELS)
        
        stream = pa.open(
            format=pyaudio.paInt16,
            channels=supported_channels,
            rate=NATIVE_RATE,
            input=True,
            frames_per_buffer=CHUNK,
            input_device_index=mic_index
        )
        
        print(f"[{get_kr_time()}][INIT] 스트리밍 시작...")
        print(f"[{get_kr_time()}][CONFIG] 마이크: {device_info['name']}")
        print(f"[{get_kr_time()}][CONFIG] 설정: {NATIVE_RATE}Hz, {supported_channels}채널, chunk={CHUNK}")
        
        # 성능 모니터링을 위한 변수들
        packets_sent = 0
        start_time = time.time()
        last_log_time = start_time
        
    except OSError as e:
        print(f"[{get_kr_time()}][ERROR] 오디오 스트림 초기화 실패: {str(e)}")
        pa.terminate()
        sock.close()
        return
    
    try:
        while True:
            data = stream.read(CHUNK, exception_on_overflow=False)
            sock.sendto(data, (AI_SERVICE, MIC_STREAM_PORT))
            
            # 성능 모니터링
            packets_sent += 1
            current_time = time.time()
            
            # 매 5초마다 통계 출력
            if current_time - last_log_time >= 5.0:
                elapsed = current_time - start_time
                rate = packets_sent / elapsed
                data_rate = (rate * CHUNK * 2) / 1024  # KB/s
                print(f"[{get_kr_time()}][STATS] 전송량: {packets_sent}개 패킷, {rate:.1f} packets/sec, {data_rate:.1f} KB/s")
                last_log_time = current_time
                
    except KeyboardInterrupt:
        print(f"[{get_kr_time()}][SYSTEM] 사용자에 의해 중지됨.")
    finally:
        print(f"[{get_kr_time()}][CLEANUP] 리소스 정리 중...")
        stream.stop_stream()
        stream.close()
        pa.terminate()
        sock.close()
        print(f"[{get_kr_time()}][SYSTEM] 프로그램 종료.")

if __name__ == '__main__':
    main()
