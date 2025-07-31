import pyaudio
import socket
from datetime import datetime
import pytz
import time

# ======================== 상수 정의 =========================
AI_SERVICE = "127.0.0.1"                # 추후 AI 서비스 서버 IP
MIC_STREAM_PORT = 7000           

MIC_INDEX = 10                          # 사용할 마이크 인덱스 (시스템에 따라 다름)
NATIVE_RATE = 48000                    
CHANNELS = 1                            # 모노 채널
CHUNK = 2048                            
# ===========================================================

def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시

def main():
    print(f"[{get_kr_time()}][INIT] mic_streamer 초기화 중...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pa = pyaudio.PyAudio()
    # 사용 가능한 오디오 장치 목록 출력
    print(f"[{get_kr_time()}][AUDIO] 사용 가능한 오디오 입력 장치:")
    for i in range(pa.get_device_count()):
        dev_info = pa.get_device_info_by_index(i)
        if dev_info['maxInputChannels'] > 0:  # 입력 장치만 출력
            print(f"[{get_kr_time()}][AUDIO] [{i}] {dev_info['name']}")

    stream = pa.open(
        format=pyaudio.paInt16,
        channels=CHANNELS,
        rate=NATIVE_RATE,
        input=True,
        frames_per_buffer=CHUNK,
        input_device_index=MIC_INDEX
    )

    print(f"[{get_kr_time()}][INIT] 스트리밍 시작... (Mic Index: {MIC_INDEX}, {NATIVE_RATE}Hz, chunk={CHUNK})")
    
    # 성능 모니터링을 위한 변수들
    packets_sent = 0
    start_time = time.time()
    last_log_time = start_time
    
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
