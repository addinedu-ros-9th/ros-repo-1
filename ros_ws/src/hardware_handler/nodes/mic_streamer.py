import socket
import threading
import pyaudio
import time

# 명령 수신용 TCP 서버 설정
TCP_IP = '0.0.0.0'
TCP_PORT = 7003

# 오디오 송신용 UDP 설정 (Talker_Manager)
UDP_IP = '192.168.0.1'   # Talker_Manager IP로 맞춰 변경
UDP_PORT = 7010

CHUNK = 320               # 20ms @ 16kHz, 16bit mono
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

# 상태 플래그
streaming = False
streaming_lock = threading.Lock()

def tcp_command_listener():
    global streaming
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print(f"[mic_streamer] Control TCP listen {TCP_PORT} ...")
    while True:
        conn, addr = s.accept()
        print(f"[mic_streamer] Command from {addr}")
        data = b''
        while True:
            chunk = conn.recv(1024)
            if not chunk:
                break
            data += chunk
            if b'\n' in data:
                break
        conn.close()
        cmd = data.strip()
        print(f"[mic_streamer] Received command: {cmd}")
        if cmd == b'SIGMIC_STREAM_STRAT':
            with streaming_lock:
                streaming = True
            print("[mic_streamer] ▶ 마이크 스트림 활성화")
        elif cmd == b'SIGMIC_STREAM_STOP':
            with streaming_lock:
                streaming = False
            print("[mic_streamer] ■ 마이크 스트림 중단")

def audio_sender():
    global streaming
    pa = pyaudio.PyAudio()
    stream = pa.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[mic_streamer] Ready to stream audio to {UDP_IP}:{UDP_PORT}")

    while True:
        with streaming_lock:
            active = streaming
        if active:
            data = stream.read(CHUNK, exception_on_overflow=False)
            sock.sendto(data, (UDP_IP, UDP_PORT))
        else:
            time.sleep(0.05)  # CPU 사용 최적화

def main():
    # 제어 쓰레드 (TCP로 명령 listen)
    t1 = threading.Thread(target=tcp_command_listener, daemon=True)
    t1.start()

    # 오디오 송신 메인루프
    audio_sender()

if __name__ == '__main__':
    main()
