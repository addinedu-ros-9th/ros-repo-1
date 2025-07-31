import pyaudio
import socket
from datetime import datetime
import pytz
import numpy as np
import threading
import queue
import time

# ======================== 상수 정의 =========================
AI_SERVICE_IP = "127.0.0.1"            # AI Service IP (TCP 서버)
SPEAKER_PORT = 7002                     # 스피커 출력용 포트 (TCP)

CHANNELS = 1                           # 모노 채널
RATE = 24000                          # 샘플링 레이트
CHUNK = 1024                          # 청크 크기
FORMAT = pyaudio.paFloat32            # 32비트 부동소수점
MAX_RETRY_COUNT = 5                   # 최대 재연결 시도 횟수
RETRY_INTERVAL = 2                    # 재연결 시도 간격(초)
# ===========================================================

def get_kr_time():
    """한국 시간 ISO 포맷으로 반환"""
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

class SpeakerNode:
    def __init__(self):
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
        
    def connect_to_server(self):
        """AI 서비스에 TCP 연결"""
        retry_count = 0
        while retry_count < MAX_RETRY_COUNT:
            try:
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.connect((AI_SERVICE_IP, SPEAKER_PORT))
                print(f"[{get_kr_time()}][TCP] AI 서비스 연결 성공")
                return True
            except ConnectionRefusedError:
                retry_count += 1
                print(f"[{get_kr_time()}][TCP] 연결 실패. {RETRY_INTERVAL}초 후 재시도... ({retry_count}/{MAX_RETRY_COUNT})")
                time.sleep(RETRY_INTERVAL)
                
        print(f"[{get_kr_time()}][ERROR] 최대 재시도 횟수 초과. 프로그램을 종료합니다.")
        return False

    def receive_audio(self):
        """TCP로 오디오 데이터 수신"""
        print(f"[{get_kr_time()}][INIT] 오디오 수신 대기 중...")
        while self.is_running:
            try:
                # 데이터 크기 수신 (4바이트)
                size_data = self.tcp_socket.recv(4)
                if not size_data:
                    print(f"[{get_kr_time()}][TCP] 서버와의 연결이 종료되었습니다.")
                    break
                    
                total_size = int.from_bytes(size_data, byteorder='big')
                received_size = 0
                
                # 메시지 유형 확인 (웨이크워드 응답인지 명령 응답인지)
                message_type = "웨이크워드 응답" if total_size < 50000 else "명령 응답"
                
                # 전체 데이터 수신
                while received_size < total_size and self.is_running:
                    chunk_size = min(CHUNK * 4, total_size - received_size)
                    data = self.tcp_socket.recv(chunk_size)
                    if not data:
                        break
                    self.audio_queue.put(data)
                    received_size += len(data)
                
                # 전체 데이터 수신 완료 후 디버깅 메시지
                print(f"[{get_kr_time()}][AUDIO] 📢 {message_type} 수신 완료: {received_size/1024:.1f}KB ({received_size}/{total_size} bytes)")
                    
            except Exception as e:
                print(f"[{get_kr_time()}][ERROR] 수신 오류: {str(e)}")
                break
                
        # 연결이 끊어진 경우 재연결 시도
        if self.is_running:
            print(f"[{get_kr_time()}][TCP] 서버와의 연결이 끊어졌습니다. 재연결을 시도합니다.")
            if self.connect_to_server():
                self.receive_audio()

    def play_audio(self):
        """수신된 오디오 재생"""
        print(f"[{get_kr_time()}][INIT] 오디오 재생 스레드 시작")
        audio_chunk_count = 0  # 재생된 청크 수 카운터
        total_bytes_played = 0  # 총 재생된 바이트 수
        
        while self.is_running:
            try:
                if not self.audio_queue.empty():
                    data = self.audio_queue.get()
                    audio_data = np.frombuffer(data, dtype=np.float32)
                    self.stream.write(audio_data.tobytes())
                    
                    # 디버깅 정보 업데이트
                    audio_chunk_count += 1
                    total_bytes_played += len(data)
                    
                    # 매 5개 청크마다 디버깅 정보 출력
                    if audio_chunk_count % 5 == 0:
                        print(f"[{get_kr_time()}][AUDIO] 🔊 재생 중: {audio_chunk_count}개 청크, "
                              f"총 {total_bytes_played/1024:.1f}KB 재생됨")
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

def main():
    speaker = SpeakerNode()
    speaker.run()

if __name__ == '__main__':
    main()
