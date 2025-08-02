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
