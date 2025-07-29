import socket
import struct
import json
import tempfile
import pygame
import os

TCP_IP = '192.168.0.1'
TCP_PORT = 7002
BUFFER_SIZE = 4096

def play_audio(file_path):
    pygame.mixer.init()
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)
    pygame.mixer.quit()

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print(f"[speaker_node] Listening on port {TCP_PORT}...")

    while True:
        conn, addr = s.accept()
        print(f"[speaker_node] Connection from {addr}")

        # 1. 4바이트 길이 먼저 받기
        length_bytes = b''
        while len(length_bytes) < 4:
            chunk = conn.recv(4 - len(length_bytes))
            if not chunk:
                print("[speaker_node] Connection closed before length received")
                conn.close()
                break
            length_bytes += chunk
        if len(length_bytes) < 4:
            continue

        msg_len = struct.unpack('!I', length_bytes)[0]

        # 2. 전체 데이터 수신 (msg_len 만큼)
        data = b''
        while len(data) < msg_len:
            chunk = conn.recv(min(BUFFER_SIZE, msg_len - len(data)))
            if not chunk:
                break
            data += chunk

        conn.close()

        # 3. JSON + | + binary 파싱
        try:
            json_part, audio_part = data.split(b'|', 1)
            if audio_part.endswith(b'\n'):
                audio_part = audio_part[:-1]
            header = json.loads(json_part.decode('utf-8'))
        except Exception as e:
            print(f"[speaker_node] Parse error: {e}")
            continue

        print(f"[speaker_node] header: {header}")

        # mp3 확정 (format 고정)
        with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as tmpf:
            tmpf.write(audio_part)
            temp_path = tmpf.name

        try:
            play_audio(temp_path)
        finally:
            os.remove(temp_path)
            print(f"[speaker_node] Finished, file deleted.")

if __name__ == '__main__':
    main()
