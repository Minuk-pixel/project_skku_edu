import serial
import time

# 🔌 아두이노 포트 번호 확인해서 수정하세요
PORT = 'COM3'
BAUDRATE = 9600

# 📦 시리얼 포트 열기
ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
time.sleep(2)  # 아두이노 리셋 대기

# 🎯 1. S15.00 명령 전송 (조향각 15도)
command = "S15.00\n"
ser.write(command.encode())
print(f"[TX] Sent: {command.strip()}")

# 📥 2. Serial 출력 수신
print("[INFO] Waiting for Arduino response...\n")

start_time = time.time()
while time.time() - start_time < 5:  # 5초간 읽기 시도
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print("[RX]", line)
        except Exception as e:
            print("[ERROR]", e)
    time.sleep(0.01)

ser.close()
