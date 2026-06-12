# src/test_combined.py
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200
DELAY = 0.3  # confirmed working delay

def send(ser, cmd):
    ser.write(cmd.encode("ascii"))
    print(f"[SENT] {cmd}")
    time.sleep(DELAY)

def stop(ser):
    send(ser, "PWMRF0000")
    send(ser, "PWMRR0000")
    send(ser, "PWMLF0000")
    send(ser, "PWMLR0000")

def forward(ser, speed=700):
    send(ser, "PWMRR0000")
    send(ser, "PWMLR0000")
    send(ser, f"PWMRF{speed:04d}")
    send(ser, f"PWMLF{speed:04d}")

def reverse(ser, speed=700):
    send(ser, "PWMRF0000")
    send(ser, "PWMLF0000")
    send(ser, f"PWMRR{speed:04d}")
    send(ser, f"PWMLR{speed:04d}")

def soft_left(ser, speed=700):
    send(ser, "PWMRR0000")
    send(ser, "PWMLR0000")
    send(ser, f"PWMRF{speed:04d}")
    send(ser, f"PWMLF{int(speed*0.7):04d}")

def soft_right(ser, speed=700):
    send(ser, "PWMRR0000")
    send(ser, "PWMLR0000")
    send(ser, f"PWMRF{int(speed*0.7):04d}")
    send(ser, f"PWMLF{speed:04d}")

def turn_left(ser, speed=700):
    send(ser, "PWMRR0000")
    send(ser, "PWMLF0000")
    send(ser, f"PWMRF{speed:04d}")
    send(ser, f"PWMLR{speed:04d}")

def turn_right(ser, speed=700):
    send(ser, "PWMRF0000")
    send(ser, "PWMLR0000")
    send(ser, f"PWMLF{speed:04d}")
    send(ser, f"PWMRR{speed:04d}")

# =============================
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
print("[OK] Connected")

try:
    print("\nLifting car wheels before test...")
    input("Press ENTER when ready...")

    stop(ser)
    time.sleep(1)

    print("\nTEST 1: FORWARD")
    forward(ser, 700)
    time.sleep(3)
    stop(ser)
    time.sleep(1)
    input("Both sides forward? ENTER...")

    print("\nTEST 2: REVERSE")
    reverse(ser, 700)
    time.sleep(3)
    stop(ser)
    time.sleep(1)
    input("Both sides reverse? ENTER...")

    print("\nTEST 3: SOFT LEFT")
    soft_left(ser, 700)
    time.sleep(3)
    stop(ser)
    time.sleep(1)
    input("Car curves left? ENTER...")

    print("\nTEST 4: SOFT RIGHT")
    soft_right(ser, 700)
    time.sleep(3)
    stop(ser)
    time.sleep(1)
    input("Car curves right? ENTER...")

    print("\nTEST 5: TURN LEFT")
    turn_left(ser, 700)
    time.sleep(2)
    stop(ser)
    time.sleep(1)
    input("Car turns left sharply? ENTER...")

    print("\nTEST 6: TURN RIGHT")
    turn_right(ser, 700)
    time.sleep(2)
    stop(ser)
    time.sleep(1)
    input("Car turns right sharply? ENTER...")

    print("\n[DONE] All tests complete!")

except KeyboardInterrupt:
    print("\nStopped!")

finally:
    stop(ser)
    ser.close()
    print("[OK] Disconnected")