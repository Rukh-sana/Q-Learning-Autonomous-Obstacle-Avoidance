import serial
import time
import re
import signal
import sys


PORT = "/dev/ttyUSB0"
BAUD = 115200
DELAY = 0.3  # confirmed working delay from test_combined.py

# =============================
# SAFETY THRESHOLDS
# =============================
FRONT_EXTREME = 60
FRONT_DANGER = 100
FRONT_NEAR = 170
FRONT_MEDIUM = 300

BACK_BLOCKED = 80

# =============================
# SPEEDS
# =============================
PWM_FORWARD = 680
PWM_SLOW = 560
PWM_TURN = 700
PWM_REVERSE = 420

SER = None
last_turn = "right"
turn_lock = None
turn_lock_count = 0


# =============================
# SERIAL + MOVEMENT
# exact logic from your working test_combined.py
# =============================
def send(ser, cmd):
    ser.write(cmd.encode("ascii"))
    ser.flush()
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
    send(ser, f"PWMLF{int(speed * 0.7):04d}")


def soft_right(ser, speed=700):
    send(ser, "PWMRR0000")
    send(ser, "PWMLR0000")
    send(ser, f"PWMRF{int(speed * 0.7):04d}")
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


def buzzer_off(ser):
    send(ser, "BUZZOF")


def front_led_off(ser):
    send(ser, "FLEDOF")


def back_led_off(ser):
    send(ser, "BLEDOF")


def all_alerts_off(ser):
    try:
        buzzer_off(ser)
        front_led_off(ser)
        back_led_off(ser)
    except Exception:
        pass


def emergency_stop(signum=None, frame=None):
    global SER

    print("\n[EMERGENCY STOP] Stopping now...")

    if SER is not None:
        try:
            stop(SER)
            all_alerts_off(SER)
            SER.close()
            print("[OK] Disconnected")
        except Exception as e:
            print(f"[STOP ERROR] {e}")

    sys.exit(0)


signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


# =============================
# SENSOR
# =============================
def normalize_value(value):
    if value is None:
        return None

    try:
        value = int(value)
    except Exception:
        return None

    # examples: 5278 -> 78, 5343 -> 143
    if 5000 <= value <= 5600:
        value = value - 5200

    if 0 <= value <= 700:
        return value

    return None


def clean_back(back):
    back = normalize_value(back)
    if back is None:
        return 999
    return back


def parse_sensor(raw):
    if raw is None:
        return None, None

    matches = re.findall(r"#\s*(\d+)\s*%\s*(\d+)", raw)

    if not matches:
        return None, None

    valid = []

    for f_raw, b_raw in matches:
        f_raw = int(f_raw)
        b_raw = int(b_raw)

        front = normalize_value(f_raw)
        back = clean_back(b_raw)

        print(f"[PARSE] Raw F={f_raw} -> {front} | Raw B={b_raw} -> {back}")

        if front is not None:
            valid.append((front, back))

    if not valid:
        return None, None

    return valid[-1]


def read_one_sensor(ser):
    try:
        ser.reset_input_buffer()
        time.sleep(0.02)

        ser.write(b"FUSS")
        ser.flush()
        print("[SENT] FUSS")

        time.sleep(0.22)

        raw = ser.read(350).decode("ascii", errors="ignore").strip()
        print(f"[RECV] {repr(raw)}")

        return parse_sensor(raw)

    except Exception as e:
        print(f"[SENSOR ERROR] {e}")
        return None, None


def read_sensors(ser):
    readings = []

    for _ in range(3):
        front, back = read_one_sensor(ser)
        if front is not None:
            readings.append((front, back))
        time.sleep(0.04)

    if not readings:
        return None, None

    readings.sort(key=lambda x: x[0])
    return readings[len(readings) // 2]


# =============================
# DEMO MOVEMENT LOGIC
# =============================
def opposite_turn():
    global last_turn

    if last_turn == "right":
        last_turn = "left"
        return "left"

    last_turn = "right"
    return "right"


def do_turn(ser, direction):
    if direction == "left":
        turn_left(ser, PWM_TURN)
    else:
        turn_right(ser, PWM_TURN)


def controlled_forward(ser):
    forward(ser, PWM_FORWARD)
    time.sleep(0.05)
    stop(ser)


def controlled_slow_forward(ser):
    forward(ser, PWM_SLOW)
    time.sleep(0.04)
    stop(ser)


def controlled_turn(ser, direction):
    do_turn(ser, direction)
    time.sleep(0.10)
    stop(ser)


def controlled_escape(ser, direction):
    # Only for extreme obstacle distance
    reverse(ser, PWM_REVERSE)
    time.sleep(0.04)
    stop(ser)

    do_turn(ser, direction)
    time.sleep(0.12)
    stop(ser)


def choose_turn_after_check(ser, front):
    """
    Prevents same-place circular movement.
    It turns once, checks sensor, and changes direction if the turn did not improve distance.
    """
    global turn_lock, turn_lock_count

    if turn_lock is None or turn_lock_count <= 0:
        turn_lock = opposite_turn()
        turn_lock_count = 2

    direction = turn_lock
    before = front

    print(f"[AVOID] Controlled turn {direction}")
    controlled_turn(ser, direction)

    new_front, new_back = read_sensors(ser)

    # If the turn made it worse, switch direction next time
    if new_front is not None and before is not None and new_front < before + 8:
        print("[AVOID] Turn did not improve enough. Switching direction.")
        turn_lock = "left" if direction == "right" else "right"
        turn_lock_count = 2
    else:
        turn_lock_count -= 1

    return new_front, new_back


def main():
    global SER, turn_lock, turn_lock_count

    SER = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    SER.reset_input_buffer()
    SER.reset_output_buffer()

    print("[OK] Connected")
    print("======================================")
    print("FINAL DEMO CONTROLLER")
    print("Movement copied from confirmed working test_combined.py")
    print("Fix: prevents circular spinning in same place")
    print("Stop: CTRL + C")
    print("======================================")

    step = 0

    try:
        stop(SER)
        all_alerts_off(SER)
        time.sleep(0.5)

        while True:
            front, back = read_sensors(SER)

            if front is None:
                mode = "SENSOR LOST - STOP"
                stop(SER)
                print(f"Step={step} | F=None B=None | Mode={mode}")
                step += 1
                continue

            # reset turn lock when path becomes clear
            if front > FRONT_MEDIUM:
                turn_lock = None
                turn_lock_count = 0

            if front <= FRONT_EXTREME:
                mode = "EXTREME ESCAPE"

                if back is not None and back <= BACK_BLOCKED:
                    # back blocked, do not reverse
                    direction = opposite_turn()
                    controlled_turn(SER, direction)
                else:
                    direction = opposite_turn()
                    controlled_escape(SER, direction)

            elif front <= FRONT_DANGER:
                mode = "DANGER TURN"
                front, back = choose_turn_after_check(SER, front)

            elif front <= FRONT_NEAR:
                mode = "NEAR TURN"
                front, back = choose_turn_after_check(SER, front)

            elif front <= FRONT_MEDIUM:
                mode = "CAUTION SLOW FORWARD"
                controlled_slow_forward(SER)

            else:
                mode = "CLEAR FORWARD"
                controlled_forward(SER)

            print(f"Step={step} | F={front} B={back} | Mode={mode}")
            step += 1

    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")

    finally:
        stop(SER)
        all_alerts_off(SER)
        SER.close()
        print("[OK] Disconnected")


if __name__ == "__main__":
    main()