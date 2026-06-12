import serial
import time
import re
import signal
import sys


PORT = "/dev/ttyUSB0"
BAUD = 115200

# Confirmed working delay from your test_combined.py
DELAY = 0.30

# Speeds
PWM_FORWARD = 720
PWM_SLOW = 620
PWM_SOFT = 650
PWM_TURN = 720
PWM_REVERSE = 450

# Sensor thresholds
FRONT_EXTREME = 60
FRONT_DANGER = 100
FRONT_NEAR = 180
FRONT_MEDIUM = 320
FRONT_CLEAR = 380

BACK_BLOCKED = 80

SER = None

last_front = None
last_back = None
last_sensor_time = 0.0

preferred_turn = "right"
failed_avoid_count = 0
unknown_count = 0


# =====================================================
# MOTOR COMMANDS — copied from your confirmed working test
# =====================================================
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
    send(ser, f"PWMLF{int(speed * 0.70):04d}")


def soft_right(ser, speed=700):
    send(ser, "PWMRR0000")
    send(ser, "PWMLR0000")
    send(ser, f"PWMRF{int(speed * 0.70):04d}")
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


def alerts_off(ser):
    try:
        buzzer_off(ser)
        front_led_off(ser)
        back_led_off(ser)
    except Exception:
        pass


def safe_stop(ser):
    stop(ser)


# =====================================================
# EMERGENCY STOP
# =====================================================
def emergency_stop(signum=None, frame=None):
    global SER
    print("\n[EMERGENCY STOP] Stopping now...")

    if SER is not None and SER.is_open:
        try:
            stop(SER)
            alerts_off(SER)
            SER.close()
            print("[OK] Disconnected")
        except Exception as e:
            print(f"[STOP ERROR] {e}")

    sys.exit(0)


signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


# =====================================================
# SENSOR
# =====================================================
def normalize_value(value):
    try:
        value = int(value)
    except Exception:
        return None

    # Example: 5337 -> 137
    if 5000 <= value <= 5600:
        value -= 5200

    if 0 <= value <= 700:
        return value

    return None


def clean_back(value):
    value = normalize_value(value)
    if value is None:
        return 999
    return value


def parse_sensor(raw):
    if not raw:
        return None, None

    pairs = re.findall(r"#\s*(\d+)\s*%\s*(\d+)", raw)
    if not pairs:
        return None, None

    valid = []

    for f_raw, b_raw in pairs:
        f_raw_i = int(f_raw)
        b_raw_i = int(b_raw)

        front = normalize_value(f_raw_i)
        back = clean_back(b_raw_i)

        print(f"[PARSE] Raw F={f_raw_i} -> {front} | Raw B={b_raw_i} -> {back}")

        if front is not None:
            valid.append((front, back))

    if not valid:
        return None, None

    # Use the latest valid pair from the received string
    return valid[-1]


def read_one_sensor(ser):
    try:
        ser.reset_input_buffer()
        time.sleep(0.05)

        ser.write(b"FUSS")
        ser.flush()
        print("[SENT] FUSS")

        time.sleep(0.25)

        raw = ser.read(500).decode("ascii", errors="ignore").strip()
        print(f"[RECV] {repr(raw)}")

        return parse_sensor(raw)

    except Exception as e:
        print(f"[SENSOR ERROR] {e}")
        return None, None


def read_sensors(ser):
    global last_front, last_back, last_sensor_time

    readings = []

    for _ in range(4):
        front, back = read_one_sensor(ser)
        if front is not None:
            readings.append((front, back))
        time.sleep(0.05)

    if readings:
        # Use median front value to reduce one-off noisy spikes
        readings.sort(key=lambda x: x[0])
        front, back = readings[len(readings) // 2]

        last_front = front
        last_back = back
        last_sensor_time = time.time()

        return front, back

    # If sensor briefly fails, reuse recent valid reading.
    if last_front is not None and time.time() - last_sensor_time <= 1.5:
        print(f"[WARN] Using last valid sensor F={last_front} B={last_back}")
        return last_front, last_back

    return None, None


# =====================================================
# CONTROL HELPERS
# =====================================================
def run_forward_step(ser):
    forward(ser, PWM_FORWARD)
    time.sleep(0.05)
    stop(ser)


def run_slow_step(ser):
    forward(ser, PWM_SLOW)
    time.sleep(0.04)
    stop(ser)


def run_soft(ser, direction):
    if direction == "left":
        soft_left(ser, PWM_SOFT)
    else:
        soft_right(ser, PWM_SOFT)

    time.sleep(0.06)
    stop(ser)


def run_turn(ser, direction):
    if direction == "left":
        turn_left(ser, PWM_TURN)
    else:
        turn_right(ser, PWM_TURN)

    time.sleep(0.07)
    stop(ser)


def run_escape(ser, direction):
    reverse(ser, PWM_REVERSE)
    time.sleep(0.04)
    stop(ser)

    if direction == "left":
        turn_left(ser, PWM_TURN)
    else:
        turn_right(ser, PWM_TURN)

    time.sleep(0.09)
    stop(ser)


def opposite(direction):
    return "left" if direction == "right" else "right"


def adaptive_avoid(ser, front, mode):
    """
    Avoid obstacle without spinning forever.
    Tries preferred direction. If front does not improve, switches direction.
    """
    global preferred_turn, failed_avoid_count

    before = front
    direction = preferred_turn

    print(f"[AVOID] {mode} using {direction}")

    if mode == "soft":
        run_soft(ser, direction)
    elif mode == "turn":
        run_turn(ser, direction)
    else:
        run_escape(ser, direction)

    after, back = read_sensors(ser)

    if after is None:
        failed_avoid_count += 1
        print("[AVOID] Sensor missing after avoid.")
        return after, back

    improvement = after - before
    print(f"[AVOID] before={before}, after={after}, improvement={improvement}")

    if improvement < 10:
        failed_avoid_count += 1
        preferred_turn = opposite(preferred_turn)
        print(f"[AVOID] Not enough improvement. Switching preferred turn to {preferred_turn}.")
    else:
        failed_avoid_count = 0

    # If soft avoiding fails twice, do a short pivot/escape recovery.
    if failed_avoid_count >= 2:
        print("[RECOVERY] Avoid failed twice. Doing controlled recovery.")
        failed_avoid_count = 0

        if back is not None and back > BACK_BLOCKED:
            run_escape(ser, preferred_turn)
        else:
            run_turn(ser, preferred_turn)

        after, back = read_sensors(ser)

    return after, back


# =====================================================
# INTELLIGENT POLICY
# =====================================================
def decide_and_act(ser, front, back):
    global unknown_count, preferred_turn

    if front is None:
        unknown_count += 1
        print(f"[MODE] SENSOR UNKNOWN count={unknown_count}")

        stop(ser)

        # If sensor is still missing, do not drive blindly.
        if unknown_count >= 3:
            print("[MODE] Sensor failed repeatedly. Holding stop.")
            time.sleep(0.3)

        return None, None, "sensor_stop"

    unknown_count = 0

    back_blocked = back is not None and back <= BACK_BLOCKED

    # Very close: reverse only if back is clear.
    if front <= FRONT_EXTREME:
        print("[MODE] EXTREME")
        if back_blocked:
            new_front, new_back = adaptive_avoid(ser, front, "turn")
        else:
            new_front, new_back = adaptive_avoid(ser, front, "escape")
        return new_front, new_back, "extreme_escape"

    # Danger: pivot turn, not forward.
    if front <= FRONT_DANGER:
        print("[MODE] DANGER")
        new_front, new_back = adaptive_avoid(ser, front, "turn")
        return new_front, new_back, "danger_turn"

    # Near: soft arc, not pivot.
    if front <= FRONT_NEAR:
        print("[MODE] NEAR")
        new_front, new_back = adaptive_avoid(ser, front, "soft")
        return new_front, new_back, "near_soft"

    # Medium: slow forward only, but very short.
    if front <= FRONT_MEDIUM:
        print("[MODE] MEDIUM")
        run_slow_step(ser)
        new_front, new_back = read_sensors(ser)

        if new_front is not None and new_front < front - 80:
            print("[SAFETY] Slow forward dropped distance too much. Switching avoid direction.")
            preferred_turn = opposite(preferred_turn)
            new_front, new_back = adaptive_avoid(ser, new_front, "soft")

        return new_front, new_back, "medium_slow"

    # Clear: forward.
    print("[MODE] CLEAR")
    run_forward_step(ser)
    new_front, new_back = read_sensors(ser)

    if new_front is not None and new_front < FRONT_NEAR:
        print("[SAFETY] Forward entered near zone. Avoiding immediately.")
        preferred_turn = opposite(preferred_turn)
        new_front, new_back = adaptive_avoid(ser, new_front, "soft")

    return new_front, new_back, "clear_forward"


def main():
    global SER

    SER = serial.Serial(PORT, BAUD, timeout=1)

    time.sleep(2)
    SER.reset_input_buffer()
    SER.reset_output_buffer()

    print("[OK] Connected")
    print("======================================")
    print("FINAL INTELLIGENT DEMO POLICY")
    print("No Q-training. Adaptive obstacle avoidance.")
    print("Near = soft arc, Danger = turn, Extreme = escape.")
    print("Stop: CTRL + C")
    print("======================================")

    step = 0

    try:
        stop(SER)
        alerts_off(SER)
        time.sleep(0.5)

        front, back = read_sensors(SER)

        while True:
            print("\n--------------------------------------")
            print(f"Step={step} | Current F={front} B={back}")

            front, back, mode = decide_and_act(SER, front, back)

            print(f"Step={step} | Mode={mode} | New F={front} B={back}")
            step += 1

    except KeyboardInterrupt:
        print("\n[STOP] Stopped by user.")

    finally:
        try:
            if SER and SER.is_open:
                stop(SER)
                alerts_off(SER)
                SER.close()
                print("[OK] Disconnected")
        except Exception as e:
            print(f"[FINAL STOP ERROR] {e}")


if __name__ == "__main__":
    main()