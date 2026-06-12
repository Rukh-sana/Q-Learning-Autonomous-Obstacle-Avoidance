import os
import json
import csv
import time
import random
import re
import signal
import sys
import serial


PORT = "/dev/ttyUSB0"
BAUD = 115200

# Confirmed working delay from your test_combined.py
DELAY = 0.30

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(BASE_DIR, "models")
DATA_DIR = os.path.join(BASE_DIR, "data")

Q_TABLE_PATH = os.path.join(MODEL_DIR, "q_table_rl_final.json")
LOG_PATH = os.path.join(DATA_DIR, "q_table_rl_training_log.csv")


# =====================================================
# SAFETY THRESHOLDS
# =====================================================
FRONT_EXTREME = 60      # reverse + turn only
FRONT_DANGER = 100      # pivot turn only
FRONT_NEAR = 180        # soft arc turn only
FRONT_MEDIUM = 320      # slow forward only
BACK_BLOCKED = 80


# =====================================================
# SPEED SETTINGS
# =====================================================
PWM_FORWARD = 760
PWM_SLOW = 650
PWM_TURN = 720
PWM_REVERSE = 450


# =====================================================
# Q-LEARNING SETTINGS
# =====================================================
ALPHA = 0.18
GAMMA = 0.88

EPSILON_START = 0.10
EPSILON_MIN = 0.03
EPSILON_DECAY = 0.985

MAX_EPISODES = 120
MAX_STEPS_PER_EPISODE = 45
SAVE_EVERY = 3


ACTIONS = [
    "forward",
    "slow_forward",
    "soft_left",
    "soft_right",
    "turn_left",
    "turn_right",
    "escape_left",
    "escape_right",
    "stop",
]


SER_REF = None
failed_soft_count = 0
failed_turn_count = 0


# =====================================================
# MOTOR COMMANDS — EXACT BASE FROM WORKING test_combined.py
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


def safe_alerts_off(ser):
    try:
        buzzer_off(ser)
        front_led_off(ser)
        back_led_off(ser)
    except Exception:
        pass


# =====================================================
# EMERGENCY STOP
# =====================================================
def emergency_shutdown(signum=None, frame=None):
    global SER_REF

    print("\n[EMERGENCY STOP] Stopping now...")

    if SER_REF is not None and SER_REF.is_open:
        try:
            stop(SER_REF)
            safe_alerts_off(SER_REF)
        except Exception as e:
            print(f"[STOP ERROR] {e}")

    raise KeyboardInterrupt


signal.signal(signal.SIGINT, emergency_shutdown)
signal.signal(signal.SIGTERM, emergency_shutdown)


# =====================================================
# SENSOR FUNCTIONS
# =====================================================
def normalize_value(value):
    if value is None:
        return None

    try:
        value = int(value)
    except Exception:
        return None

    # Examples from your logs: 5278 -> 78, 5343 -> 143
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

    for front_raw, back_raw in matches:
        f_raw = int(front_raw)
        b_raw = int(back_raw)

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

        time.sleep(0.20)

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
        time.sleep(0.03)

    if not readings:
        return None, None

    readings.sort(key=lambda x: x[0])
    return readings[len(readings) // 2]


# =====================================================
# FILE HELPERS
# =====================================================
def ensure_dirs():
    os.makedirs(MODEL_DIR, exist_ok=True)
    os.makedirs(DATA_DIR, exist_ok=True)


def load_q_table():
    if not os.path.exists(Q_TABLE_PATH):
        return {}

    if os.path.getsize(Q_TABLE_PATH) == 0:
        return {}

    try:
        with open(Q_TABLE_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError:
        print("[WARN] Q-table damaged. Starting fresh.")
        return {}


def save_q_table(q_table):
    with open(Q_TABLE_PATH, "w", encoding="utf-8") as f:
        json.dump(q_table, f, indent=2)


def init_log():
    if not os.path.exists(LOG_PATH) or os.path.getsize(LOG_PATH) == 0:
        with open(LOG_PATH, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "episode",
                "step",
                "front",
                "back",
                "state",
                "allowed_actions",
                "action",
                "reward",
                "next_front",
                "next_back",
                "next_state",
                "q_value",
                "epsilon",
            ])


def write_log(row):
    with open(LOG_PATH, "a", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow(row)


# =====================================================
# RL STATE
# =====================================================
def front_zone(front):
    if front is None:
        return "unknown"
    if front <= FRONT_EXTREME:
        return "extreme"
    if front <= FRONT_DANGER:
        return "danger"
    if front <= FRONT_NEAR:
        return "near"
    if front <= FRONT_MEDIUM:
        return "medium"
    return "clear"


def back_zone(back):
    if back is None:
        return "unknown"
    if back <= BACK_BLOCKED:
        return "blocked"
    return "clear"


def get_state(front, back):
    return f"F:{front_zone(front)}|B:{back_zone(back)}"


def ensure_state(q_table, state):
    if state not in q_table:
        q_table[state] = {}

    for action in ACTIONS:
        if action not in q_table[state]:
            q_table[state][action] = 0.0

    if state.startswith("F:clear"):
        q_table[state]["forward"] = max(q_table[state]["forward"], 0.60)
        q_table[state]["slow_forward"] = max(q_table[state]["slow_forward"], 0.30)

    elif state.startswith("F:medium"):
        q_table[state]["slow_forward"] = max(q_table[state]["slow_forward"], 0.45)

    elif state.startswith("F:near"):
        q_table[state]["soft_left"] = max(q_table[state]["soft_left"], 0.40)
        q_table[state]["soft_right"] = max(q_table[state]["soft_right"], 0.40)

    elif state.startswith("F:danger"):
        q_table[state]["turn_left"] = max(q_table[state]["turn_left"], 0.40)
        q_table[state]["turn_right"] = max(q_table[state]["turn_right"], 0.40)

    elif state.startswith("F:extreme"):
        q_table[state]["escape_left"] = max(q_table[state]["escape_left"], 0.45)
        q_table[state]["escape_right"] = max(q_table[state]["escape_right"], 0.45)


def allowed_actions(front, back):
    if front is None:
        return ["stop"]

    back_blocked = back is not None and back <= BACK_BLOCKED

    # Extreme: reverse escape only here
    if front <= FRONT_EXTREME:
        if back_blocked:
            return ["turn_left", "turn_right", "stop"]
        return ["escape_left", "escape_right"]

    # Danger: pivot only
    if front <= FRONT_DANGER:
        return ["turn_left", "turn_right"]

    # Near: soft arc movement, not pivot
    if front <= FRONT_NEAR:
        return ["soft_left", "soft_right"]

    # Medium: slow forward
    if front <= FRONT_MEDIUM:
        return ["slow_forward"]

    # Clear
    return ["forward", "slow_forward"]


def choose_action(q_table, state, actions, epsilon):
    ensure_state(q_table, state)

    if not actions:
        return "stop"

    if random.random() < epsilon:
        return random.choice(actions)

    return max(actions, key=lambda a: q_table[state].get(a, 0.0))


def update_q(q_table, state, action, reward, next_state, next_actions):
    ensure_state(q_table, state)
    ensure_state(q_table, next_state)

    old_q = q_table[state].get(action, 0.0)

    if next_actions:
        best_next = max(q_table[next_state].get(a, 0.0) for a in next_actions)
    else:
        best_next = 0.0

    new_q = old_q + ALPHA * (reward + GAMMA * best_next - old_q)
    q_table[state][action] = new_q

    return new_q


def reward_function(front_before, front_after, action):
    if front_before is None:
        return 3.0 if action == "stop" else -10.0

    if front_after is None:
        return -8.0

    before = front_zone(front_before)
    improvement = max(-100, min(100, front_after - front_before)) / 10.0

    if front_after <= FRONT_EXTREME and before != "extreme":
        return -25.0

    if front_after <= FRONT_DANGER and before in ["medium", "clear", "near"]:
        return -20.0

    if abs(front_after - front_before) <= 3:
        if action in ["forward", "slow_forward"]:
            return -2.0
        if action in ["soft_left", "soft_right", "turn_left", "turn_right", "escape_left", "escape_right"]:
            return -4.0

    if before == "clear":
        if action == "forward":
            if front_after <= FRONT_MEDIUM:
                return -12.0
            return 10.0 + improvement

        if action == "slow_forward":
            if front_after <= FRONT_NEAR:
                return -12.0
            return 6.0 + improvement

        return -4.0

    if before == "medium":
        if action == "slow_forward":
            if front_after <= FRONT_NEAR:
                return -12.0
            return 5.0 + improvement
        return -6.0

    if before == "near":
        if action in ["soft_left", "soft_right"]:
            if front_after > FRONT_MEDIUM:
                return 16.0
            if front_after > front_before + 25:
                return 10.0 + improvement
            if front_after > front_before + 8:
                return 5.0 + improvement
            return -4.0

        if action in ["turn_left", "turn_right"]:
            return -8.0

        return -20.0

    if before == "danger":
        if action in ["turn_left", "turn_right"]:
            if front_after > FRONT_MEDIUM:
                return 16.0
            if front_after > front_before + 25:
                return 10.0 + improvement
            if front_after > front_before + 8:
                return 5.0 + improvement
            return -4.0

        return -20.0

    if before == "extreme":
        if action in ["escape_left", "escape_right"]:
            if front_after > FRONT_MEDIUM:
                return 18.0
            if front_after > front_before + 25:
                return 12.0 + improvement
            return -5.0

        return -25.0

    return -1.0


# =====================================================
# ACTION EXECUTION + RECOVERY
# =====================================================
def opposite_action(action):
    if action == "soft_left":
        return "soft_right"
    if action == "soft_right":
        return "soft_left"
    if action == "turn_left":
        return "turn_right"
    if action == "turn_right":
        return "turn_left"
    if action == "escape_left":
        return "escape_right"
    if action == "escape_right":
        return "escape_left"
    return "soft_right"


def perform_action(ser, action):
    if action == "forward":
        forward(ser, PWM_FORWARD)
        time.sleep(0.08)

    elif action == "slow_forward":
        forward(ser, PWM_SLOW)
        time.sleep(0.06)

    elif action == "soft_left":
        soft_left(ser, PWM_SLOW)
        time.sleep(0.08)

    elif action == "soft_right":
        soft_right(ser, PWM_SLOW)
        time.sleep(0.08)

    elif action == "turn_left":
        turn_left(ser, PWM_TURN)
        time.sleep(0.08)

    elif action == "turn_right":
        turn_right(ser, PWM_TURN)
        time.sleep(0.08)

    elif action == "escape_left":
        reverse(ser, PWM_REVERSE)
        time.sleep(0.04)
        stop(ser)
        time.sleep(0.04)
        turn_left(ser, PWM_TURN)
        time.sleep(0.10)

    elif action == "escape_right":
        reverse(ser, PWM_REVERSE)
        time.sleep(0.04)
        stop(ser)
        time.sleep(0.04)
        turn_right(ser, PWM_TURN)
        time.sleep(0.10)

    else:
        stop(ser)
        time.sleep(0.05)

    stop(ser)


def apply_recovery_if_needed(ser, front, back, next_front, action):
    global failed_soft_count, failed_turn_count

    if action in ["soft_left", "soft_right"]:
        if front is None or next_front is None or next_front <= front + 5:
            failed_soft_count += 1
        else:
            failed_soft_count = 0

        if failed_soft_count >= 2:
            print("[RECOVERY] Soft arc not improving. Switching soft direction.")
            failed_soft_count = 0

            recovery_action = opposite_action(action)
            if recovery_action == "soft_left":
                soft_left(ser, PWM_SLOW)
            else:
                soft_right(ser, PWM_SLOW)

            time.sleep(0.10)
            stop(ser)
            return read_sensors(ser)

        return next_front, None

    if action in ["turn_left", "turn_right"]:
        if front is None or next_front is None or next_front <= front + 5:
            failed_turn_count += 1
        else:
            failed_turn_count = 0

        if failed_turn_count >= 2:
            print("[RECOVERY] Pivot not improving. Short reverse + opposite turn.")
            failed_turn_count = 0

            if back is not None and back > BACK_BLOCKED:
                reverse(ser, PWM_REVERSE)
                time.sleep(0.04)
                stop(ser)

            recovery_action = opposite_action(action)
            if recovery_action == "turn_left":
                turn_left(ser, PWM_TURN)
            else:
                turn_right(ser, PWM_TURN)

            time.sleep(0.10)
            stop(ser)
            return read_sensors(ser)

        return next_front, None

    failed_soft_count = 0
    failed_turn_count = 0
    return next_front, None


# =====================================================
# MAIN TRAINING
# =====================================================
def main():
    global SER_REF

    ensure_dirs()
    init_log()

    q_table = load_q_table()
    epsilon = EPSILON_START

    ser = serial.Serial(PORT, BAUD, timeout=1)
    SER_REF = ser

    time.sleep(2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("[OK] Connected")
    print("======================================")
    print("Q-TABLE RL TRAINING — FINAL NO-CIRCLE VERSION")
    print("Near = soft arc movement, not pivot.")
    print("Danger = pivot. Extreme = escape.")
    print("Movement = confirmed test_combined.py logic.")
    print(f"Q-table : {Q_TABLE_PATH}")
    print(f"Log     : {LOG_PATH}")
    print("Stop    : CTRL + C")
    print("======================================")

    try:
        stop(ser)
        safe_alerts_off(ser)
        time.sleep(0.5)

        for episode in range(1, MAX_EPISODES + 1):
            print(f"\n--- Episode {episode}/{MAX_EPISODES} | epsilon={epsilon:.3f} ---")

            front, back = read_sensors(ser)
            state = get_state(front, back)
            total_reward = 0.0

            for step in range(MAX_STEPS_PER_EPISODE):
                actions = allowed_actions(front, back)
                action = choose_action(q_table, state, actions, epsilon)

                perform_action(ser, action)

                next_front, next_back = read_sensors(ser)

                recovered_front, recovered_back = apply_recovery_if_needed(
                    ser, front, back, next_front, action
                )

                if recovered_front is not None:
                    next_front = recovered_front
                    next_back = recovered_back

                next_state = get_state(next_front, next_back)
                next_actions = allowed_actions(next_front, next_back)

                reward = reward_function(front, next_front, action)
                q_value = update_q(q_table, state, action, reward, next_state, next_actions)

                write_log([
                    episode,
                    step,
                    front,
                    back,
                    state,
                    "|".join(actions),
                    action,
                    round(reward, 2),
                    next_front,
                    next_back,
                    next_state,
                    round(q_value, 4),
                    round(epsilon, 4),
                ])

                print(
                    f"Ep={episode} Step={step} | "
                    f"F={front} B={back} | State={state} | "
                    f"Allowed={actions} | Action={action} | "
                    f"Reward={reward:+.2f} | NextF={next_front} | Q={q_value:.3f}"
                )

                total_reward += reward
                front, back, state = next_front, next_back, next_state

            print(f"[EPISODE DONE] total_reward={total_reward:.2f}")

            epsilon = max(EPSILON_MIN, epsilon * EPSILON_DECAY)

            if episode % SAVE_EVERY == 0:
                save_q_table(q_table)
                print("[OK] Q-table saved.")

    except KeyboardInterrupt:
        print("\n[STOP] Training stopped by user.")

    finally:
        save_q_table(q_table)

        try:
            if ser and ser.is_open:
                stop(ser)
                safe_alerts_off(ser)
                ser.close()
                print("[OK] Disconnected")
        except Exception as e:
            print(f"[FINAL STOP ERROR] {e}")

        print("[OK] Final Q-table saved and RoboCar stopped safely.")


if __name__ == "__main__":
    main()