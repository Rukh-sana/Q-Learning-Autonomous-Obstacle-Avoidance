import serial
import time
import re


class RoboCarSerial:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

        self.last_front = None
        self.last_back = None
        self.last_sensor_time = 0

        # CONFIRMED WORKING FROM YOUR test_combined.py
        self.command_delay = 0.30

    # =====================================================
    # CONNECTION
    # =====================================================
    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=1
            )
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"[OK] Connected on {self.port} at {self.baudrate} baud")
            return True

        except serial.SerialException as e:
            print(f"[ERROR] Serial connection failed: {e}")
            return False

    def disconnect(self):
        try:
            self.emergency_stop()
        except Exception:
            pass

        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OK] Disconnected")

    # =====================================================
    # BASIC SERIAL
    # =====================================================
    def send_command(self, cmd, delay=None):
        if not self.ser or not self.ser.is_open:
            print("[ERROR] Serial not connected")
            return False

        if delay is None:
            delay = self.command_delay

        try:
            self.ser.write(cmd.encode("ascii"))
            self.ser.flush()
            print(f"[SENT] {cmd}")
            time.sleep(delay)
            return True

        except serial.SerialException as e:
            print(f"[ERROR] Serial command failed: {e}")
            return False

    def send_and_read(self, cmd, delay=0.18, read_size=350):
        if not self.ser or not self.ser.is_open:
            print("[ERROR] Serial not connected")
            return None

        try:
            self.ser.reset_input_buffer()
            time.sleep(0.02)

            self.ser.write(cmd.encode("ascii"))
            self.ser.flush()
            print(f"[SENT] {cmd}")

            time.sleep(delay)

            raw = self.ser.read(read_size).decode("ascii", errors="ignore").strip()
            print(f"[RECV] {repr(raw)}")
            return raw

        except serial.SerialException as e:
            print(f"[ERROR] Serial read failed: {e}")
            return None

    def _speed(self, value):
        value = int(value)
        value = max(0, min(1023, value))
        return f"{value:04d}"

    # =====================================================
    # MOTOR CONTROL
    # CONFIRMED WORKING MAPPING
    # =====================================================
    #
    # Forward:
    #   PWMRR0000
    #   PWMLR0000
    #   PWMRFxxxx
    #   PWMLFxxxx
    #
    # Reverse:
    #   PWMRF0000
    #   PWMLF0000
    #   PWMRRxxxx
    #   PWMLRxxxx
    #
    # Soft left:
    #   Right forward faster, left forward slower
    #
    # Soft right:
    #   Right forward slower, left forward faster
    #
    # Turn left:
    #   Right forward + left reverse
    #
    # Turn right:
    #   Left forward + right reverse
    # =====================================================

    def stop(self):
        if not self.ser or not self.ser.is_open:
            return

        self.send_command("PWMRF0000")
        self.send_command("PWMRR0000")
        self.send_command("PWMLF0000")
        self.send_command("PWMLR0000")

    def hard_stop(self):
        for _ in range(2):
            self.stop()
            time.sleep(0.05)

    def emergency_stop(self):
        if not self.ser or not self.ser.is_open:
            return

        self.stop()
        time.sleep(0.1)

        self.buzzer_off()
        self.front_led_off()
        self.back_led_off()

    def forward(self, right_speed=700, left_speed=None):
        if left_speed is None:
            left_speed = right_speed

        self.send_command("PWMRR0000")
        self.send_command("PWMLR0000")
        self.send_command(f"PWMRF{self._speed(right_speed)}")
        self.send_command(f"PWMLF{self._speed(left_speed)}")

    def reverse(self, right_speed=700, left_speed=None):
        if left_speed is None:
            left_speed = right_speed

        self.send_command("PWMRF0000")
        self.send_command("PWMLF0000")
        self.send_command(f"PWMRR{self._speed(right_speed)}")
        self.send_command(f"PWMLR{self._speed(left_speed)}")

    def soft_left(self, right_speed=700, left_speed=None):
        if left_speed is None:
            left_speed = int(right_speed * 0.70)

        self.send_command("PWMRR0000")
        self.send_command("PWMLR0000")
        self.send_command(f"PWMRF{self._speed(right_speed)}")
        self.send_command(f"PWMLF{self._speed(left_speed)}")

    def soft_right(self, right_speed=700, left_speed=None):
        if left_speed is None:
            left_speed = right_speed
            right_speed = int(left_speed * 0.70)

        self.send_command("PWMRR0000")
        self.send_command("PWMLR0000")
        self.send_command(f"PWMRF{self._speed(right_speed)}")
        self.send_command(f"PWMLF{self._speed(left_speed)}")

    def turn_left(self, speed=700):
        self.send_command("PWMRR0000")
        self.send_command("PWMLF0000")
        self.send_command(f"PWMRF{self._speed(speed)}")
        self.send_command(f"PWMLR{self._speed(speed)}")

    def turn_right(self, speed=700):
        self.send_command("PWMRF0000")
        self.send_command("PWMLR0000")
        self.send_command(f"PWMLF{self._speed(speed)}")
        self.send_command(f"PWMRR{self._speed(speed)}")

    # =====================================================
    # LED / BUZZER
    # =====================================================
    def front_led_on(self):
        return self.send_command("FLEDON", delay=0.08)

    def front_led_off(self):
        return self.send_command("FLEDOF", delay=0.08)

    def back_led_on(self):
        return self.send_command("BLEDON", delay=0.08)

    def back_led_off(self):
        return self.send_command("BLEDOF", delay=0.08)

    def buzzer_on(self):
        return self.send_command("BUZZON", delay=0.08)

    def buzzer_off(self):
        return self.send_command("BUZZOF", delay=0.08)

    # =====================================================
    # ULTRASONIC
    # =====================================================
    def _normalize_raw_value(self, value):
        if value is None:
            return None

        try:
            value = int(value)
        except (TypeError, ValueError):
            return None

        # Examples from your logs: 5277 -> 77, 5318 -> 118
        if 5000 <= value <= 5600:
            normalized = value - 5200
            if 0 <= normalized <= 700:
                return normalized
            return None

        if 0 <= value <= 700:
            return value

        return None

    def _clean_back_value(self, back):
        if back is None:
            return 999

        back = self._normalize_raw_value(back)

        if back is None:
            return 999

        if back < 0 or back > 700:
            return 999

        return back

    def _is_valid_front(self, front):
        if front is None:
            return False

        front = self._normalize_raw_value(front)

        if front is None:
            return False

        return 0 <= front <= 700

    def _extract_pair(self, raw):
        if raw is None:
            return None, None

        matches = re.findall(r"#\s*(\d+)\s*%\s*(\d+)", raw)

        if not matches:
            return None, None

        valid_pairs = []

        for front_str, back_str in matches:
            front_raw = int(front_str)
            back_raw = int(back_str)

            front = self._normalize_raw_value(front_raw)
            back = self._clean_back_value(back_raw)

            print(f"[PARSE] Raw F={front_raw} -> {front} | Raw B={back_raw} -> {back}")

            if self._is_valid_front(front):
                valid_pairs.append((front, back))

        if not valid_pairs:
            return None, None

        return valid_pairs[-1]

    def _update_last_valid(self, front, back):
        if self._is_valid_front(front):
            self.last_front = front
            self.last_back = self._clean_back_value(back)
            self.last_sensor_time = time.time()

    def _get_last_valid_if_fresh(self, max_age=0.8):
        if self.last_front is None:
            return None, None

        if time.time() - self.last_sensor_time <= max_age:
            print(f"[WARN] Using last valid sensor: F={self.last_front} B={self.last_back}")
            return self.last_front, self.last_back

        return None, None

    def read_both_ultrasonic(self):
        for _ in range(2):
            raw = self.send_and_read("FUSS", delay=0.18, read_size=350)
            front, back = self._extract_pair(raw)

            if self._is_valid_front(front):
                self._update_last_valid(front, back)
                return {
                    "front": front,
                    "back": back,
                    "front_raw": raw,
                    "back_raw": raw,
                }

        last_front, last_back = self._get_last_valid_if_fresh(max_age=0.8)

        return {
            "front": last_front,
            "back": last_back,
            "front_raw": None,
            "back_raw": None,
        }