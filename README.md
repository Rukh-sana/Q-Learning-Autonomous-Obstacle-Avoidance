# 🤖 Q-Learning Autonomous Obstacle Avoidance — Hanback RoboCar

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%20%2B%20ATmega128-green.svg)]()
[![Algorithm](https://img.shields.io/badge/Algorithm-Q--Learning-orange.svg)]()
[![Hardware](https://img.shields.io/badge/Hardware-Embedded%20RL-red.svg)]()
[![Status](https://img.shields.io/badge/Status-Validated%20Prototype-brightgreen.svg)]()

> **Reinforcement Learning deployed on a real embedded robotic platform** — Q-learning for autonomous obstacle avoidance using Raspberry Pi high-level control, ATmega128 motor firmware, ultrasonic sensing, and a hard-safety override layer.

---

## 📌 Project Overview

This project implements a **Q-learning-based autonomous navigation system** on a Hanback RoboCar physical platform. Rather than simulated environments, the agent operates on **real hardware under real-world sensor noise**, bridging the gap between RL theory and embedded deployment.

The system uses a **two-tier control hierarchy**:
- **Raspberry Pi** — high-level Q-learning policy in Python
- **ATmega128 microcontroller** — low-level motor PWM control via Flowcode firmware

A **safety override layer** runs before the learned policy at every step, ensuring the robot never enters collision-critical states during early training phases — a design pattern directly applicable to safe autonomous systems.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Raspberry Pi                        │
│   ┌─────────────────────────────────────────────┐   │
│   │         Q-Learning Controller (Python)       │   │
│   │  State → Safety Check → Q-table → Action    │   │
│   └─────────────────┬───────────────────────────┘   │
│                     │ Serial (115200 baud)            │
└─────────────────────┼───────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────┐
│               ATmega128 Microcontroller              │
│   ┌─────────────────────────────────────────────┐   │
│   │   Flowcode Firmware — PWM Motor Control      │   │
│   └──────────┬──────────────────────┬───────────┘   │
│              │                      │                │
│         Left Motor             Right Motor           │
│         (DC + Driver)          (DC + Driver)         │
└──────────────────────────────────────────────────────┘
                      ▲
         ┌────────────┴────────────┐
    Front USS (FUSS)         Rear USS (BUSS)
    Ultrasonic Sensor        Ultrasonic Sensor
```

---

## 🧠 Reinforcement Learning Design

### State Space — 12 States
| Front Sensor | Rear Sensor | Combined States |
|---|---|---|
| DANGER (< 10 cm) | DANGER (< 10 cm) | 4 × 3 = **12 states** |
| NEAR (10–20 cm) | NEAR (10–20 cm) | |
| MEDIUM (20–40 cm) | CLEAR (≥ 40 cm) | |
| CLEAR (≥ 40 cm) | | |

### Action Space — 9 Actions
`forward` · `slow_forward` · `soft_left` · `soft_right` · `turn_left` · `turn_right` · `reverse` · `escape` · `stop`

### Q-Learning Parameters
```python
alpha = 0.10    # Learning rate  — gradual Q-table updates
gamma = 0.90    # Discount factor — prioritise long-term safe movement
epsilon = 0.20  # Exploration rate — ε-greedy policy
```

### Reward Function
| Condition | Reward |
|---|---|
| Safe forward (CLEAR) | **+10** |
| Turn away (NEAR) | **+8** |
| Reverse from danger | **+7** |
| Stop at danger | **+6** |
| Cautious (MEDIUM) | **+5** |
| Forward into obstacle | **−10** |
| Unsafe reverse | **−10** |
| Collision risk | **−15** |
| Repeated stop | **−2** |

### Bellman Update
```
Q(s, a) ← Q(s, a) + α [ r + γ · max Q(s', a') − Q(s, a) ]
```

---

## 🛡️ Safety Override Layer

A hard-rule safety layer runs **before** the Q-table is consulted at every control step:

```
Read Sensors
     │
     ├─ Sensor UNKNOWN?  →  STOP immediately
     ├─ Front DANGER?    →  ESCAPE sequence
     │                        ① Stop → ② Reverse → ③ Stop → ④ Pivot Left → ⑤ Stop
     ├─ Front NEAR?      →  Turn away
     └─ Otherwise        →  Consult Q-table
```

This design ensures **zero collisions** even during early training when the Q-table is uninitialised — a key safety engineering principle for real-world robot deployment.

---

## 📡 Serial Communication Protocol

Commands sent from Raspberry Pi to ATmega128 at **115,200 baud**:

| Command | Description | Example |
|---|---|---|
| `PWMRFxxxx` | Right motor forward | `PWMRF0650` |
| `PWMRRxxxx` | Right motor reverse | `PWMRR0700` |
| `PWMLFxxxx` | Left motor forward | `PWMLF0650` |
| `PWMLRxxxx` | Left motor reverse | `PWMLR0700` |
| `FUSS` | Request sensor readings | `FUSS#95%187` |

### Sensor Noise Handling
Readings prefixed with ~5200 (hardware artifact) are normalised:
```python
if raw_reading > 5000:
    normalised = raw_reading - 5200  # e.g., 5270 → 70 → NEAR
```

---

## 📊 Experimental Results

**Testing Setup:** Indoor carpeted surface · SSH-controlled Raspberry Pi · Evidence via terminal logs, Q-table snapshots, and video

| State | Learned Action | Observed Behaviour |
|---|---|---|
| F:clear \| B:clear | Forward | Stable forward movement ✅ |
| F:medium \| B:clear | Soft turn | Early direction correction ✅ |
| F:near \| B:clear | Turn away | Safety override triggered ✅ |
| F:danger \| B:clear | Escape | Reverse + pivot response ✅ |
| F:unknown | Stop | Safe fallback behaviour ✅ |

**Learned Q-Table Policy (converged):**
```
F:clear  | B:clear  → forward
F:medium | B:clear  → soft_turn
F:near   | B:clear  → turn_away
F:danger | B:clear  → escape / turn
F:unknown           → stop
```

**Sample terminal log:**
```
[RECV] 'FUSS#91%187#95%187'
[PARSE] Raw F=91 → 91 | Raw B=187 → 187
Step=44 | State=F:medium|B:clear | Action=soft_right

[RECV] 'FUSS#5270%219#5270%219'
[PARSE] Raw F=5270 → 70  (noise normalised)
[SAFETY OVERRIDE] turn_left
Step=45 | State=F:near|B:clear  | Action=turn_left
```

---

## 🔧 Hardware Components

| Component | Role |
|---|---|
| Hanback RoboCar platform | Physical robot chassis |
| Raspberry Pi | High-level Q-learning controller |
| ATmega128 microcontroller | Low-level PWM motor firmware |
| Front ultrasonic sensor (FUSS) | Forward obstacle detection |
| Rear ultrasonic sensor (BUSS) | Rear clearance detection |
| DC motors + driver | Differential drive actuation |
| USB serial interface | Raspberry Pi ↔ ATmega128 link |

---

## 📁 Repository Structure

```
robocar-rl/
├── raspberry_pi/           # High-level Q-learning controller
│   ├── q_learning_agent.py # Core RL agent
│   ├── serial_interface.py # Serial communication handler
│   ├── sensor_parser.py    # Ultrasonic sensor parsing + noise normalisation
│   ├── safety_override.py  # Hard-rule safety layer
│   └── main.py             # Entry point
├── atmega128/              # Low-level embedded firmware
│   └── ROBOCAR_ATMEGA128_Code/  # Flowcode firmware for motor PWM control
├── logs/                   # Training logs and Q-table snapshots
│   └── training_log.csv
├── results/                # Q-table convergence data
├── docs/                   # Project report and presentation slides
└── README.md
```

---

## 🚀 How to Run

### Prerequisites
```bash
pip install pyserial numpy
```

### Hardware Setup
1. Flash ATmega128 with Flowcode firmware from `atmega128/`
2. Connect Raspberry Pi to ATmega128 via USB serial
3. Connect front and rear ultrasonic sensors

### Run the Agent
```bash
cd raspberry_pi/
python main.py --port /dev/ttyUSB0 --baud 115200 --episodes 100
```

### Key Parameters (editable in `q_learning_agent.py`)
```python
ALPHA   = 0.10   # Learning rate
GAMMA   = 0.90   # Discount factor
EPSILON = 0.20   # Exploration rate
```

---

## 🔮 Future Extensions

- [ ] Add left/right ultrasonic sensors (expand to 48-state space)
- [ ] Integrate Raspberry Pi Camera for vision-based navigation
- [ ] Replace Q-table with Deep Q-Network (DQN) for continuous state spaces
- [ ] Implement ROS2 interface for modular robot middleware integration
- [ ] Record collision count, success rate, and reward curves automatically
- [ ] Sim-to-real transfer using Gazebo/PyBullet for pre-training

---

## 📄 Academic Context

This project was completed as part of **ML for Embedded Systems** coursework at **NUST College of E&ME**, under the supervision of **Dr. Ali Hassan**. It demonstrates deployment of reinforcement learning on a resource-constrained embedded platform with real-world sensor noise — bridging the gap between theoretical RL and physical robot systems.

---

## 👩‍💻 Author

**Rukhsana Perveen**
Embedded AI Researcher | PhD Candidate (AI & Robotics)
📧 Rukhsanaperveench@gmail.com
🔗 [LinkedIn](https://www.linkedin.com/in/rukhsana-perveen-0b044411a)
🔗 [GitHub](https://github.com/Rukh-sana)

---

## 📜 License

MIT License — see [LICENSE](LICENSE) for details.
