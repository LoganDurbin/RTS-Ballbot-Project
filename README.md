# RTS Ballbot

A self-balancing robot that stands on top of a sports ball using three omni-wheels arranged in a 120° pattern. The Raspberry Pi Pico reads orientation from an IMU, runs a PD control loop at 200 Hz, and drives the wheels to keep the robot upright.

---

## How It Works

The robot uses a **complementary filter** to estimate tilt from an LSM6DS3 IMU (accelerometer + gyroscope). A **PD controller** converts pitch and roll error into wheel speed commands. The three omni-wheels are oriented 120° apart, so any combination of forward/sideways motion can be produced by mixing their speeds.

```
IMU → tilt estimate → PD controller → wheel speed mix → 3× motor drivers
```

The control loop runs at **200 Hz** (5 ms period). Each iteration is timed and the loop busy-waits for the remainder of the period so timing stays consistent regardless of how fast the computation finishes.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Raspberry Pi Pico (RP2040) |
| IMU | LSM6DS3 (I2C, address `0x6A`) |
| Motors | 3× DC motors with dual H-bridge drivers (IN1/IN2 per motor) |
| Wheels | 3× omni-wheels, 120° apart |
| Ball | Sports ball (basketball, soccer ball, etc.) |

### Pin Assignments

| Signal | GPIO |
|---|---|
| IMU SDA | 6 |
| IMU SCL | 7 |
| Motor A IN1 / IN2 | 0 / 1 |
| Motor B IN1 / IN2 | 2 / 3 |
| Motor C IN1 / IN2 | 4 / 5 |

---

## Software

### Project Structure

```
src/
├── main.cpp              — setup, loop, global state
├── include/
│   ├── config.h          — all pin and tuning constants
│   ├── imu.h             — LSM6DS3 driver + complementary filter
│   ├── motors.h          — PWM motor driver
│   ├── controller.h      — PD controller + omni-wheel mixing
│   └── timing.h          — fixed-rate loop timing + diagnostics
└── tests/
    └── test_mapping.cpp  — standalone host test for wheel speed math
```

### Dependencies

- **Framework:** Arduino (Mbed core for RP2040)
- **Library:** Wire (I2C)
- **Toolchain:** PlatformIO + `platform = raspberrypi`

---

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- USB cable to flash the Pico

### Build & Flash

```bash
# Build
pio run

# Flash (hold BOOTSEL on Pico while plugging in, then run)
pio run --target upload

# Open serial monitor
pio device monitor
```

On first boot the robot prints calibration status and then `Ready.` over serial at 115200 baud.

---

## Tuning

All tunable constants live in [src/include/config.h](src/include/config.h).

| Constant | Default | Description |
|---|---|---|
| `KP` | `15.0` | Proportional gain — higher = stiffer response |
| `KD` | `0.5` | Derivative gain — higher = more damping |
| `LOOP_HZ` | `200` | Control loop rate |
| `MAX_SPEED` | `255` | PWM ceiling (0–255) |
| `DEADBAND` | `10` | PWM below which motors are forced off |
| `INJECT_DELAY_MS` | `0` | Artificial loop delay for instability testing |

**Suggested tuning process:**
1. Start with `KP` low (e.g. `5`) and `KD = 0`. Increase `KP` until the robot resists tipping but oscillates.
2. Increase `KD` to damp the oscillation.
3. Comment out `timing_print_stats()` in `main.cpp` once tuned — it adds serial latency.

### Gyro Calibration

On every boot, `calibrate_gyro(200)` averages 200 IMU samples to zero the gyro bias. **Keep the robot still during the first ~1 second after power-on.**

---

## Serial Output

While `timing_print_stats()` is active, each loop prints:

```
loop_us: 312  max_us: 418
```

This shows the current and worst-case loop execution time in microseconds. The budget per loop is **5000 µs** (200 Hz). Values well under that leave margin for the complementary filter and motor writes.

---

## Wheel Speed Mapping

Motor outputs are mixed from cartesian velocity commands `(vx, vy)`:

```
A =  vy
B = -0.866·vx  - 0.5·vy
C =  0.866·vx  - 0.5·vy
```

This is derived from the 120°-spaced omni-wheel geometry. You can verify the mapping by running the host test:

```powershell
cd src/tests
g++ test_mapping.cpp -o test_mapping; if ($?) { ./test_mapping }
```
