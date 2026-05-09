# RTS Ballbot Project

A self-balancing robot that balances on top of a ball of choice using three omni-wheels arranged in a 120° pattern. The Raspberry Pi Pico reads orientation from an LSM6DS3 IMU, runs a PID control loop at 100 Hz, and drives the wheels to keep the robot upright.

---

## How It Works

The robot uses a **complementary filter** to estimate tilt from an LSM6DS3 IMU (accelerometer + gyroscope). A **PID controller** converts pitch and roll error into wheel speed commands. The three omni-wheels are oriented 120° apart, so any combination of X/Y motion can be produced by mixing their speeds.

```
IMU → accel low-pass filter → complementary filter → PID controller → wheel mix → 3× DRV8871 drivers
```

The control loop runs at **100 Hz** (10 ms period). Each iteration is timed and the loop busy-waits for the remainder of the period so timing stays deterministic regardless of computation time.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Raspberry Pi Pico (RP2040) |
| IMU | LSM6DS3 (I2C — auto-detects `0x6A` or `0x6B`) |
| Motor drivers | 3× DRV8871 (one per motor, IN1/IN2 PWM control) |
| Motors | 3× DC gear motors |
| Wheels | 3× 48mm omni-wheels, 120° apart |
| Ball | Volleyball |
| Power | 2S LiPo (~7.4V) → DRV8871 VM; buck converter → 3.3V for Pico |

### Pin Assignments

| Signal | GPIO |
|---|---|
| IMU SDA | 4 |
| IMU SCL | 5 |
| Motor A IN1 / IN2 | 0 / 1 |
| Motor B IN1 / IN2 | 2 / 3 |
| Motor C IN1 / IN2 | 8 / 9 |

> **Note:** The IMU is mounted upside-down on the frame. The driver compensates by negating the Z accelerometer axis and Z gyro axis in `imu.h`.

---

## Software

### Project Structure

```
src/
├── main.cpp              — setup, loop, global state
include/
├── config.h              — all pin and tuning constants
├── imu.h                 — LSM6DS3 class: driver, filter, calibration
├── motors.h              — DRV8871 PWM abstraction
├── controller.h          — PID controller + omni-wheel mixing
└── timing.h              — fixed-rate loop timing + jitter diagnostics
tests/
└── test_mapping.cpp      — standalone host test for wheel speed math
```

### Dependencies

- **Framework:** Arduino (RP2040 core)
- **Library:** Wire (I2C, built-in)
- **Toolchain:** PlatformIO + `platform = raspberrypi`

No external libraries required. The LSM6DS3 is driven via direct I2C register reads.

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

On first boot the robot auto-detects the IMU address, runs accelerometer calibration, then prints `Ready.` over serial at 115200 baud. **Keep the robot still and flat during the 5-second startup delay.**

---

## IMU Driver

The `LSM6DS3` class in `imu.h` handles everything:

- **Auto-address detection** — tries `0x6A` then `0x6B`, no manual config needed
- **Soft reset + BDU** — block data update prevents reading mid-conversion
- **416 Hz output rate** — accel and gyro both configured at 416 Hz, ±2g / ±250 dps
- **Accel low-pass filter** — α = 0.12, reduces motor vibration noise feeding into the tilt estimate
- **Complementary filter** — α = 0.96 (gyro weight), fuses gyro integration with accel angle
- **Boot calibration** — averages 50 accel samples at startup to zero pitch/roll bias

---

## Tuning

All tunable constants live in `include/config.h`.

| Constant | Current Value | Description |
|---|---|---|
| `KP` | `8.0` | Proportional gain — higher = stiffer response |
| `KD` | `5.0` | Derivative gain — higher = more damping |
| `KI` | `0.5` | Integral gain — present in controller but disabled |
| `LOOP_HZ` | `100` | Control loop rate |
| `MAX_SPEED` | `80` | PWM ceiling (conservative — increase after stability confirmed) |
| `DEADBAND` | `4` | PWM below which motors are forced off |
| `INJECT_DELAY_MS` | `0` | Artificial loop delay for instability demonstration |

**Suggested tuning process:**
1. Set `KD = 0`. Raise `KP` from a low value (e.g. `4`) until the robot actively fights tilt but oscillates.
2. Raise `KD` until oscillation damps out — current value of `5.0` is high relative to `KP`, adjust together.
3. Once stable, raise `MAX_SPEED` gradually to allow stronger correction.
4. Comment out `timing_print_stats()` in `main.cpp` once tuned — serial output adds latency.

---

## Real-Time Demo

Set `INJECT_DELAY_MS` in `config.h` to demonstrate the effect of missed deadlines:

```cpp
#define INJECT_DELAY_MS 20   // adds 20ms per loop
```

Expected behavior:
- **Normal (0ms)** — controller sees fresh sensor data, stable response
- **20ms injected** — controller works on stale data, oscillation increases, system destabilizes
- **Remove delay** — stability recovers

---

## Serial Output

While `timing_print_stats()` is active:

```
loop_us: 312  max_us: 418
```

Shows current and worst-case loop execution time in microseconds. Budget per loop is **5000 µs** (100 Hz). Values well under that confirm the control logic fits comfortably in the timing window.

---

## Wheel Speed Mapping

Wheel A is aligned with the robot's X axis. B and C are at +120° and +240°:

```
A =  vx
B = -0.5·vx  - 0.866·vy
C = -0.5·vx  + 0.866·vy
```

Where `vx` and `vy` come from the PID controller output:

```
vx = KP·pitch + Ki·pitch_integral + KD·pitch_rate
vy = KP·roll  + Ki·roll_integral + KD·roll_rate
```

Verify the mapping without hardware:

```powershell
cd tests
g++ test_mapping.cpp -o test_mapping; if ($?) { ./test_mapping }
```

---

## Motor Test

Uncomment `motor_test_sequence()` in the `loop()` function and comment out the PID logic to run each motor independently before enabling the full control loop. Each motor runs forward then reverse at PWM 150 for 1 second each.
