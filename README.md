# XIAO ESP32-C3 + ICM-20948 — NED 9-axis USB streamer

A compact Arduino/PlatformIO firmware for the **Seeed XIAO ESP32-C3 + ICM-20948**
(9-axis, AK09916 magnetometer on board) that samples all nine axes at **100 Hz** and
streams them as one line of CSV per sample over **USB CDC (virtual serial)** in a
**NED (X=North, Y=East, Z=Down) right-handed frame**.

It is designed as the *sensor front-end* for a host that fuses the stream — for example
the external-IMU serial injection path in [pypilot](https://github.com/pypilot/pypilot)
(`imuserial.py` → `RTIMU.setExtIMUData()`). The host may be on Windows/Linux/macOS; nothing
here depends on the host platform.

- Sample rate: 100 Hz (non-blocking 10 ms timer), USB CDC 115200 baud
- Output: physical units — accel **[g]**, gyro **[rad/s]**, magnetometer **[µT]**
- Accel expressed as the **gravity vector**: a level, stationary sensor reads `az ≈ +1.0`
- Robustness: I2C timeouts, frozen-bus and out-of-range self-checks with automatic
  `ESP.restart()` (see [Diagnostics](#7-diagnostics--fail-safe))
- Optional 11th column `ts_us` (device `micros()` at read completion) lets the host
  recover the **true sample interval (dynamic dt)** even when USB batching delays delivery

---

## 1. Repository layout

| path | purpose |
|---|---|
| `src/main.cpp` | firmware (100 Hz NED 9-axis CSV streamer) |
| `platformio.ini` | PlatformIO build config (`seeed_xiao_esp32c3`, Arduino) |
| `visualizer/gauge_monitor.py` | realtime analog-gauge monitor of the 9 streamed values |
| `visualizer/pyproject.toml`, `visualizer/uv.lock` | Python (uv) environment for the visualizer |

---

## 2. Hardware & wiring

Example board: Adafruit ICM-20948 9DoF breakout (I2C address `0x69`, AD0 = High).

| ICM-20948 | XIAO ESP32-C3 | note |
|---|---|---|
| VIN / 3Vo | 3V3 | 3.3 V power |
| GND | GND | |
| SDA | D4 (GPIO 6) | I2C |
| SCL | D5 (GPIO 7) | I2C, 400 kHz |

- I2C timeout 5 ms (prevents an infinite wait if the bus hangs)
- keep wires 3–5 cm; for magnetometer work keep ferrous metal, PCs and power supplies
  at least ~50 cm away

---

## 3. Build & upload

1. Open this folder in VS Code with the **PlatformIO** extension.
2. Click **✓ Build**, then **→ Upload**.
3. Open the PlatformIO Serial Monitor (115200) — you should see
   `#stream start 100Hz` followed by one CSV line per sample.

`platformio.ini`:

```ini
[env:seeed_xiao_esp32c3]
platform = espressif32
board = seeed_xiao_esp32c3
framework = arduino
monitor_speed = 115200
lib_deps = wollewald/ICM20948_WE @ ^1.2.9
```

---

## 4. Serial output format

One sample per line, ASCII, `\n` (or `\r\n`) terminated:

```
seq,ts_us,ax,ay,az,gx,gy,gz,mx,my,mz
```

| field | meaning |
|---|---|
| `seq` | uint16 rolling sequence 0..65535 (detects resets / drops) |
| `ts_us` | uint32 `micros()` taken right after the sensor read completed (~71.5 min wrap). The host derives the true sample interval `dt` from the delta between successive samples |
| `ax..az` | acceleration [g], 5 decimals. Gravity-vector notation: level and stationary ⇒ `az ≈ +1.0` |
| `gx..gz` | gyroscope [rad/s], 5 decimals. Right-hand positive (see §5) |
| `mx..mz` | magnetometer [µT], 2 decimals. NED: `mz` positive downward |

Legacy hosts that do not understand `ts_us` may use the 10-column format
(`seq,ax,...,mz`); `imuserial.py` in pypilot accepts both.

Lines starting with `#` are diagnostics (`#stream start 100Hz`, `#RESTART <reason>`, …).

---

## 5. Sensor axis alignment

> The point of this section: **you do not need to trust the vendor axis drawing.** With
> two short physical tests (accelerometer, gyroscope) you can check — and if necessary
> re-map — *any* IMU to a NED frame yourself. The ICM-20948 mapping used by this firmware
> is given in §5.6 as a worked example.

### 5.1 The NED frame

NED is the right-handed frame commonly used for marine/aerial attitude:

- **X = North**
- **Y = East**
- **Z = Down** (towards the centre of the Earth)

Two conventions make every axis test unambiguous:

- **Accelerometer** is reported as the **gravity vector**. A stationary sensor on a level
  table has its sensitive axis pointing at the Earth's centre, therefore the reading on
  that axis is **positive**: level ⇒ `az ≈ +1.0 g`.
- **Gyroscope** is positive for a **right-hand / right-hand-screw rotation**: rotate the
  board **clockwise when you look along the +axis direction** ⇒ the value on that axis is
  positive.

### 5.2 Accelerometer check — “point an axis down, it must read +1 g”

1. Place the sensor level and read the stream.
   The axis that reads ≈ `+1.0` (and the other two ≈ 0) is the **Z (down)** axis.
2. For each of the three axes in turn: **point that axis straight down** (towards the
   Earth's centre). It must read ≈ `+1.0`:
   - reads `+1.0` ⇒ keep that channel as-is
   - reads `-1.0` ⇒ invert that channel (multiply by `-1`)

```
roll the board so the X axis points straight down … ax should read ≈ +1.0
roll the board so the Y axis points straight down … ay should read ≈ +1.0
roll the board so the Z axis points straight down … az should read ≈ +1.0
```

### 5.3 Gyroscope check — “clockwise, seen along the +axis, is positive”

For each axis, hold the board so the **+axis points towards you**, then rotate the board
**clockwise** (right-hand screw). The corresponding gyro channel must go **positive**.

Easy-to-perform equivalents:

| axis | test | expect |
|---|---|---|
| +Z (down) | board flat on the table, rotate **clockwise seen from above** | `gz > 0` |
| +X (north) | rotate **clockwise seen looking north** (board standing, X toward you) | `gx > 0` |
| +Y (east) | rotate **clockwise seen looking east** | `gy > 0` |

### 5.4 Magnetometer check

On a level surface point the intended **+X toward magnetic north**:

- `mx` should read **positive**, `my ≈ 0`
- In the northern hemisphere the field also dips downward, so `mz > 0` (Z is down)

### 5.5 Step-by-step mapping for a sensor with unknown axes

Do the checks top to bottom; each step decides one part of the mapping.

1. **Find +Z.** Level the sensor. The channel that reads ≈ `+1.0 g` is the down axis.
2. **Fix the Z sign.** Point that face straight down; if it reads `-1.0`, invert the
   channel. *(Flipping a single axis turns a right-handed frame into a left-handed one.
   Whenever you flip an axis, flip a second one together with it — e.g. Y *and* Z for the
   gyro — so the frame stays right-handed. See the ICM-20948 example below.)*
3. **Find +X.** Level the board and point its intended “forward” edge at magnetic north.
   The horizontal channel that reads positive is X. If the wrong channel responds, swap
   X and Y; if it reads negative, invert it.
4. **Verify Y by handedness.** NED is right-handed (`X × Y = Z`, all positive). With
   X = north and Z = down, Y must be east: point the Y edge east and the magnetometer Y
   channel must be positive.
5. **Fix the gyro signs.** Run the three gyro tests of §5.3 and invert the channels that
   come out negative.
6. **Record and apply.** You now have an axis order plus a per-axis `±1` sign vector.
   Apply it in your firmware (multiply each physical channel by `±1`, or configure your
   IMU driver's axis-mapping settings), then re-run the checks to confirm.

### 5.6 Worked example: ICM-20948 in this firmware

Sensor facts (specific to the ICM-20948):

- The accel/gyro **chip axes are ~NWU** (X=north, Y=west, Z=up).
- The magnetometer **AK09916 is a separate die** whose raw axes already come out in NED.
- This firmware therefore needs **no axis swap, only sign flips** (`±1`):

```cpp
static const int8_t ACCEL_SIGN[3] = { -1,  1,  1 };  // accel: see note below
static const int8_t GYRO_SIGN[3]  = {  1, -1, -1 };  // gyro : NWU -> NED: flip Y & Z
static const int8_t MAG_SIGN[3]   = {  1,  1,  1 };  // mag  : AK09916 raw is already NED
```

The accel signs are the *combined* result of two steps: negating the chip's specific
force (measured positive *upward*) to obtain the gravity vector, and flipping Y and Z to
go from the chip's NWU axes to the NED frame. On X only the first step applies (−1); on
Y and Z the two −1's cancel (+1). The gyro needs only the NWU→NED flip, and because
inverting *only* Z would make the frame left-handed, Y and Z are flipped together — a
180° rotation about X — which keeps the frame right-handed. The AK09916 magnetometer is a
separate die that already outputs NED, so it needs no flip.

Result (all checks of §5.2–§5.4 pass):

- level, component side up: `ax ≈ 0, ay ≈ 0, az ≈ +1.0`
- clockwise rotation seen from above: `gz > 0`
- +X (board X, printed) pointing to magnetic north: `mx > 0`

---

## 6. Realtime gauge monitor

`visualizer/` is a [uv](https://docs.astral.sh/uv/)-managed Python project. It prints the
nine streamed values as an analog-tachometer-style 3×3 gauge (rows = accel/gyro/mag,
columns = X/Y/Z) — the easiest way to watch the axis checks of §5 live.

```powershell
cd visualizer
uv run gauge_monitor.py                      # dummy fixed values (no USB needed)
uv run gauge_monitor.py --live --port COM10  # read the real device
uv run gauge_monitor.py --selftest           # drawing self test, no GUI window
```

- Gauge spans are derived from the sensor full scale (`ACCEL_FS_G`, `GYRO_FS_DPS`,
  `MAG_FS_UT` at the top of the file — change as needed).
- `DUMMY_DATA = True` shows a fixed NED “level” sample (`az = +1.0`) without a device;
  `--live` switches to the real COM port.
- Find your port with `pio device list`.

---

## 7. Diagnostics & fail-safe

| ID | trigger | action |
|---|---|---|
| init | sensor init fails | retry 3×, then `ESP.restart()` |
| I2C | NACK ×10, or a read hanging >8 ms ×10 | `ESP.restart()` |
| frozen | all 9 axes identical for 50 frames (~0.5 s) | `ESP.restart()` |
| out-of-range | accel norm outside 0.5–1.9 g, gyro >±30 rad/s, or mag norm outside 10–100 µT for ~1 s | `ESP.restart()` |

The restart reason is printed as `#RESTART <reason>` (`i2c-nack`, `9axis-frozen`,
`range-out`, `init-failed`, …).

---

## 8. Dependencies & license

- Hardware: Seeed XIAO ESP32-C3, an ICM-20948 breakout (I2C `0x69`)
- Firmware library: `wollewald/ICM20948_WE` (^1.2.9), pulled in by PlatformIO
- Visualizer: Python 3 + matplotlib/numpy, managed by `uv`

See the license headers in `src/main.cpp` / `visualizer/` for terms.


