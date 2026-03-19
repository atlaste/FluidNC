# FluidNC Custom Modifications Changelog

**Base**: FluidNC upstream
**Total commits**: 271
**Files changed**: ~320 | ~40,000 lines added, ~3,600 lines removed

This document describes every feature, enhancement, and system-level change made on top of the upstream FluidNC codebase. It is intended to serve as a complete reference for building documentation and a feature showcase website.

---

## Table of Contents

1. [Lathe G-Code: CSS, Threading, Feed-Per-Rev, Diameter Mode](#1-lathe-g-code-css-threading-feed-per-rev-diameter-mode)
2. [Spindle Encoder](#2-spindle-encoder)
3. [ODrive Spindle](#3-odrive-spindle)
4. [Tool Table](#4-tool-table)
5. [Cutter Radius Compensation (G41/G42)](#5-cutter-radius-compensation-g41g42)
6. [Pneumatic Tool Turret (ATC)](#6-pneumatic-tool-turret-atc)
7. [Tailstock with Dynamic Soft Limits](#7-tailstock-with-dynamic-soft-limits)
8. [Pressure Sensor / ADC](#8-pressure-sensor--adc)
9. [S-Curve Acceleration Planner](#9-s-curve-acceleration-planner)
10. [Advanced Soft Limits System](#10-advanced-soft-limits-system)
11. [Dynamic Limits](#11-dynamic-limits)
12. [1D Compensation (Lathe Runout)](#12-1d-compensation-lathe-runout)
13. [2D Compensation (Bed Leveling)](#13-2d-compensation-bed-leveling)
14. [State Persistence (FRAM)](#14-state-persistence-fram)
15. [LED Strip Feedback](#15-led-strip-feedback)
16. [PinOn (Boot-Time Pin Configuration)](#16-pinon-boot-time-pin-configuration)
17. [Performance Profiler](#17-performance-profiler)
18. [System Monitor](#18-system-monitor)
19. [UVC Camera Streaming](#19-uvc-camera-streaming)
20. [Ethernet Support](#20-ethernet-support)
21. [WiFi Improvements](#21-wifi-improvements)
22. [Telnet Improvements](#22-telnet-improvements)
23. [USB-CDC for ESP-IDF](#23-usb-cdc-for-esp-idf)
24. [ESP-IDF Native Build System](#24-esp-idf-native-build-system)
25. [TMC SPI for ESP-IDF](#25-tmc-spi-for-esp-idf)
26. [Scheduler / Coroutine Framework](#26-scheduler--coroutine-framework)
27. [Spindle Ramping & Speed Maps](#27-spindle-ramping--speed-maps)
28. [Homing & Limit Improvements](#28-homing--limit-improvements)
29. [Comprehensive Unit Test Suite](#29-comprehensive-unit-test-suite)
30. [Console & $ Commands](#30-console--commands)
31. [Bug Fixes & Misc Improvements](#31-bug-fixes--misc-improvements)

---

## 1. Lathe G-Code: CSS, Threading, Feed-Per-Rev, Diameter Mode

A complete set of lathe-oriented G-codes has been added, bringing FluidNC to feature parity with industrial lathe controllers (LinuxCNC/Fanuc-style).

### G96 / G97 — Constant Surface Speed (CSS) / Constant RPM

- **G96 S\<speed\>** — Enables Constant Surface Speed mode. `S` is the desired surface speed in meters per minute. The controller dynamically adjusts spindle RPM based on the current cutting radius so the surface speed at the tool tip stays constant.
- **G96 D\<max_rpm\>** — Optionally sets a maximum RPM cap on the same line (LinuxCNC convention).
- **G97** — Returns to Constant RPM mode (the default). `S` is interpreted as RPM directly.
- **G50 S\<max_rpm\>** — Sets the maximum spindle speed for CSS mode. This prevents the spindle from exceeding a safe speed when cutting near the centerline.

**How CSS works internally:**
- The RPM formula is: `RPM = (surface_speed × 1000) / (π × diameter_mm)`.
- The radius is computed from the machine position on the CSS axis minus the tool length offset: `|MPos[css_axis] - TLO[css_axis]|`. This means the CSS axis (typically X for lathes) must be set up so that machine X=0 is the spindle centerline.
- During motion, each stepper segment recalculates the CSS axis position and adjusts RPM in real time, including during G1 cuts toward or away from center.
- Speed overrides are applied to the surface speed before conversion to RPM, so the cap still applies.

**Configuration (machine YAML):**
```yaml
css_axis: X   # Which axis represents the cutting radius (X for lathes)
```

### G95 — Feed Per Revolution

- **G95** — Feed rate is interpreted as mm per spindle revolution instead of mm/min.
- **G94** — Returns to standard feed per minute mode.
- Requires a spindle encoder to function. The encoder drives stepping: the stepper ISR is triggered by encoder pulses rather than a timer.
- This is a "soft sync" mode: feed override is allowed.

### G33 — Single-Pass Threading

- **G33 X\<x\> Z\<z\> K\<pitch\>** — Executes a single spindle-synchronized threading pass. `K` is the thread pitch in mm/rev.
- The motion is rigidly synchronized to the spindle encoder. Feed override is disabled during the cut.
- Before the move starts, the system inserts a "wait for index" segment that synchronizes to the spindle index pulse, ensuring consistent thread start positions.

### G76 — Multi-Pass Threading Canned Cycle

- **G76 P\<pitch\> Z\<end\> J\<first_depth\> K\<full_depth\> I\<taper\> Q\<angle\> H\<spring_passes\> R\<degression\>** — Automated multi-pass threading cycle.
  - `P` — Thread pitch (mm/rev)
  - `Z` — Thread end position (Z axis)
  - `J` — First cut depth
  - `K` — Full thread depth (total)
  - `I` — Taper amount at the Z end (0 for straight threads)
  - `Q` — Compound infeed angle in degrees (e.g., 29.5° for metric)
  - `H` — Number of spring passes (finish passes at final depth)
  - `R` — Degression factor: `1` = constant chip area (depth decreases per pass), `≥2` = constant depth per pass

**Cycle sequence per pass:**
1. Rapid to the next depth
2. G33-style synchronized cut to the Z end
3. Rapid retract off the thread
4. Rapid return to the start Z
5. Repeat until full depth, then execute spring passes

### G7 / G8 — Diameter Mode / Radius Mode

- **G7** — Diameter programming mode. X values in G-code represent diameter (standard on many lathe controllers). Internally, the value is halved to get radius.
- **G8** — Radius programming mode (default). X values represent radius directly.
- Only affects the axis configured as `css_axis` (typically X for lathes).
- Modal state is reported in status queries.

### Modal Group Summary

| G-Code | Modal Group | Description |
|--------|-------------|-------------|
| G96 | MG14 | Constant Surface Speed |
| G97 | MG14 | Constant RPM (default) |
| G50 | MG0 | Set max spindle speed for CSS |
| G95 | MG5 | Feed per revolution |
| G94 | MG5 | Feed per minute (default) |
| G33 | MG1 | Single-pass threading |
| G76 | MG1 | Multi-pass threading cycle |
| G7 | MG15 | Diameter mode |
| G8 | MG15 | Radius mode (default) |

---

## 2. Spindle Encoder

A hardware spindle encoder implementation using the ESP32 PCNT (Pulse Counter) peripheral.

### Purpose

- Provides real-time spindle position and speed feedback for G95, G33, G76, and CSS speed validation.
- Counts quadrature encoder pulses with hardware accuracy (no software overhead).

### How It Works

- **Two PCNT units**: One for total running count (`pcnt_total`), one for alarm thresholds (`pcnt_alm`).
- **Overflow handling**: Watch points at ±32767 with an ISR that accumulates into a 64-bit `totalCount`.
- **Threshold-driven stepping**: For G33/G95, the alarm PCNT fires an ISR when the encoder reaches the next step target. This ISR triggers the stepper pulse function directly, achieving rigid spindle-to-axis synchronization.
- **Fixed-point math**: Uses 1024× scaling for sub-count step accumulation (`countsPerStep` calculation), avoiding floating point in the ISR.
- **Speed validation**: `validateSpeed()` compares encoder-derived RPM to the commanded RPM within a configurable tolerance. If the spindle deviates beyond the tolerance for more than `allowed_errors` consecutive checks, an alarm is raised.
- **Index pulse**: Used for G33/G76 thread synchronization (wait-for-index before starting the cut).

### Configuration

```yaml
spindle_encoder:
  pin_a: gpio.40         # Quadrature channel A
  pin_b: gpio.41         # Quadrature channel B
  cpr: 2800              # Counts per revolution (after quadrature decoding)
  gear_ratio: -1         # Gear ratio in 1/10000 units. Negative = reverse.
  tolerance: 15          # RPM deviation tolerance (%)
  allowed_errors: 5      # Consecutive tolerance violations before alarm
```

### Hardware Notes

- Encoder pins should have 390Ω series resistors and no capacitors for clean high-frequency signals.
- Maximum encoder frequency is around 50 kHz (typical optical encoders). With 2800 CPR this supports up to ~1070 RPM.

---

## 3. ODrive Spindle

Full integration with ODrive motor controllers over CAN bus for high-performance spindle control.

### Features

- CAN bus communication using ESP32 TWAI peripheral.
- Velocity control mode with configurable gear factor.
- Supports forward (M3) and reverse (M4) with direction via gear factor sign.
- CSS-compatible: provides `setSpeedfromISR()` for real-time speed updates from the stepper ISR without blocking on ramp waits.
- Spindle ramping with configurable ramp time: `speedIsValid()` returns false during ramp-up/down to suppress encoder validation errors.
- Automatic tool changer (ATC) integration: the ODrive spindle section references which ATC to use.

### Configuration

```yaml
odrive:
  can_tx: gpio.9
  can_rx: gpio.47
  odrive_node_id: 1
  gear_factor: -0.777778   # Negative for direction inversion; ratio for gearing
  max_speed: 3000           # Maximum RPM
  atc: pneumatic_tool_turret  # Which ATC to use for tool changes
```

### CAN Protocol

Uses ODrive's CAN Simple protocol:
- `Set_Input_Vel` for speed commands
- `Get_Encoder_Estimates` for feedback
- `Set_Axis_State` for enable/disable
- Heartbeat monitoring for connection health

---

## 4. Tool Table

A full tool table system for storing tool offsets and metadata, loaded from a YAML file.

### Features

- Stores per-tool offsets for all axes (X, Y, Z, A, B, C) plus tool radius.
- Turret position mapping: maps physical turret positions (1–24) to tool numbers.
- Integrates with G10 for programmatic offset setting, G43 for tool length offset application, and G41/G42 for cutter compensation radius lookup.
- Persists to `/localfs/tooltable.yaml`.

### G-Codes

| G-Code | Description |
|--------|-------------|
| **G10 L1 P# [X Y Z ...] [R#]** | Set tool offset directly. P = tool number. R = radius. |
| **G10 L10 P# [X Y Z ...]** | Set offset so current work position equals given values. |
| **G10 L11 P# [X Y Z ...]** | Set offset relative to G59.3. |
| **G43 H#** | Apply tool length offset from tool table (H = tool number). |
| **G49** | Cancel tool length offset. |
| **T#** | Pre-select tool number. |
| **M6** | Execute tool change (physical change via ATC, then apply TLO). |
| **M61 Q#** | Set current tool number without physical change. |

### $ Commands

| Command | Description |
|---------|-------------|
| `$TT` | Display the full tool table |
| `$TTL` | Reload tool table from file |
| `$TTS` | Save tool table to file |

### File Format (`/localfs/tooltable.yaml`)

```yaml
tool1:
  name: "SCLCR 1010"
  x: 60.212
  z: 0
  radius: 0.4

tool3:
  name: "SDNCN Threading"
  x: 61.908
  z: -0.5
  radius: 0

turret:
  position1: 1
  position2: 3
  position3: 5
```

---

## 5. Cutter Radius Compensation (G41/G42)

Implements standard cutter radius compensation (also known as cutter comp or CDC), offsetting the tool path by the tool's cutting radius.

### G-Codes

| G-Code | Description |
|--------|-------------|
| **G41 D#** | Enable left cutter compensation. D = tool number for radius lookup (0 or omitted = current tool). |
| **G42 D#** | Enable right cutter compensation. |
| **G40** | Cancel cutter compensation (ramp-off move to programmed path). |

### Behavior

- The tool path is offset perpendicular to the direction of travel by the tool radius.
- **Outside corners**: An arc is inserted to smoothly round the corner (CCW for G41, CW for G42).
- **Inside corners**: A line intersection calculation determines the correct offset path.
- **First move**: Direct move to the compensated start position.
- **G40 ramp-off**: A transition move from the compensated position back to the programmed path.
- Supports XY, ZX, and YZ planes (selected via G17/G18/G19).
- Rapid moves (G0) update the compensation state but do not apply offset.
- G53 (machine coordinate moves) are not allowed while compensation is active.
- Tool radius is looked up from the tool table via the D word.

---

## 6. Pneumatic Tool Turret (ATC)

An automatic tool changer for lathes with pneumatic indexing turrets.

### Features

- Retracts to a safe position before tool change, with awareness of inside (boring) vs. outside (turning) tools for correct retract sequencing.
- Pneumatic coupling engage/disengage via digital output.
- Indexes to the target turret position using the C axis.
- Applies tool length offset from the tool table after the change.
- Optional pressure sensor check: waits for minimum pneumatic pressure (2.7 bar) before proceeding.
- Backlash compensation: overshoots the target position, then backs off to eliminate mechanical play.
- Saves and restores ATC state for persistence across reboots.

### Retract Logic

- **Outside tool (turning)**: Retract X to safe position, then Z.
- **Inside tool (boring)**: Retract Z first (pull out of bore), then X, then Z to safe.

### Configuration

```yaml
pneumatic_tool_turret:
  safe_x: 88.867                # Safe X position (machine coordinates)
  safe_z: 0                     # Safe Z position
  offsets_per_revolution: 80    # Encoder/step positions per full turret revolution
  tool_offsets: 0 10 20 30 40 50 60 70 8 18 28 38 48 58 68 78
  tool_types: "OOOOOOOOIIIIIIII"  # O=Outside, I=Inside per position
  safety_margin: 5.0            # Extra clearance in mm
  use_pressure_sensor: true     # Require min pressure before change
```

Referenced from the spindle config:
```yaml
odrive:
  atc: pneumatic_tool_turret
```

---

## 7. Tailstock with Dynamic Soft Limits

Integrates a tailstock into the soft limits system to prevent turret–tailstock collisions.

### Features

- When the tailstock is extended and a tool is loaded, it dynamically adjusts the maximum travel on the tailstock axis.
- The limit is: `max_position = tailstock_position + tool_tlo - safety_margin`.
- Implements the `DynamicLimitProvider` interface, so it works with the broader dynamic limits system.
- Tailstock position and extended state are set programmatically (e.g., via macros).

### Configuration

```yaml
tailstock:
  axis: Z                # Axis the tailstock is on
  tool: 10               # Tool number (e.g., live center)
  safety_margin: 5.0     # Clearance in mm
  spindle_index: 1       # Holder index
```

---

## 8. Pressure Sensor / ADC

Low-level analog input for reading pneumatic line pressure.

### Features

- ESP32 ADC with calibration (curve fitting) for millivolt accuracy.
- Linear mapping from voltage to pressure (0.5–4.5 V → 0–1.6 MPa).
- Used by the pneumatic tool turret to validate system pressure before tool changes.
- Configurable voltage divider ratio and sample averaging.

---

## 9. S-Curve Acceleration Planner

A jerk-limited motion planner that replaces the traditional trapezoidal acceleration profile with smooth S-curves.

### What It Does

Instead of instant acceleration changes (trapezoidal), the S-curve planner limits the rate of acceleration change (jerk). This produces smoother motion with less mechanical vibration and ringing, which is especially important for:
- Surface finish quality on lathes and mills
- Reducing mechanical stress on the machine
- Eliminating resonance excitation

### 7-Phase Velocity Profile

Each motion block has up to 7 phases:
1. **Jerk+** — Acceleration ramps from 0 to max
2. **Constant acceleration** — At maximum acceleration
3. **Jerk-** — Acceleration ramps from max to 0
4. **Cruise** — Constant velocity
5. **Jerk-** — Acceleration ramps from 0 to -max (start of decel)
6. **Constant deceleration** — At maximum deceleration
7. **Jerk+** — Acceleration ramps from -max to 0

Short moves may skip phases (e.g., no cruise phase, or triangular accel/decel).

### Architecture

- **BasePlanner** — Abstract base class with block buffer management, junction speed calculation, and the planner interface.
- **TrapezoidPlanner** — The original GRBL-style constant-acceleration planner (still available).
- **SCurvePlanner** — Extends TrapezoidPlanner with 7-phase profiles. Uses `SCurveMath` for kinematic calculations.
- **SCurveMath** — Pure math library for jerk-limited kinematics: distance, velocity, and acceleration as functions of time, plus binary search solvers for max entry/exit speeds.
- Selected via YAML; factory pattern for registration.

### Configuration

```yaml
# Use one of:
trapezoid_planner:     # Original GRBL-style (no extra config needed)

# Or:
scurve_planner:
  jerk: 50000          # Jerk limit in mm/min³ (range: 1000–1000000)
```

### Feed Hold

The S-curve planner computes proper jerk-limited deceleration for feed holds, falling back to trapezoidal deceleration if the remaining distance is too short for a full S-curve stop.

---

## 10. Advanced Soft Limits System

A modular, extensible soft limits framework supporting multiple geometric shapes.

### Components

| Component | Description |
|-----------|-------------|
| **SoftLimitsComponent** | Abstract base class. Defines `TestLimit(from, to)` interface. |
| **FixedBoundingBox** | Rectangular exclusion zone in machine coordinates. Uses slab method for line-AABB intersection testing. |
| **FixedCylinder** | Cylindrical exclusion zone (e.g., chuck, spindle housing). Tests line-circle intersection in 2D + Z range. |
| **MovingBoundingBox** | Bounding box that moves with one or more axes (e.g., a gang tool holder). Tied axes shift the box position. |
| **LimitsChecker** | Central registry. All soft limit components register here. Every motion command is tested against all registered components before execution. |

### Configuration

```yaml
fixed_box:
  enabled: true
  min_x: -50
  min_y: -100
  min_z: -200
  max_x: 50
  max_y: 100
  max_z: -150

fixed_cylinder:
  axis: Z
  center_x: 0
  center_y: 0
  start: 0
  length: 80
  radius: 75

moving_box:
  min_x: -40
  max_x: 40
  min_z: -50
  max_z: 0
  tie_x: true
  tie_z: true
```

---

## 11. Dynamic Limits

A system for axis travel limits that change at runtime.

### Features

- `DynamicLimitProvider` interface with `getDynamicLimits()`, `isActive()`, and `limitProviderName()`.
- Multiple providers can be registered (e.g., tailstock, gang tool).
- `getEffectiveLimits()` merges all active providers, taking the most restrictive limit for each axis.
- `checkMove()` and `checkPosition()` validate motion including tool length offset.
- Uses NAN to indicate "no limit" on a given axis/direction.

---

## 12. 1D Compensation (Lathe Runout)

Compensates X-axis error as a function of Z position — for correcting spindle runout or bed wear on lathes.

### How It Works

- A lookup table maps Z positions to X corrections.
- Fixed-granularity table with linear interpolation between points.
- Wraps any existing kinematics (decorator pattern): `Compensated1D` → `Cartesian`.

### Configuration

```yaml
kinematics:
  Compensated1D:
    z_min: -500
    z_max: 500
    granularity: 0.1
    file: /localfs/comp1d.yaml
    Cartesian: {}
```

### $ Commands

| Command | Description |
|---------|-------------|
| `$C1` | Show current 1D compensation table |
| `$C1P` | Set a compensation point (`$C1P Z=100 X=0.02`) |
| `$C1C` | Clear all compensation data |

---

## 13. 2D Compensation (Bed Leveling)

Compensates Z-axis error as a function of X,Y position — for bed leveling on mills.

### How It Works

- A grid of Z corrections indexed by X,Y position.
- Uses bicubic Catmull-Rom interpolation for smooth compensation between grid points.
- Decorator pattern wrapping existing kinematics.

### Configuration

```yaml
kinematics:
  Compensated2D:
    x_min: 0
    x_max: 200
    y_min: 0
    y_max: 200
    x_count: 10
    y_count: 10
    file: /localfs/comp2d.yaml
    Cartesian: {}
```

### $ Commands

| Command | Description |
|---------|-------------|
| `$C2` | Show current 2D compensation grid |
| `$C2P` | Set a compensation point (`$C2P X=5 Y=3 Z=0.05`) |
| `$C2C` | Clear all compensation data |

---

## 14. State Persistence (FRAM)

Persists full machine state across power loss or restart using external FRAM (Ferroelectric RAM) over SPI.

### What Is Saved

- **Motor step positions** for all axes (no re-homing needed after power cycle)
- **Homing status** (which axes are homed)
- **G-code parser state** (`gc_state` — active G-codes, coordinate system, feed rate, spindle state, etc.)
- **Feed, rapid, and spindle overrides**
- **Named parameters** (# variables)
- **ATC state** (current tool, turret position)

### How It Works

- A FreeRTOS task on core 0 saves state every `save_interval_ms` (default 100 ms).
- Uses SHA256 hash of the machine config to detect configuration changes. If the config changes, FRAM is invalidated and the machine requires re-homing.
- `forceSave()` is available for immediate saves (e.g., before planned shutdown).
- FRAM layout: config hash, motor steps, homing status, overrides, parser state, ATC data, parameters — all within 8 KB.

### FRAM Driver (FM25VXX)

- SPI driver for Cypress/Ramtron FM25Vxx FRAM chips (128 Kbit to 2 Mbit).
- Auto-detects chip via RDID command; falls back to FM25CL64B (8 KB) if detection fails.
- Polling SPI transfers (no DMA) for reliability.

### Configuration

```yaml
state_persistence:
  cs_pin: gpio.5
  wp_pin: NO_PIN          # Optional write protect pin
  hold_pin: NO_PIN        # Optional hold pin
  save_interval_ms: 100   # Save frequency (10–10000 ms)
  spi_freq_mhz: 20        # SPI clock (1–40 MHz)
  save_parameters: true    # Also save # variables
```

---

## 15. LED Strip Feedback

Addressable LED strip integration for real-time visual machine state feedback.

### Machine State Visualization

| State | Effect |
|-------|--------|
| **Startup** | Rainbow animation |
| **Idle** | Breathing effect with configurable brightness and period |
| **Running** | Green wave; LEDs near the tool position brighten (position tracking) |
| **Homing** | Cyan wave |
| **Hold** | Orange pulse |
| **Safety Door** | Red pulse |
| **Alarm** | Solid red |
| **Sleep** | Off |
| **Probe Touch** | White flash |

### Physical LED Mapping

LEDs can be mapped to physical machine positions. The `direction` string assigns each LED to an axis (X, Y, or space for gaps), and `travel` defines the machine-space distance for each segment. This allows the "position boost" feature to brighten LEDs near the current tool position.

### Supported LED Types

WS2812, WS2812B, WS2811 (fast/slow), SK6812, SK6812 RGBW, WS2813, WS2815.

### RMT Driver

- Uses ESP32 RMT peripheral for precise timing.
- Double buffering to prevent flicker during updates.
- Runs as a coroutine at ~6 FPS (167 ms interval).

### Configuration

```yaml
led_strip:
  pin: gpio.45
  type: WS2811_FAST
  leds: 40 39 38 37 36 35 34 33 32 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
  color_order: GRB
  direction: "XXXXXXXXXXX YYYY XXXXXXXXXXX YYYY XXXXXXXXXXX"
  travel: -300 -100 300 100 -300
  feedback:
    enabled: true
    idle_brightness: 70
    idle_breathe_period: 30000    # Breathing cycle in ms
    running_boost: 10             # Extra brightness for running state
    position_boost: 25            # Brightness boost near tool position
```

---

## 16. PinOn (Boot-Time Pin Configuration)

Sets GPIO pins to fixed values at boot time. Useful for RS485 direction control, pull-ups, or other hardware that needs a specific pin state at startup.

### Configuration

```yaml
pin_on:
  pin1: gpio.1
  value1: 1        # Set high
  pin2: gpio.2
  value2: 0        # Set low
  # Up to pin5/value5
```

---

## 17. Performance Profiler

A statistical CPU profiler with a web UI for identifying performance bottlenecks.

### Two Modes

1. **Performance Counters** — Streams CPU usage, heap stats, and FreeRTOS task info over WebSocket.
2. **PC Sampling** — 1 kHz stack sampling on both CPU cores. Records caller/callee pairs in a hash table (7,459 entries). Identifies hot functions and call paths.

### Web Interface

- Accessible at `http://<host>/performance.html`
- Loads symbol table from `/symbols.txt` (generated at build time, served gzipped)
- Features: Start/Stop profiling, flat and tree views of hotspots, task table with CPU%, C++ name demangling, filtering, CSV export.
- Real-time updates via WebSocket at `/profiler`.

### WebSocket Protocol

```json
// Commands
{"cmd": "start"}
{"cmd": "stop"}
{"cmd": "status"}

// Responses
{"type": "status", ...}
{"type": "results", "data": [{"caller": "0x4200abcd", "callee": "0x4200ef01", "cycles": 12345}, ...]}
{"type": "perf_counters", "cpu": ..., "heap": ..., "tasks": [...]}
```

### Configuration

```yaml
performance_profiler:
  enable: true
  enable_pc_sampling: true   # Enable stack profiling (uses hardware timers)
  update_rate: 1000          # Update interval in ms (100–10000)
```

### Symbol Generation

Build scripts (`generate-symbols.py`, `generate-symbols-post-build.cmake`) extract function symbols from the ELF binary for the profiler to resolve addresses to function names.

---

## 18. System Monitor

A task-manager style system monitor streamed over WebSocket.

### Data Provided

- **CPU**: Per-core usage percentage
- **Memory**: Total/free/used/largest free block, PSRAM stats
- **Tasks**: Name, priority, CPU%, stack high water mark, state
- **Uptime**

### Web Interface

- WebSocket endpoint: `ws://<host>/sysmon`
- One-way streaming of JSON data at configurable intervals.

### Configuration

```yaml
system_monitor:
  enable: true
  update_rate: 1000    # ms (100–5000)
  track_cpu: true
  track_memory: true
  track_tasks: true
```

---

## 19. UVC Camera Streaming

Streams MJPEG video from USB UVC cameras to web clients via WebSocket.

### Features

- Uses ESP32 `usb_stream` component for USB host UVC support.
- PSRAM frame buffers (1 MB each).
- Configurable resolution and frame rate.
- Suspend-when-idle: only captures frames when WebSocket clients are connected.
- Connect/disconnect event handling.

### Web Interface

- Accessible at `http://<host>/camera.html`
- WebSocket endpoint: `ws://<host>/camera` (binary JPEG frames)
- Simple viewer with connect/disconnect and frame count.

### Configuration

```yaml
uvc_camera:
  enable: true
  frame_rate_limit: 15       # 1–30 FPS
  preferred_width: 800       # 320–3840
  preferred_height: 600      # 240–2160
  buffer_count: 2            # 2–4 frame buffers
```

### Note

UVC camera and USB-CDC cannot be used simultaneously (they share the USB peripheral).

---

## 20. Ethernet Support

SPI-based Ethernet via DM9051 controller.

### Features

- DHCP or static IP configuration.
- MAC address derived from ESP32 chip ID.
- Enables all network services (WebUI, Telnet, mDNS) when connected.

### Configuration

```yaml
ethernet:
  cs_pin: gpio.10
  int_pin: gpio.4        # Optional interrupt pin
  rst_pin: NO_PIN        # Optional reset pin
  dhcp: true
  ip: 0                  # Static IP (if dhcp: false)
  gateway: 0
  netmask: 0
  spi_freq_mhz: 20
```

---

## 21. WiFi Improvements

### WiFi Connection Cache

- Caches the last successful WiFi channel and BSSID in NVS.
- On reconnect, tries the cached channel/BSSID first with `WIFI_FAST_SCAN`, dramatically reducing connection time.
- Falls back to full scan if the cached connection fails.

### Fast Scan Setting

- New setting: `WiFi/FastScan` (On/Off).
- When enabled, uses `WIFI_FAST_SCAN` to connect to the first matching AP found instead of scanning all channels.

---

## 22. Telnet Improvements

- `setNoDelay(true)` for lower latency on Telnet connections.
- Starts only when network services are available (WiFi or Ethernet connected).
- mDNS advertisement for `_telnet._tcp`.

---

## 23. USB-CDC for ESP-IDF

Native USB CDC serial channel using TinyUSB, replacing the Arduino Serial implementation when building with ESP-IDF.

### Features

- 1040-byte RX ring buffer.
- DTR/RTS signal handling for bootloader entry (same protocol as standard ESP32 USB).
- CR insertion (`\n` → `\r\n`) for terminal compatibility.
- Works with ESP-IDF 5+ and `CONFIG_TINYUSB_CDC_ENABLED`.

---

## 24. ESP-IDF Native Build System

A complete ESP-IDF build system alongside the existing PlatformIO/Arduino build.

### What Was Added

- **Top-level `CMakeLists.txt`** — ESP-IDF project definition with component directories, compiler flags (`-fexceptions`, `-fcoroutines` for C++20), and the `IDFBUILD` preprocessor define.
- **`src/CMakeLists.txt`** — Registers all FluidNC source files as an ESP-IDF component with proper dependencies (fatfs, vfs, sdmmc, lwip, esp_netif, mbedtls, usb_stream, perfmon, etc.).
- **`sdkconfig.defaults`** — Default SDK configuration targeting ESP32-S3 at 240 MHz with PSRAM, FreeRTOS trace facility, TinyUSB CDC, DM9051 Ethernet, and UVC USB host support.
- **`partitions.csv`** — Partition table with NVS, OTA, app, and LittleFS partitions.
- **`copy-machine-config.cmake`** — Build-time tool to copy machine-specific config into the data directory.
- **`build-machine.sh`** — Helper script for building with a specific machine config: `./build-machine.sh lathe`.
- **Machine config directory** (`machine_configs/`) — Named configs (default.yaml, lathe.yaml, test.yaml) that can be selected at build time via `-DMACHINE=lathe.yaml`.
- **Symbol generation** — Post-build scripts to extract ELF symbols for the performance profiler.

### Building

```bash
# Standard ESP-IDF build
idf.py build

# Build with specific machine config
idf.py -DMACHINE=lathe.yaml build

# Or via helper script
./build-machine.sh lathe
```

---

## 25. TMC SPI for ESP-IDF

ESP-IDF SPI implementation for TMC2130/TMC5160 stepper drivers.

### Why

The Arduino SPI library uses register-level access that can corrupt shared SPI buses. This implementation uses the proper ESP-IDF `spi_master` API with device-level transactions.

### Details

- SPI Mode 3, 2 MHz clock.
- Supports daisy-chained drivers (up to 8).
- Overrides `TMC2130Stepper::write()` and `read()` weak symbols.
- Polling transfers (no DMA) for the small 5-byte TMC packets.

---

## 26. Scheduler / Coroutine Framework

A C++20 coroutine-based task scheduler for non-blocking, cooperative multitasking.

### Components

| Component | Description |
|-----------|-------------|
| **Schedulable\<T\>** | C++20 coroutine type. `co_yield Timespan` for delays, `co_yield QueueMe` for suspension, `co_await` for calling sub-coroutines. |
| **EventScheduler** | Min-heap priority queue of timed events. |
| **SlowEventScheduler** | Extends EventScheduler with FreeRTOS integration (spinlocks for ISR safety, `vTaskDelay` for idle). |
| **Timer** | Platform-abstracted timing (`esp_timer_get_time()` on ESP32). |
| **Timepoint / Timespan** | Time types with literal operators (`100_msec`, `co_delay_sec(1)`). |
| **FixedMemory** | 32 KB arena allocator for coroutine frames (avoids heap fragmentation). |
| **ScopedSpinlock** | ESP32 spinlock wrapper for ISR-safe critical sections. |
| **SchedulerTask** | FreeRTOS task that runs the scheduler loop. |

### Usage

The scheduler is used by the LED strip feedback (coroutine-based animation loop), spindle encoder speed monitoring, and other periodic tasks that need cooperative scheduling without dedicated FreeRTOS tasks.

---

## 27. Spindle Ramping & Speed Maps

### Ramping

All spindle types now support ramping:
- `startRamp()` / `endRamp()` mark the beginning and end of speed changes.
- `_speedIsValidAfter` timestamp: `speedIsValid()` returns false until the ramp time expires.
- This prevents false encoder validation errors during acceleration/deceleration.

### Speed Maps

- `linearSpeeds` configuration for mapping commanded speed to actual output.
- `max_speed` configuration for all spindle types.
- Speed validation during M3/M4: the commanded speed is always validated against the spindle's speed range.

### Spindle Types Enhanced

All built-in spindle types (10V, PWM, DAC, HBridge, OnOff, VFD, Null) received updates for:
- Proper `setSpeedfromISR()` support (for CSS)
- Ramping support
- Speed validation

---

## 28. Homing & Limit Improvements

- **Homing validation**: Homing now errors out if a homed axis has no limit switches configured, preventing silent failures.
- **Soft limit quantization**: Soft limit checks now quantize positions to avoid floating-point rounding errors triggering false limit violations.
- **Limit pin improvements**: Better handling of limit pin inversion and pull-up/pull-down attributes.

---

## 29. Comprehensive Unit Test Suite

A large set of unit tests covering the new and existing functionality.

### Test Categories

| Test Suite | What It Tests |
|------------|---------------|
| **Planner Tests** | TrapezoidPlanner, SCurvePlanner, SCurveMath, integration tests with multi-block sequences |
| **Soft Limits Tests** | FixedBoundingBox, FixedCylinder, MovingBoundingBox, LimitsChecker integration |
| **Kinematics Tests** | Compensated1D, Compensated2D, testable kinematics harness |
| **Scheduler Tests** | Coroutines, EventScheduler, SlowEventScheduler, Timer, Timepoint/Timespan |
| **Motor Tests** | Motor simulator, stepping verification |
| **Spindle Tests** | Spindle simulator, speed control, ramping |
| **Configuration Tests** | YAML parsing, machine config, complete config roundtrip |
| **Pin Tests** | GPIO, ErrorPin, PinOptions, Undefined pins |
| **String/UTF8 Tests** | String utilities, UTF-8 handling |

### Test Infrastructure

- **Mock framework**: Hardware simulators for motors, spindles, GPIO, I2C, UART, CAN.
- **Test logging**: Captured log output for assertion checking.
- **Platform abstraction**: Tests run on Windows (x86) with hardware-abstracted mocks.

---

## 30. Console & $ Commands

### New $ Commands

| Command | Name | Description |
|---------|------|-------------|
| `$TT` | ToolTable/Show | Display the full tool table |
| `$TTL` | ToolTable/Load | Reload tool table from `/localfs/tooltable.yaml` |
| `$TTS` | ToolTable/Save | Save tool table to file |
| `$C1` | Comp1D/Show | Show 1D compensation table |
| `$C1P` | Comp1D/Set | Set a 1D compensation point |
| `$C1C` | Comp1D/Clear | Clear 1D compensation data |
| `$C2` | Comp2D/Show | Show 2D compensation grid |
| `$C2P` | Comp2D/Set | Set a 2D compensation point |
| `$C2C` | Comp2D/Clear | Clear 2D compensation data |

### Enhanced Status Reporting

- G7/G8 (diameter/radius mode) is included in modal state reports.
- G96/G97 (CSS/RPM mode) is included in modal state reports.
- G95 (feed per rev) mode reporting.
- Current tool number and tool length offset reporting.

---

## 31. Bug Fixes & Misc Improvements

### Stepper / Motion

- Fixed subtle bug in step timer where the LL API changed from values to masks.
- Fixed union bug in stepper ISR.
- Fixed reload and timer interaction bug in stepper code.
- Fixed direction vector implementation bugs.
- Fixed remainder calculation for encoder-driven stepping.
- AMASS (Adaptive Multi-Axis Step Smoothing) now only applies to timer mode, not encoder mode.

### GPIO / Pins

- Fixed GPIO handling bug where an intrinsic's fine print was not followed.
- Fixed pin attributes and pin mapper bugs.
- Fixed `initialon` for pins.
- Fixed step engine discovery bug.

### Configuration

- Fixed auto-create bug in configuration completion sequence.
- Fixed Parser bugs for edge cases.
- Fixed duplicate check issues.

### Networking

- Fixed WebUI server init order with mDNS.
- Fixed Ethernet code for proper ESP-IDF integration.
- Fixed VFS WDT timeout for large SD card operations.
- Fixed `std::space` crash on ESP-IDF (not implemented by IDF; graceful fallback added).

### General

- Fixed NVS initialization code.
- Fixed UART code for ESP-IDF compatibility (renamed `uart.cpp` → `fnc_uart.cpp` to avoid name clash).
- Added IRAM attributes to ISR functions for reliability.
- Fixed heap corruption bug in LED strip code.
- Fixed `unique_ptr` / `new`/`free` mismatch bugs.
- Multiple fixes to the I2C pin extender code.
- Added watchdog feeds in critical paths.

---

## Complete YAML Configuration Reference

Below is a representative machine configuration showing all new features (based on the lathe config):

```yaml
board: FluidNC-S3
name: Lathe

stepping:
  engine: I2S_STREAM
  idle_ms: 255
  pulse_us: 4
  segments: 12

spi:
  miso_pin: gpio.13
  mosi_pin: gpio.11
  sck_pin: gpio.12

# --- New: State Persistence ---
state_persistence:
  cs_pin: gpio.5
  save_interval_ms: 100
  spi_freq_mhz: 20
  save_parameters: true

# --- New: Spindle Encoder ---
spindle_encoder:
  pin_a: gpio.40
  pin_b: gpio.41
  cpr: 2800
  gear_ratio: -1
  tolerance: 15

# --- New: ODrive Spindle ---
odrive:
  can_tx: gpio.9
  can_rx: gpio.47
  odrive_node_id: 1
  gear_factor: -0.777778
  max_speed: 3000
  atc: pneumatic_tool_turret

# --- New: Pneumatic Tool Turret ---
pneumatic_tool_turret:
  safe_x: 88.867
  safe_z: 0
  offsets_per_revolution: 80
  tool_offsets: 0 10 20 30 40 50 60 70 8 18 28 38 48 58 68 78
  tool_types: "OOOOOOOOIIIIIIII"
  safety_margin: 5
  use_pressure_sensor: true

# --- New: CSS axis ---
css_axis: X

# --- New: LED Strip ---
led_strip:
  pin: gpio.45
  type: WS2811_FAST
  leds: 40 39 38 ... 1 0
  direction: "XXXXXXXXXXX YYYY XXXXXXXXXXX YYYY XXXXXXXXXXX"
  travel: -300 -100 300 100 -300
  feedback:
    enabled: true
    idle_brightness: 70
    idle_breathe_period: 30000
    running_boost: 10
    position_boost: 25

# --- New: Performance Profiler ---
performance_profiler:
  enable: true
  enable_pc_sampling: true
  update_rate: 1000

# --- New: S-Curve Planner ---
scurve_planner:
  # Default jerk settings

# Optional features (commented in real config):
# ethernet:
#   cs_pin: gpio.10
#   dhcp: true
#
# uvc_camera:
#   enable: true
#   frame_rate_limit: 15
#
# system_monitor:
#   enable: true
#   update_rate: 1000
#
# pin_on:
#   pin1: gpio.1
#   value1: 1
```

---

## New G-Code Quick Reference

| G-Code | Category | Description |
|--------|----------|-------------|
| G7 | Lathe | Diameter programming mode |
| G8 | Lathe | Radius programming mode (default) |
| G10 L1/L10/L11 | Tool Table | Set tool offsets and radius |
| G33 | Threading | Single-pass spindle-synchronized threading |
| G40 | Compensation | Cancel cutter radius compensation |
| G41 | Compensation | Cutter compensation left |
| G42 | Compensation | Cutter compensation right |
| G43 H# | Tool Table | Apply tool length offset |
| G49 | Tool Table | Cancel tool length offset |
| G50 S# | CSS | Set max spindle speed for CSS |
| G76 | Threading | Multi-pass threading canned cycle |
| G95 | Feed | Feed per revolution |
| G96 | CSS | Constant Surface Speed mode |
| G97 | CSS | Constant RPM mode (default) |
| M6 | Tool Change | Execute tool change |
| M61 Q# | Tool Change | Set tool number without physical change |
