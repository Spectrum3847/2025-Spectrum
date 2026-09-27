# SpectrumLib

Reusable robot library shared across Spectrum teams (3847, 8515, Flash, etc.). Contains framework abstractions, hardware wrappers, telemetry utilities, and simulation helpers designed to carry forward year-to-year.

---

## Package Structure

```
frc.spectrumLib
├── framework/      Base classes and interfaces for robot and subsystem architecture
├── hardware/       Hardware wrappers: TalonFX, CANcoder, servo, and RIO identity
├── telemetry/      Logging, tunable values, and battery usage tracking
├── util/           General-purpose utilities and math helpers
├── mechanism/      Abstract TalonFX-driven mechanism base class
├── gamepads/       Xbox controller abstraction with deadbanding and rumble
├── leds/           CTRE CANdle wrapper with built-in pattern library
├── sim/            Mechanism2d simulation helpers (arm, roller, linear)
├── swerve/         Swerve-specific utilities (SysID, Maple-Sim bridge)
└── vision/         Limelight helpers and vision logging
```

---

## Packages

### `framework`

Core structural interfaces and base classes.

|      Class      |                                                   Description                                                   |
|-----------------|-----------------------------------------------------------------------------------------------------------------|
| `SpectrumRobot` | Extends `TimedRobot`; silences joystick connection warnings and extends loop overrun watchdog timeout to 200 ms |
| `SpectrumState` | Named boolean state backed by a WPILib `Trigger`; supports timed, toggled, and command-driven state transitions |

---

### `hardware`

Low-level hardware wrappers and robot identity constants.

|          Class           |                                                       Description                                                       |
|--------------------------|-------------------------------------------------------------------------------------------------------------------------|
| `Rio`                    | Enum mapping RoboRIO serial numbers to robot identities; exposes `Rio.CANIVORE` and `Rio.RIO_CANBUS` bus name constants |
| `SpectrumCANcoder`       | Configures a CANcoder and wires it into a `TalonFX` as Remote, Fused, or Sync feedback                                  |
| `SpectrumCANcoderConfig` | Configuration holder for CANcoder offset, gear ratios, inversion, and attachment flag                                   |
| `SpectrumServo`          | PWM servo wrapper that also implements `Subsystem`                                                                      |
| `TalonFXFactory`         | Factory for creating `TalonFX` instances with consistent default configuration                                          |

---

### `telemetry`

Logging, alerts, and runtime-tunable values.

|      Class      |                                        Description                                        |
|-----------------|-------------------------------------------------------------------------------------------|
| `Telemetry`     | DogLog-based logging system; provides `log()`, `print()`, and alert monitoring            |
| `BatteryLogger` | Accumulates per-subsystem current/power/energy each loop and logs totals via `logPower()` |
| `TuneValue`     | SmartDashboard-backed tunable `double` for in-match parameter adjustment                  |

---

### `util`

General-purpose utilities, math, and data structures.

|              Class              |                                        Description                                        |
|---------------------------------|-------------------------------------------------------------------------------------------|
| `CachedDouble`                  | Wraps a `DoubleSupplier` and caches its value once per scheduler iteration                |
| `CanDeviceId`                   | Typed CAN device identifier (device number + bus name)                                    |
| `Conversions`                   | Unit conversion helpers (rotations ↔ inches, RPM ↔ RPS, etc.)                             |
| `CrashTracker`                  | Logs uncaught exceptions to a file on the RIO for post-match debugging                    |
| `Curve` / `ExpCurve`            | Exponential input curve with deadband and scalar for joystick shaping                     |
| `Network`                       | NetworkTables helper utilities                                                            |
| `Trio`                          | Generic three-element tuple                                                               |
| `Util`                          | Miscellaneous utilities; exposes `Util.teleop`, `Util.autoMode`, `Util.disabled` triggers |
| `exceptions/KillRobotException` | Thrown to trigger a controlled robot shutdown on fatal errors                             |

---

### `mechanism`

Abstract base class for all TalonFX-driven mechanisms.

|    Class    |                                                                                                             Description                                                                                                              |
|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `Mechanism` | Manages motor construction, follower configuration, control requests (voltage, velocity, motion magic, torque-FOC), sensor reads, soft limits, current limits, and simulation hooks. All hardware access is gated by `isAttached()`. |

Extend `Mechanism` and call its protected setters from command `execute()` bodies. Inner class `Mechanism.Config` holds all TalonFX configuration and PID/FF gains.

---

### `gamepads`

Xbox controller abstraction.

|   Class   |                                                                                                Description                                                                                                 |
|-----------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `Gamepad` | Abstract class wrapping `CommandXboxController`; provides deadbanded/curved axis reads, bumper/trigger modifier combos, stick direction helpers, alliance-aware cardinals, and a `rumbleCommand()` factory |

---

### `leds`

CTRE CANdle LED subsystem with a pattern library.

|     Class      |                                                                                                                                                                 Description                                                                                                                                                                  |
|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `SpectrumLEDs` | Manages a shared CTRE CANdle with non-overlapping `startIdx`/`numLeds` ranges per instance; provides `solid()`, `blink()`, `breathe()`, `rainbow()`, `scrollingRainbow()`, `chase()`, `bounce()`, `fire()`, `rgbCycle()`, `stripe()`, `gradient()`, `edges()`, `ombre()`, `wave()`, `countdown()`, and `switchCountdown()` pattern factories |

Supports multiple LED zones on a single CANdle and a priority system to prevent low-priority commands from overriding higher-priority ones.

---

### `sim`

Mechanism2d simulation helpers for visualizing robot mechanisms in DriverStation.

|            Class             |                        Description                        |
|------------------------------|-----------------------------------------------------------|
| `ArmSim` / `ArmConfig`       | Simulates a rotating arm                                  |
| `RollerSim` / `RollerConfig` | Simulates a spinning roller/wheel                         |
| `LinearSim` / `LinearConfig` | Simulates a linear extension                              |
| `Mount` / `Mountable`        | Attachment point system for mounting sims onto other sims |
| `Circle`                     | Utility for drawing circular shapes in Mechanism2d        |

---

### `swerve`

Swerve-specific utilities.

|           Class            |                              Description                              |
|----------------------------|-----------------------------------------------------------------------|
| `MapleSimSwerveDrivetrain` | Maple-Sim simulation bridge for CTRE swerve                           |
| `SysID`                    | SysID characterization routine wrapper (translation, rotation, steer) |

---

### `vision`

Limelight vision utilities.

|       Class        |                                                        Description                                                         |
|--------------------|----------------------------------------------------------------------------------------------------------------------------|
| `Limelight`        | Wrapper around `LimelightHelpers` with null-safe MegaTag1/MegaTag2 pose access, tag-count queries, and distance estimation |
| `LimelightHelpers` | Vendored Limelight utility library                                                                                         |
| `VisionLogger`     | Logs vision pose estimates and tag data to telemetry                                                                       |

---

## Dependencies

- [WPILib](https://github.com/wpilibsuite/allwpilib)
- [CTRE Phoenix 6](https://pro.docs.ctr-electronics.com/en/stable/)
- [DogLog](https://github.com/jonahsnider/doglog)
- [PathPlanner](https://github.com/mjansen4857/pathplanner)
- [Maple-Sim](https://github.com/Shenzhen-Robotics-Alliance/Maple-Sim) *(simulation only)*
- [Lombok](https://projectlombok.org/) *(compile-time `@Getter`/`@Setter` generation)*

---

## Usage

Copy the `spectrumLib` directory into your robot project under `src/main/java/frc/`. Extend `SpectrumRobot` as your robot base class and extend `Mechanism` for any TalonFX-driven mechanism.
