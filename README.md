# Gamma Ray (3847, REEFSCAPE 2025) on the 2026 stack

The 2025 robot's code, moved to WPILib 2026 and the architecture from the 2026-Spectrum
`2026-offseason-bot` branch. It still runs on the roboRIO. Team number 3847.

## What changed from the 2025 code

- **One state machine.** `subsystems/SuperStructure.java` replaces `RobotStates` and the
  per-mechanism `*States` files. Buttons set a `WantedSuperState`; each loop the superstructure
  picks a `CurrentSuperState` and writes every mechanism's target. Nothing is scheduled per
  motion, so a one-loop trigger flicker can no longer cancel a move. All setpoints are in one
  table, `SuperStructure.Pose`, copied from the tuned 2025 values.
- **Fixed loop order** in `Robot.robotPeriodic()`: vision, then the superstructure, then the
  scheduler (mechanism outputs). Bindings are made once, not rebuilt on every mode change.
- **Vision** (`subsystems/vision/Vision.java`), ported from 2026 for the two Limelight 4s:
  - While disabled, MegaTag1 seeds the pose and heading. The seed only counts as confirmed after a
    second of steady multi-tag frames.
  - While enabled, only translation is fused, and only MegaTag2 once the seed is confirmed. The
    2025 code fused MegaTag1 heading every teleop loop and had an `xyStds = 0` case.
  - Frames are rejected for:
    - spinning, checked over the last 0.3 s
    - a solve that puts the robot off the floor or tilted
    - being out of the field or too far away
  - Safety nets: seeding while enabled, confirming or refuting the auto start pose, and fixing a
    gross heading error.
  - From 2026-FM-Systemcore issue #10:
    - 6328's std-dev model
    - trusting tags less near the image edge
    - fusing both cameras' frames in capture order
    - rejection counters (`Vision/Rejections/*`)
    - Limelight Rewind capture at the end of FMS matches
    - camera-disconnected alerts
- **Reef alignment against one tag** (`subsystems/vision/ReefTagTracker.java`, issue #10 item
  8c). For the final approach the robot measures itself against the target reef tag alone:
  - heading from the gyro
  - direction to the tag from its tx/ty
  - range from the Limelight's distance to the tag

  There is no PnP ambiguity, and map errors cancel because the goal pose is built from the same
  tag. The estimate is carried forward by odometry through short dropouts. Either camera can
  provide it.

- **Score gate.**

  - It opens only when all of these hold for 3 loops in a row:
    - the tag-relative pose is within 1 in along, 1 in lateral and 1.5° of the goal
    - every joint is at its target
  - The score it starts is latched for 0.75 s, so bumping the reef can't strand the coral.
  - The fused field pose never opens the gate. That was the 2025 failure: MegaTag1 heading jitter
    moved the pose in and out of a 0.7 in / 0.35° window.
- **Sides.** Coral is scored from the front only (the Limelight on the front is the scoring
  camera). The ground intakes pick a side from the drive direction, and the station intake from
  which end faces the station. The net throw is aimed by the twist, so it works either way round.

## Controls

**Pilot** (port 0)

|          Input           |                                  Action                                   |
|--------------------------|---------------------------------------------------------------------------|
| LT (hold)                | Ground coral, side chosen from the drive direction                        |
| LB (hold)                | Station coral, side chosen from which end faces the station               |
| RT (hold)                | Auto-align to the nearest reef face and score at the operator's selection |
| RB (hold / release)      | Stage the selection / score it (manual)                                   |
| A (hold)                 | Pull algae from the nearest reef face (high or low is automatic)          |
| X (hold)                 | Ground algae                                                              |
| B (hold / release)       | Stage the net / throw                                                     |
| Y (hold / release)       | Stage the processor / score                                               |
| Select                   | Home everything                                                           |
| D-pad                    | Reorient: the front now faces away from you, left, toward you, or right   |
| Disabled: A / B / Select | Coast / brake / reset pose from vision                                    |

**Operator** (port 1)

|       Input        |                                  Action                                   |
|--------------------|---------------------------------------------------------------------------|
| A / B / X / Y      | Select L1 / L2 / L3 / L4 (latched; shown on the dashboard)                |
| D-pad left / right | Select the left / right branch                                            |
| LT (hold)          | Eject                                                                     |
| Start (hold)       | Climb prep. While holding Start, RT winds the climb in and LT lets it out |
| Select             | Home everything                                                           |

## Simulation

```bash
./gradlew simulateJava
```

Connect AdvantageScope to `localhost` (NT4) and set up these views:

- **3D field (2025):**
  - Robot model "Gamma Ray" from `advantagescope-custom-assets/Robot_GammaRay` (point
    AdvantageScope's custom assets folder there). Robot pose `/Robot/Swerve/State/Pose`,
    components `/Robot/Sim/Components`.
  - Coral in the claw: `/Robot/Sim/HeldCoral`. Algae in the claw: `/Robot/Sim/HeldAlgae`.
  - Scored coral: `/Robot/Sim/ScoredCoral`.
  - Alignment goal (as a ghost): `/Robot/SuperStructure/ReefGoal`.
- **Graphs for alignment:** `/Robot/SuperStructure/ScoreGate/*` and `/Robot/Vision/ReefTag/*`.

The sim generates Limelight tag sightings from the simulator's true pose, with noise and latency
(`SimTagCamera`), so alignment runs just as it does on the robot.

**Regression run:** `SIM_SCENARIO=1 ./gradlew simulateJava`. It does three teleop auto-scores from
offset, angled starts, a ground intake and the 3-coral auto, then prints `SCENARIO PASS/FAIL`
lines and exits with the failure count. Current result: all pass. Each auto-score takes 2.0–2.4 s
from 1.2–1.8 m out and ends within about 0.7 in and 0.6° of the goal.

## Bring-up checklist (things the sim can't tell us)

1. **Camera mounts:** `Vision.VisionConfig`. Measure on the robot. Code pushes them to the cameras
   every 2 s, so the Limelight web UI values get overwritten. A pitch that's off by a degree shows
   up as range error at every tag.
2. **Field layout:** `Field.TAG_LAYOUT` is set to AndyMark. It must match the event and the
   Limelights.
3. **Reef standoff:** `SuperStructure.REEF_STANDOFF` is 29.5/2 + 10 in, from the 2025 offsets. Use
   `setReefStandoffTrim` for per-event trims.
4. **Gate tolerances:** the `GATE_*` constants in `SuperStructure`. Check
   `ScoreGate/*ErrorIn` in logs from real scores.
5. **Gyro heading matters for alignment.** A heading error of θ becomes about 0.6·sin(θ) m of
   lateral error at the reef. The disabled seed confirmation is what keeps θ small, so make sure
   `Vision/PoseSeedConfirmed` turns true before each match.
6. **Front/back:** the CAD-derived model takes CAD −Y as the front, and the 2025 setpoints put
   ground intake out the back. Confirm on the robot that the front Limelight faces the side that
   scores coral.
7. **Twist sign in the 3-D model:** unverified. If the claw rolls the wrong way in AdvantageScope,
   flip `t` in `RobotSim`.

## Structure

- `frc.robot.subsystems.*`: mechanisms (wanted-state machines), `SuperStructure`, `vision`,
  `swerve`, `leds`
- `frc.robot.auton.Auton`: auto routines. The 2025 PathPlanner event markers map onto
  superstructure requests.
- `frc.reefscape`: field constants and tag helpers
- `frc.spectrumLib`: the shared library, copied from 2026-Spectrum
