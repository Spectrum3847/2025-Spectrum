# Gamma Ray (3847, 2025) - AdvantageScope model

Generated from `0. Gamma Ray.step`. All GLBs are already in the **robot frame at the HOME pose** (see below), meters,
+X forward, +Y left, +Z up, origin = robot center on the floor. So `config.json` has no rotations and every
component has zero `zeroedRotations`/`zeroedPosition`.

|    File     |                                           Contents                                            |
|-------------|-----------------------------------------------------------------------------------------------|
| model.glb   | drivetrain, static elevator stage, climber (static; climber arm not articulated), electronics |
| model_0.glb | elevator inner stage = carriage (single moving stage) + shoulder gearbox/plates/sprockets     |
| model_1.glb | shoulder arm ("Bicep")                                                                        |
| model_2.glb | elbow arm ("Forearm": elbow tube, twist gearbox, support plates)                              |
| model_3.glb | twist shaft + claw/intake ("Twist" + "Claw")                                                  |

Not in the CAD: bumpers. Fasteners were dropped; meshes decimated.

## Joints (robot frame, GLB zero pose = code HOME)

The STEP had the elevator at full extension (~0.78 m up); the component GLBs were shifted down 0.775 m so the
zero pose is HOME: elevator 0, shoulder 0 (bicep hanging down), elbow 180 (forearm straight up), twist 90.

|  Joint   | Point on axis (m)  |                Axis                 |
|----------|--------------------|-------------------------------------|
| Elevator | -                  | +Z translation, 0.775 m at 21.1 rot |
| Shoulder | (0, -0.200, 0.641) | Y                                   |
| Elbow    | (0, -0.200, 0.387) | Y                                   |
| Twist    | (0, 0, 0.387)      | forearm Z                           |

The elbow is chain-driven from a motor on the carriage (via a 1:1 sprocket on the shoulder axis),
so elbowDeg is ABSOLUTE relative to the carriage, not relative to the bicep.

## Computing component poses (Pose3d[4], checked against the tuned 2025 setpoints)

```java
static Transform3d pivot(Translation3d p, Rotation3d r) {
  return new Transform3d(p, r).plus(new Transform3d(p.unaryMinus(), new Rotation3d()));
}
static final Translation3d SHOULDER = new Translation3d(0, -0.200, 0.641);
static final Translation3d ELBOW    = new Translation3d(0, -0.200, 0.387);
static final Translation3d TWIST    = new Translation3d(0, 0, 0.387);
static final double TRAVEL = 0.775; // m at 21.1 rotations

double h = elevatorRotations / 21.1 * TRAVEL;
double a = -Math.toRadians(shoulderDeg);        // SHOULDER_SIGN = -1
double b =  Math.toRadians(elbowDeg - 180);     // ELBOW_SIGN = +1, absolute forearm angle vs carriage
double t =  Math.toRadians(twistDeg - 90);      // TWIST_SIGN = +1 (unverified)
Pose3d carriage = new Pose3d(0, 0, h, new Rotation3d());
Pose3d bicep   = carriage.transformBy(pivot(SHOULDER, new Rotation3d(0, a, 0)));
Pose3d forearm = bicep.transformBy(pivot(ELBOW, new Rotation3d(0, b - a, 0)));  // absolute elbow: subtract shoulder
Pose3d claw    = forearm.transformBy(pivot(TWIST, new Rotation3d(0, 0, t)));
Logger.recordOutput("Mechanism/Components", new Pose3d[] {carriage, bicep, forearm, claw});
```

With this convention reef scoring (L2-L4, reef algae) is in front (+X) at the expected heights, NET is at ~2.26 m,
and the ground intakes (coral, algae) reach near the floor BEHIND the robot. No sign choice puts both ground
intake and reef scoring in front, so the code itself puts them on opposite sides.

## Uncertain

- Front direction: CAD -Y was taken as robot front (the CAD includes a Reef placed on that side, which the arm faces).
  If backwards, add `{"axis":"z","degrees":180}` to the top-level `rotations` - but then component poses must also be
  rotated, so better to regenerate.
- Twist sign and twist zero (90) are unverified: the claw is roughly symmetric about the twist axis, so position checks cannot pin them down.
- Back camera rotation order and camera FOV/resolution are guesses; positions match the code (and the CAD LL mounts within ~1 cm).
- Elevator belts are in model_0 and will ride up with the carriage.
