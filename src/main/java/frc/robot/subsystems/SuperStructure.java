package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.Field;
import frc.reefscape.Field.ReefHeight;
import frc.reefscape.FieldHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.twist.Twist;
import frc.robot.subsystems.vision.ReefTagTracker;
import frc.robot.subsystems.vision.Vision;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;

/**
 * Maps one wanted robot state onto every mechanism's wanted state each loop.
 *
 * <p>Replaces the 2025 trigger graph (RobotStates plus a *States file per mechanism), where every
 * motion was a command that a one-loop trigger flicker could cancel. Here nothing is scheduled:
 * each loop picks a {@link CurrentSuperState} from the {@link WantedSuperState} and the sensors,
 * then writes every mechanism's target. All setpoints are in one table, {@link Pose}.
 *
 * <p>Not a Subsystem: {@link Robot#robotPeriodic()} runs it once per loop, after vision and before
 * the scheduler, so a decision reaches the mechanisms in the same loop.
 *
 * <p>Sides: coral is scored from the front only. Intakes pick a side (the claw works either way
 * equally), and the net throw is steered by the twist, so it works whichever way the robot faces.
 */
public class SuperStructure {

    // ── States ─────────────────────────────────────────────────────────────────

    public enum WantedSuperState {
        IDLE,
        /** Everything to home and forget any held piece. */
        FORCE_HOME,
        GROUND_CORAL,
        STATION_CORAL,
        GROUND_ALGAE,
        /** Drive to the nearest reef face and pull its algae, high or low as the face has it. */
        REEF_ALGAE,
        /** Arm to the selected coral level while the pilot drives. */
        STAGE_CORAL,
        /** Score at the selected level now. */
        SCORE_CORAL,
        /** Drive to the nearest face, stage, and score when aligned. */
        AUTO_SCORE_CORAL,
        STAGE_NET,
        SCORE_NET,
        STAGE_PROCESSOR,
        SCORE_PROCESSOR,
        EJECT,
        CLIMB_PREP,
        CLIMB
    }

    public enum CurrentSuperState {
        IDLE,
        FORCE_HOME,
        GROUND_CORAL,
        STATION_CORAL,
        GROUND_ALGAE,
        REEF_ALGAE,
        STAGE_CORAL,
        ALIGN_CORAL,
        SCORING_CORAL,
        STAGE_NET,
        SCORING_NET,
        STAGE_PROCESSOR,
        SCORING_PROCESSOR,
        EJECT,
        CLIMB_PREP,
        CLIMB
    }

    public enum Piece {
        NONE,
        CORAL,
        ALGAE
    }

    public enum Branch {
        LEFT,
        RIGHT
    }

    // ── Setpoints ──────────────────────────────────────────────────────────────

    /**
     * One arm configuration, for the front side. The rear side negates the shoulder and elbow and
     * mirrors the twist. Elevator in motor rotations, joints in degrees. Values are the 2025
     * robot's tuned setpoints.
     */
    public record Pose(double elevator, double shoulder, double elbow, double twist) {
        public static final double FULL = 21.1 * 0.999;
        static final double TWIST_LEFT = 90;
        static final double TWIST_RIGHT = 270;

        public static final Pose HOME = new Pose(0, 0, 180, 90);
        public static final Pose STATION = new Pose(0, -9.2, -158.7, 0);
        public static final Pose GROUND_CORAL = new Pose(0, 4, 76, 179.9);
        public static final Pose GROUND_ALGAE = new Pose(4.5, 0, 64, 0);
        public static final Pose L2_ALGAE = new Pose(1, 160, -86, 179.9);
        public static final Pose L3_ALGAE = new Pose(12, 160, -86, 179.9);
        public static final Pose NET = new Pose(FULL, 180, -180, 179.9);
        public static final Pose PROCESSOR = new Pose(0, -143.877, 64.072, 0);
        public static final Pose CLIMB_PREP = new Pose(0, 45, 180, 179.9);

        // Coral: prep pose, then the pose that places it. Twist is set by the branch.
        public static final Pose L1 = new Pose(0.3, 16.9, -130.6, 0);
        public static final Pose L2 = new Pose(6.4, -19.8, -127.1, TWIST_LEFT);
        public static final Pose L2_SCORE = new Pose(4.1, 25, -116, TWIST_LEFT);
        public static final Pose L3 = new Pose(16.9, -19.8, -127.1, TWIST_LEFT);
        public static final Pose L3_SCORE = new Pose(14.6, 30, -106.4, TWIST_LEFT);
        public static final Pose L4 = new Pose(FULL, 193.5, -131.6, TWIST_LEFT);
        public static final Pose L4_SCORE = new Pose(FULL - 3, 145.8, -104, TWIST_LEFT);

        Pose withTwist(double degrees) {
            return new Pose(elevator, shoulder, elbow, degrees);
        }

        Pose rear() {
            return new Pose(elevator, -shoulder, -elbow, Twist.mirrorForRear(twist));
        }

        static Pose prep(ReefHeight level) {
            return switch (level) {
                case L1 -> L1;
                case L2 -> L2;
                case L3 -> L3;
                case L4 -> L4;
            };
        }

        static Pose score(ReefHeight level) {
            return switch (level) {
                case L1 -> L1;
                case L2 -> L2_SCORE;
                case L3 -> L3_SCORE;
                case L4 -> L4_SCORE;
            };
        }
    }

    // ── Timing and gate constants ──────────────────────────────────────────────

    /** Coral score latch: once started it finishes, whatever the alignment does. */
    private static final double CORAL_SCORE_SECONDS = 0.75;

    /** After a score starts, the shoulder waits this long before swinging to the score angle. */
    private static final double SHOULDER_SCORE_DELAY = 0.3;

    /** After a score starts, L2/L3 rollers wait this long before pushing the coral off. */
    private static final double EJECT_DELAY = 0.2;

    private static final double NET_SCORE_SECONDS = 0.5;
    private static final double PROCESSOR_SCORE_SECONDS = 0.5;

    /** An intake held at least this long is assumed to have got its piece, sensor or not. */
    private static final double ASSUME_PIECE_SECONDS = 0.25;

    /**
     * Distance from the reef tag to the robot centre when scoring: half the 29.5 in frame plus the
     * 10 in the 2025 offsets used. Tune per event with {@link #reefStandoffTrim}.
     */
    private static final double REEF_STANDOFF = Units.inchesToMeters(29.5 / 2 + 10);

    private static final double ALGAE_STANDOFF = REEF_STANDOFF;

    /** Raise the arm once the robot is this close to its reef goal. */
    private static final double STAGE_DISTANCE = 1.25;

    /** Score gate, strict: all must hold for {@link #GATE_DEBOUNCE_LOOPS} loops in a row. */
    private static final double GATE_ALONG = Units.inchesToMeters(1.0);

    private static final double GATE_LATERAL = Units.inchesToMeters(1.0);
    private static final double GATE_HEADING_DEG = 1.5;
    private static final int GATE_DEBOUNCE_LOOPS = 3;

    private static final double ALIGN_MAX_SPEED = 3.0;
    /** Slower once the elevator is high, so the robot does not tip into the reef. */
    private static final double ALIGN_MAX_SPEED_HIGH = 1.5;

    /** Per-event trim on the reef standoff, metres (+ further from the reef). */
    @Setter private double reefStandoffTrim = 0;

    // ── Fields ─────────────────────────────────────────────────────────────────

    @Getter private final Swerve swerve;
    @Getter private final Elevator elevator;
    @Getter private final Shoulder shoulder;
    @Getter private final Elbow elbow;
    @Getter private final Twist twist;
    @Getter private final Intake intake;
    @Getter private final Climb climb;
    private final ReefTagTracker tracker;

    @Getter private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    @Getter private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
    private CurrentSuperState previousSuperState = CurrentSuperState.IDLE;
    private double stateStartSeconds = 0;

    /** Held piece; auto sets CORAL for the preload. */
    @Getter @Setter private Piece heldPiece = Piece.NONE;

    @Getter @Setter private ReefHeight selectedLevel = ReefHeight.L4;
    @Getter @Setter private Branch selectedBranch = Branch.LEFT;

    /**
     * Use the mirrored (other side) intake pose for the current intake state; latched on entry.
     *
     * <p>The table's ground poses reach out of the robot's back (+X is the scoring side) and its
     * station pose out of the front, so "mirrored" means the front for ground intakes and the back
     * for the station. See advantagescope-custom-assets/Robot_GammaRay/README.md.
     */
    @Getter private boolean intakeMirrored = false;
    /** Mirrors intakes while set: auto's "reverse" marker, as in the 2025 paths. */
    @Setter private boolean forceMirroredIntake = false;

    private int reefTag = -1;
    private Pose2d reefGoal = Pose2d.kZero;
    private int gateStreak = 0;
    @Getter private boolean scoreGateOpen = false;
    /** Counts completed coral scores, so auto can wait for one. */
    @Getter private int coralScoreCount = 0;

    public SuperStructure(
            Swerve swerve,
            Elevator elevator,
            Shoulder shoulder,
            Elbow elbow,
            Twist twist,
            Intake intake,
            Climb climb,
            ReefTagTracker tracker) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.twist = twist;
        this.intake = intake;
        this.climb = climb;
        this.tracker = tracker;
    }

    public void setWantedSuperState(WantedSuperState state) {
        wantedSuperState = state;
    }

    public Command setStateCommand(WantedSuperState state) {
        return Commands.runOnce(() -> setWantedSuperState(state))
                .ignoringDisable(true)
                .withName("SuperStructure." + state);
    }

    // ── Loop ───────────────────────────────────────────────────────────────────

    public void periodic() {
        currentSuperState = handleStateTransitions();
        if (currentSuperState != previousSuperState) {
            onExit(previousSuperState);
            stateStartSeconds = Timer.getFPGATimestamp();
            onEnter(currentSuperState);
        }
        applyStates();
        previousSuperState = currentSuperState;

        Telemetry.log("SuperStructure/WantedSuperState", wantedSuperState.toString());
        Telemetry.logDash("SuperStructure/CurrentSuperState", currentSuperState.toString());
        Telemetry.logDash("SuperStructure/HeldPiece", heldPiece.toString());
        Telemetry.logDash("SuperStructure/SelectedLevel", selectedLevel.toString());
        Telemetry.logDash("SuperStructure/SelectedBranch", selectedBranch.toString());
        Telemetry.log("SuperStructure/IntakeMirrored", intakeMirrored);
    }

    private double stateSeconds() {
        return Timer.getFPGATimestamp() - stateStartSeconds;
    }

    private CurrentSuperState handleStateTransitions() {
        // Timed scores finish on their own and then hand back to idle.
        switch (currentSuperState) {
            case SCORING_CORAL:
                if (stateSeconds() < CORAL_SCORE_SECONDS) {
                    return CurrentSuperState.SCORING_CORAL;
                }
                wantedSuperState = WantedSuperState.IDLE;
                break;
            case SCORING_NET:
                if (stateSeconds() < NET_SCORE_SECONDS) {
                    return CurrentSuperState.SCORING_NET;
                }
                wantedSuperState = WantedSuperState.IDLE;
                break;
            case SCORING_PROCESSOR:
                if (stateSeconds() < PROCESSOR_SCORE_SECONDS) {
                    return CurrentSuperState.SCORING_PROCESSOR;
                }
                wantedSuperState = WantedSuperState.IDLE;
                break;
            default:
                break;
        }

        return switch (wantedSuperState) {
            case IDLE -> CurrentSuperState.IDLE;
            case FORCE_HOME -> CurrentSuperState.FORCE_HOME;
            case GROUND_CORAL -> CurrentSuperState.GROUND_CORAL;
            case STATION_CORAL -> CurrentSuperState.STATION_CORAL;
            case GROUND_ALGAE -> CurrentSuperState.GROUND_ALGAE;
            case REEF_ALGAE -> CurrentSuperState.REEF_ALGAE;
            case STAGE_CORAL -> CurrentSuperState.STAGE_CORAL;
            case SCORE_CORAL -> CurrentSuperState.SCORING_CORAL;
            case AUTO_SCORE_CORAL -> scoreGateOpen
                    ? CurrentSuperState.SCORING_CORAL
                    : CurrentSuperState.ALIGN_CORAL;
            case STAGE_NET -> CurrentSuperState.STAGE_NET;
            case SCORE_NET -> CurrentSuperState.SCORING_NET;
            case STAGE_PROCESSOR -> CurrentSuperState.STAGE_PROCESSOR;
            case SCORE_PROCESSOR -> CurrentSuperState.SCORING_PROCESSOR;
            case EJECT -> CurrentSuperState.EJECT;
            case CLIMB_PREP -> CurrentSuperState.CLIMB_PREP;
            case CLIMB -> CurrentSuperState.CLIMB;
        };
    }

    private void onEnter(CurrentSuperState state) {
        switch (state) {
            case GROUND_CORAL, GROUND_ALGAE -> intakeMirrored =
                    forceMirroredIntake || drivingForward(intakeMirrored);
            case STATION_CORAL -> intakeMirrored = forceMirroredIntake || backFacesStation();
            case ALIGN_CORAL, REEF_ALGAE -> {
                reefTag = Vision.nearestReefTag();
                tracker.setTargetTag(reefTag);
                gateStreak = 0;
            }
            default -> {}
        }
    }

    private void onExit(CurrentSuperState state) {
        switch (state) {
            case GROUND_CORAL, STATION_CORAL -> takePieceIfIntaken(Piece.CORAL);
            case GROUND_ALGAE, REEF_ALGAE -> takePieceIfIntaken(Piece.ALGAE);
            case SCORING_CORAL -> {
                heldPiece = Piece.NONE;
                coralScoreCount++;
                tracker.setTargetTag(-1);
            }
            case SCORING_NET, SCORING_PROCESSOR, EJECT -> heldPiece = Piece.NONE;
            case ALIGN_CORAL -> {
                scoreGateOpen = false;
                gateStreak = 0;
            }
            default -> {}
        }
        if (state == CurrentSuperState.ALIGN_CORAL || state == CurrentSuperState.REEF_ALGAE) {
            if (currentSuperState != CurrentSuperState.SCORING_CORAL) {
                tracker.setTargetTag(-1);
            }
        }
    }

    private void takePieceIfIntaken(Piece piece) {
        if (intake.hasPiece() || stateSeconds() >= ASSUME_PIECE_SECONDS) {
            heldPiece = piece;
        }
    }

    // ── State application ──────────────────────────────────────────────────────

    private void applyStates() {
        boolean auto = DriverStation.isAutonomous();
        switch (currentSuperState) {
            case IDLE -> {
                driveTeleop();
                moveArm(Pose.HOME, false, auto);
                intake.setWantedState(holdState());
                climb.setWantedState(Climb.WantedState.HOME);
            }
            case FORCE_HOME -> {
                heldPiece = Piece.NONE;
                driveTeleop();
                moveArm(Pose.HOME, false, auto);
                intake.setWantedState(Intake.WantedState.OFF);
                climb.setWantedState(Climb.WantedState.HOME);
            }
            case GROUND_CORAL -> intakeAt(Pose.GROUND_CORAL, Intake.WantedState.GROUND_CORAL);
            case STATION_CORAL -> intakeAt(Pose.STATION, Intake.WantedState.STATION_CORAL);
            case GROUND_ALGAE -> intakeAt(Pose.GROUND_ALGAE, Intake.WantedState.INTAKE_ALGAE);
            case REEF_ALGAE -> reefAlgae();
            case STAGE_CORAL -> {
                driveHeadingLockToReef();
                moveCoralPrep();
                intake.setWantedState(holdState());
            }
            case ALIGN_CORAL -> alignCoral();
            case SCORING_CORAL -> scoreCoral();
            case STAGE_NET -> {
                driveTeleop();
                moveNet();
                intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
            }
            case SCORING_NET -> {
                driveTeleop();
                moveNet();
                intake.setWantedState(Intake.WantedState.SCORE_NET);
            }
            case STAGE_PROCESSOR -> {
                driveTeleop();
                moveArm(Pose.PROCESSOR, false, auto);
                intake.setWantedState(Intake.WantedState.HOLD_ALGAE);
            }
            case SCORING_PROCESSOR -> {
                driveTeleop();
                moveArm(Pose.PROCESSOR, false, auto);
                intake.setWantedState(Intake.WantedState.SCORE_PROCESSOR);
            }
            case EJECT -> {
                driveTeleop();
                intake.setWantedState(Intake.WantedState.EJECT);
            }
            case CLIMB_PREP -> {
                driveTeleop();
                moveArm(Pose.CLIMB_PREP, false, false);
                intake.setWantedState(Intake.WantedState.OFF);
                climb.setWantedState(Climb.WantedState.PREP);
            }
            case CLIMB -> {
                driveTeleop();
                moveArm(Pose.CLIMB_PREP, false, false);
                intake.setWantedState(Intake.WantedState.OFF);
                climb.setWantedState(Climb.WantedState.MANUAL);
            }
        }
    }

    private Intake.WantedState holdState() {
        return switch (heldPiece) {
            case CORAL -> Intake.WantedState.HOLD_CORAL;
            case ALGAE -> Intake.WantedState.HOLD_ALGAE;
            case NONE -> Intake.WantedState.OFF;
        };
    }

    private void intakeAt(Pose pose, Intake.WantedState rollers) {
        driveTeleop();
        moveArm(pose, intakeMirrored, false);
        intake.setWantedState(rollers);
    }

    /**
     * Moves every joint to a pose at once. Auto uses the slow profiles going home, as the 2025
     * autos did, so a fast stow does not throw the robot off its path.
     */
    private void moveArm(Pose front, boolean rear, boolean slow) {
        Pose pose = rear ? front.rear() : front;
        elevator.setWantedState(
                slow ? Elevator.WantedState.SLOW_POSITION : Elevator.WantedState.POSITION,
                pose.elevator());
        shoulder.setWantedState(
                slow ? Shoulder.WantedState.SLOW_POSITION : Shoulder.WantedState.POSITION,
                pose.shoulder());
        elbow.setWantedState(
                front == Pose.GROUND_CORAL
                        ? Elbow.WantedState.GROUND_POSITION
                        : slow ? Elbow.WantedState.SLOW_POSITION : Elbow.WantedState.POSITION,
                pose.elbow());
        twist.setWantedState(Twist.WantedState.POSITION, pose.twist(), rear);
    }

    /**
     * Coral prep for the selected level and branch. For L2 to L4 the elbow stays stowed until the
     * elevator has reached its height, so the claw cannot swing into the robot on the way up.
     */
    private void moveCoralPrep() {
        ReefHeight level = selectedLevel;
        Pose pose = Pose.prep(level);
        if (level != ReefHeight.L1) {
            pose = pose.withTwist(branchTwist());
        }
        boolean slow = DriverStation.isAutonomous() && level == ReefHeight.L4;
        moveArm(pose, false, slow);
        if (level != ReefHeight.L1 && !elevator.isAt(pose.elevator())) {
            elbow.setWantedState(Elbow.WantedState.POSITION, Pose.HOME.elbow());
        }
        if (atReef()) {
            twist.setWantedState(Twist.WantedState.POSITION_AT_REEF, pose.twist(), false);
        }
    }

    private double branchTwist() {
        return selectedBranch == Branch.RIGHT ? Pose.TWIST_RIGHT : Pose.TWIST_LEFT;
    }

    /** True once the arm is up at a branch level, where the twist must route around the pipe. */
    private boolean atReef() {
        return selectedLevel != ReefHeight.L1 && isArmAtTarget();
    }

    public boolean isArmAtTarget() {
        return elevator.isAtTarget()
                && shoulder.isAtTarget()
                && elbow.isAtTarget()
                && twist.isAtTarget();
    }

    private void scoreCoral() {
        holdReefGoalIfTracking();
        ReefHeight level = selectedLevel;
        double t = stateSeconds();
        if (level == ReefHeight.L1) {
            moveArm(Pose.L1, false, false);
            intake.setWantedState(Intake.WantedState.SCORE_L1);
            return;
        }
        Pose score = Pose.score(level).withTwist(branchTwist());
        Pose prep = Pose.prep(level);
        moveArm(score, false, false);
        twist.setWantedState(Twist.WantedState.POSITION_AT_REEF, score.twist(), false);
        if (t < SHOULDER_SCORE_DELAY) {
            shoulder.setWantedState(Shoulder.WantedState.POSITION, prep.shoulder());
        }
        // L4 releases on the arm swing alone; the 2025 code never ran the rollers there.
        boolean eject = level != ReefHeight.L4 && t >= EJECT_DELAY;
        intake.setWantedState(eject ? Intake.WantedState.SCORE_CORAL : holdState());
    }

    private void moveNet() {
        // Point the claw's throw at the net whichever way the robot faces.
        double twistDeg = netTwistDegrees();
        moveArm(Pose.NET.withTwist(twistDeg), false, false);
        if (Math.abs(shoulder.getDegrees() - Pose.NET.shoulder()) > 90) {
            twist.setWantedState(Twist.WantedState.POSITION, Pose.NET.twist(), false);
        }
    }

    /**
     * Twist angle that throws toward the barge. Carried over from the 2025 net turret: the claw
     * points away from our driver station, flipped when the robot is past half field.
     */
    private double netTwistDegrees() {
        if (DriverStation.isAutonomous()) {
            return Pose.NET.twist();
        }
        double offset = 180;
        if (FieldHelpers.isRed()) {
            offset += 180;
        }
        boolean onBlueHalf = swerve.getRobotPose().getX() < Field.fieldLength / 2;
        if (!FieldHelpers.isRed() != onBlueHalf) {
            offset += 180;
        }
        return Twist.mod360(-swerve.getRobotPose().getRotation().getDegrees() + offset);
    }

    // ── Reef alignment and the score gate ──────────────────────────────────────

    /** Where the robot centre goes to score on the tracked tag: facing it, centred on it. */
    private Pose2d reefGoal(double standoff) {
        return Field.tagPose(reefTag)
                .transformBy(new Transform2d(standoff + reefStandoffTrim, 0, Rotation2d.k180deg));
    }

    /** Pose to align with: the tag-relative one when fresh, else the fused field pose. */
    private Pose2d alignMeasuredPose() {
        return tracker.getPose().orElse(swerve.getRobotPose());
    }

    private void alignCoral() {
        reefGoal = reefGoal(REEF_STANDOFF);
        Pose2d measured = alignMeasuredPose();
        double distance = measured.getTranslation().getDistance(reefGoal.getTranslation());
        boolean high = selectedLevel == ReefHeight.L3 || selectedLevel == ReefHeight.L4;
        swerve.setDriveToPoseGoal(
                reefGoal, this::alignMeasuredPose, high ? ALIGN_MAX_SPEED_HIGH : ALIGN_MAX_SPEED);
        swerve.setWantedState(Swerve.WantedState.DRIVE_TO_POSE);

        if (distance < STAGE_DISTANCE) {
            moveCoralPrep();
        } else {
            moveArm(Pose.HOME, false, false);
        }
        intake.setWantedState(holdState());
        updateScoreGate(measured);
    }

    /**
     * Opens the gate after the tag-relative pose and every joint have been on target for a few
     * loops in a row. The score it starts is latched ({@link #CORAL_SCORE_SECONDS}), so a bump
     * against the reef once scoring cannot strand the coral. The fused field pose never opens it:
     * only the pose measured against the target tag is good enough to score from.
     */
    private void updateScoreGate(Pose2d measured) {
        Optional<Pose2d> tagPose = tracker.getPose();
        Transform2d error = new Transform2d(reefGoal, measured);
        boolean poseReady =
                tagPose.isPresent()
                        && Math.abs(error.getX()) < GATE_ALONG
                        && Math.abs(error.getY()) < GATE_LATERAL
                        && Math.abs(error.getRotation().getDegrees()) < GATE_HEADING_DEG;
        boolean armReady = isArmAtTarget();
        gateStreak = poseReady && armReady ? gateStreak + 1 : 0;
        scoreGateOpen = gateStreak >= GATE_DEBOUNCE_LOOPS;

        Telemetry.log("SuperStructure/ScoreGate/TagPoseFresh", tagPose.isPresent());
        Telemetry.log("SuperStructure/ScoreGate/AlongErrorIn", Units.metersToInches(error.getX()));
        Telemetry.log(
                "SuperStructure/ScoreGate/LateralErrorIn", Units.metersToInches(error.getY()));
        Telemetry.log("SuperStructure/ScoreGate/HeadingErrorDeg", error.getRotation().getDegrees());
        Telemetry.log("SuperStructure/ScoreGate/PoseReady", poseReady);
        Telemetry.log("SuperStructure/ScoreGate/ArmReady", armReady);
        Telemetry.logDash("SuperStructure/ScoreGate/Open", scoreGateOpen);
        Telemetry.log("SuperStructure/ReefGoal", reefGoal);
    }

    /** During a latched score keep holding the reef goal, if the score came from an alignment. */
    private void holdReefGoalIfTracking() {
        if (tracker.getTargetTag() >= 0 && previousWasAlign()) {
            swerve.setWantedState(Swerve.WantedState.DRIVE_TO_POSE);
        } else {
            driveTeleop();
        }
    }

    private boolean previousWasAlign() {
        return wantedSuperState == WantedSuperState.AUTO_SCORE_CORAL;
    }

    private void reefAlgae() {
        boolean high = Field.isHighAlgae(reefTag);
        reefGoal = reefGoal(ALGAE_STANDOFF);
        swerve.setDriveToPoseGoal(reefGoal, this::alignMeasuredPose, ALIGN_MAX_SPEED);
        swerve.setWantedState(Swerve.WantedState.DRIVE_TO_POSE);
        moveArm(high ? Pose.L3_ALGAE : Pose.L2_ALGAE, false, false);
        intake.setWantedState(Intake.WantedState.INTAKE_ALGAE);
    }

    // ── Drive helpers ──────────────────────────────────────────────────────────

    private void driveTeleop() {
        swerve.setWantedState(
                DriverStation.isAutonomous()
                        ? Swerve.WantedState.IDLE
                        : Swerve.WantedState.TELEOP_DRIVE);
    }

    /** Pilot drives; the heading holds the front square to the nearest reef face. */
    private void driveHeadingLockToReef() {
        if (DriverStation.isAutonomous()) {
            swerve.setWantedState(Swerve.WantedState.IDLE);
            return;
        }
        int tag = Vision.nearestReefTag();
        swerve.setHeadingGoal(Field.tagPose(tag).getRotation().plus(Rotation2d.k180deg));
        swerve.setWantedState(Swerve.WantedState.TELEOP_HEADING_LOCK);
    }

    /**
     * True when the robot is being driven forward, in robot axes: the pilot's command if there is
     * one, else the measured speed. Nearly still keeps {@code otherwise}.
     */
    private boolean drivingForward(boolean otherwise) {
        Translation2d field =
                new Translation2d(
                        Robot.getPilot().getDriveFwdPositive(),
                        Robot.getPilot().getDriveLeftPositive());
        if (FieldHelpers.isRed()) {
            field = field.unaryMinus();
        }
        double forward =
                field.getNorm() > 0.3
                        ? field.rotateBy(swerve.getRobotPose().getRotation().unaryMinus()).getX()
                        : swerve.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
        if (Math.abs(forward) < 0.3) {
            return otherwise;
        }
        return forward > 0;
    }

    /** True when the back of the robot is the side nearer to facing the nearest coral station. */
    private boolean backFacesStation() {
        Pose2d robot = swerve.getRobotPose();
        Pose2d left = flipIfRed(Field.leftCoralStation);
        Pose2d right = flipIfRed(Field.rightCoralStation);
        Pose2d station =
                robot.getTranslation().getDistance(left.getTranslation())
                                < robot.getTranslation().getDistance(right.getTranslation())
                        ? left
                        : right;
        // The station face points into the field; the side that faces the station points back.
        Rotation2d faceStation = station.getRotation().plus(Rotation2d.k180deg);
        return Math.abs(MathUtil.angleModulus(robot.getRotation().minus(faceStation).getRadians()))
                > Math.PI / 2;
    }

    private static Pose2d flipIfRed(Pose2d blue) {
        if (!FieldHelpers.isRed()) {
            return blue;
        }
        return new Pose2d(
                Field.fieldLength - blue.getX(),
                Field.fieldWidth - blue.getY(),
                blue.getRotation().plus(Rotation2d.k180deg));
    }
}
