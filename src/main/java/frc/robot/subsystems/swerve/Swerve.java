// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.reefscape.FieldHelpers;
import frc.robot.Robot;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.CanConfigBudget;
import frc.spectrumLib.swerve.MapleSimSwerveDrivetrain;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * CTRE swerve drivetrain with a wanted-state machine driven by {@link
 * frc.robot.subsystems.SuperStructure}. PathPlanner drives it directly in auto.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    public enum WantedState {
        /** Field-centric pilot driving. */
        TELEOP_DRIVE,
        /**
         * Closed-loop drive to {@link #setDriveToPoseGoal}, measuring from the pose the caller
         * supplies. Reef alignment supplies a tag-relative pose; see {@code ReefTagTracker}.
         */
        DRIVE_TO_POSE,
        /** Pilot drives translation; heading is held at {@link #setHeadingGoal}. */
        TELEOP_HEADING_LOCK,
        X_BRAKE,
        /** No request: PathPlanner owns the drive in auto. */
        IDLE
    }

    @Getter private WantedState wantedState = WantedState.IDLE;

    @Getter private final SwerveConfig config;

    private final SwerveRequest.FieldCentric fieldCentric =
            new SwerveRequest.FieldCentric()
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.FieldCentricFacingAngle facingAngle =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.ApplyFieldSpeeds fieldSpeeds =
            new SwerveRequest.ApplyFieldSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.ApplyRobotSpeeds autoRequest =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDesaturateWheelSpeeds(true);
    private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Alert pigeonAlert = new Alert("Pigeon IMU Disconnected", Alert.AlertType.kError);

    private Pose2d driveToPoseGoal = Pose2d.kZero;
    private Supplier<Pose2d> driveToPoseMeasured = this::getRobotPose;
    private double driveToPoseMaxSpeed = 3.0;
    private Rotation2d headingGoal = Rotation2d.kZero;

    public Swerve(SwerveConfig config) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.getDrivetrainConstants(),
                250.0,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(config.getModules()));
        this.config = config;

        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();

        facingAngle.HeadingController.setPID(
                config.getKPRotationController(), 0, config.getKDRotationController());
        facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        this.register();
        if (!CanConfigBudget.exhausted()) {
            optimizeBusUtilization();
        }
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // ── State machine ──────────────────────────────────────────────────────────

    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    /**
     * Sets where {@link WantedState#DRIVE_TO_POSE} drives, and which pose it measures from.
     *
     * @param goal field pose to reach
     * @param measured current pose in the same frame as the goal
     * @param maxSpeed translation cap, m/s
     */
    public void setDriveToPoseGoal(Pose2d goal, Supplier<Pose2d> measured, double maxSpeed) {
        driveToPoseGoal = goal;
        driveToPoseMeasured = measured;
        driveToPoseMaxSpeed = maxSpeed;
    }

    public void setHeadingGoal(Rotation2d heading) {
        headingGoal = heading;
    }

    @Override
    public void periodic() {
        switch (wantedState) {
            case TELEOP_DRIVE -> setControl(
                    fieldCentric
                            .withVelocityX(pilotFieldX())
                            .withVelocityY(pilotFieldY())
                            .withRotationalRate(Robot.getPilot().getDriveCCWPositive()));
            case TELEOP_HEADING_LOCK -> setControl(
                    facingAngle
                            .withVelocityX(pilotFieldX())
                            .withVelocityY(pilotFieldY())
                            .withTargetDirection(headingGoal));
            case DRIVE_TO_POSE -> setControl(fieldSpeeds.withSpeeds(driveToPoseSpeeds()));
            case X_BRAKE -> setControl(xBrake);
            case IDLE -> {
                // PathPlanner's command runs after this and owns the modules in auto.
                if (!DriverStation.isAutonomousEnabled()) {
                    setControl(idle);
                }
            }
        }

        SwerveDriveState state = loopState();
        Telemetry.log("Swerve/WantedState", wantedState.toString());
        Telemetry.log("Swerve/State/Pose", state.Pose);
        Telemetry.log("Swerve/State/TargetStates", state.ModuleTargets);
        Telemetry.log("Swerve/State/MeasuredStates", state.ModuleStates);
        Telemetry.log("Swerve/State/MeasuredSpeeds", state.Speeds);
        if (wantedState == WantedState.DRIVE_TO_POSE) {
            Telemetry.log("Swerve/DriveToPose/Goal", driveToPoseGoal);
        }
        pigeonAlert.set(!getPigeon2().isConnected());
        if (mapleSim != null) {
            Telemetry.log("Sim/TruePose", mapleSim.mapleSimDrive.getSimulatedDriveTrainPose());
        }
    }

    /** Pilot translation in blue-origin field axes; red drives the other way. */
    private double pilotFieldX() {
        double x = Robot.getPilot().getDriveFwdPositive();
        return FieldHelpers.isRed() ? -x : x;
    }

    private double pilotFieldY() {
        double y = Robot.getPilot().getDriveLeftPositive();
        return FieldHelpers.isRed() ? -y : y;
    }

    /**
     * Field-relative speeds toward {@link #driveToPoseGoal}.
     *
     * <p>Translation speed is the least of a proportional term, the speed from which the robot can
     * still stop in the remaining distance, and the cap; it points straight at the goal, so the
     * robot never curves in. A small floor overcomes static friction until inside the deadband.
     */
    private ChassisSpeeds driveToPoseSpeeds() {
        Pose2d measured = driveToPoseMeasured.get();
        Translation2d error = driveToPoseGoal.getTranslation().minus(measured.getTranslation());
        double distance = error.getNorm();
        double speed = 0;
        if (distance > config.getDriveToPoseDeadbandMeters()) {
            speed =
                    Math.min(
                            Math.min(
                                    config.getKPTranslationController() * distance,
                                    Math.sqrt(2 * config.getDriveToPoseMaxAccel() * distance)),
                            driveToPoseMaxSpeed);
            speed = Math.max(speed, config.getDriveToPoseMinSpeed());
        }
        Translation2d velocity =
                distance > 1e-6 ? error.times(speed / distance) : Translation2d.kZero;

        double headingError =
                MathUtil.angleModulus(
                        driveToPoseGoal.getRotation().minus(measured.getRotation()).getRadians());
        double omega =
                Math.abs(headingError) < config.getRotationToleranceRadians()
                        ? 0
                        : MathUtil.clamp(
                                config.getKPRotationController() * headingError,
                                -config.getMaxAngularVelocity(),
                                config.getMaxAngularVelocity());
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
    }

    // ── Per-loop drivetrain state ──────────────────────────────────────────────

    private SwerveDriveState loopState;
    private long loopStateLoop = -1;

    /**
     * The drivetrain state, read once per loop. {@code getState()} is a JNI call plus the odometry
     * lock plus a copy; every caller in a loop now sees the same snapshot. Dropped whenever the
     * pose is changed on purpose, so a correction early in the loop is seen by everything after.
     */
    protected SwerveDriveState loopState() {
        long loop = RobotLoop.count();
        if (loopState == null || loopStateLoop != loop) {
            loopState = getState().clone();
            loopStateLoop = loop;
        }
        return loopState;
    }

    private void invalidateLoopState() {
        loopStateLoop = -1;
    }

    // ── Pose ───────────────────────────────────────────────────────────────────

    public Pose2d getRobotPose() {
        return loopState().Pose;
    }

    /** The estimator's pose at an FPGA timestamp, or the current pose if it is out of range. */
    public Pose2d getPoseAtTimestamp(double fpgaSeconds) {
        Optional<Pose2d> sampled = samplePoseAt(Utils.fpgaToCurrentTime(fpgaSeconds));
        return sampled.orElse(getRobotPose());
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return loopState().Speeds;
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (mapleSim != null) {
            mapleSim.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05);
        }
        super.resetPose(pose);
        invalidateLoopState();
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        super.resetRotation(rotation);
        invalidateLoopState();
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
        super.addVisionMeasurement(visionRobotPose, timestampSeconds, visionStdDevs);
        invalidateLoopState();
    }

    /** True when the front is nearer the given field heading than the back is. */
    public boolean frontClosestToAngle(Rotation2d heading) {
        return Math.abs(getRobotPose().getRotation().minus(heading).getDegrees()) <= 90;
    }

    // ── Reorient ───────────────────────────────────────────────────────────────

    /** Resets the heading so the front faces {@code blueDegrees} (flipped on red). */
    private Command reorient(double blueDegrees, String name) {
        return runOnce(
                        () ->
                                resetRotation(
                                        Rotation2d.fromDegrees(
                                                FieldHelpers.flipAngleIfRed(blueDegrees))))
                .ignoringDisable(true)
                .withName("Swerve." + name);
    }

    public Command reorientForward() {
        return reorient(0, "reorientForward");
    }

    public Command reorientLeft() {
        return reorient(90, "reorientLeft");
    }

    public Command reorientBack() {
        return reorient(180, "reorientBack");
    }

    public Command reorientRight() {
        return reorient(270, "reorientRight");
    }

    // ── PathPlanner ────────────────────────────────────────────────────────────

    private void configurePathPlanner() {
        try {
            AutoBuilder.configure(
                    this::getRobotPose,
                    this::resetPose,
                    this::getCurrentRobotChassisSpeeds,
                    (speeds, feedforwards) ->
                            setControl(
                                    autoRequest
                                            .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                            .withWheelForceFeedforwardsX(
                                                    feedforwards.robotRelativeForcesX())
                                            .withWheelForceFeedforwardsY(
                                                    feedforwards.robotRelativeForcesY())),
                    new PPHolonomicDriveController(
                            new PIDConstants(6, 0, 0), new PIDConstants(8, 0, 0)),
                    RobotConfig.fromGUISettings(),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    // ── Simulation ─────────────────────────────────────────────────────────────

    @Getter private MapleSimSwerveDrivetrain mapleSim = null;

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSim =
                new MapleSimSwerveDrivetrain(
                        Seconds.of(config.getSimLoopPeriod()),
                        Kilograms.of(58),
                        Meters.of(0.927),
                        Meters.of(0.927),
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getKrakenX60Foc(1),
                        1.2,
                        getModuleLocations(),
                        getPigeon2(),
                        getModules(),
                        config.getFrontLeft(),
                        config.getFrontRight(),
                        config.getBackLeft(),
                        config.getBackRight());
        Notifier simNotifier = new Notifier(mapleSim::update);
        simNotifier.startPeriodic(config.getSimLoopPeriod());
    }
}
