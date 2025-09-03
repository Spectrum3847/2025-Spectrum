package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field;
import frc.reefscape.FieldHelpers;
import frc.reefscape.TagProperties;
import frc.reefscape.Zones;
import frc.reefscape.offsets.HomeOffsets;
import frc.robot.Robot;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class SwerveStates {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();
    static Zones zones = new Zones();

    static Command pilotSteerCommand =
            log(pilotDrive().withName("SwerveCommands.pilotSteer").ignoringDisable(true));
    static SpectrumState steeringLock = new SpectrumState("SteeringLock");

    public static final Trigger isFrontClosestToLeftStation =
            new Trigger(
                    () ->
                            swerve.frontClosestToAngle(
                                    FieldHelpers.flipAngleIfRed(
                                            Field.CoralStation.leftFaceRobotPovDegrees)));
    public static final Trigger isFrontClosestToRightStation =
            new Trigger(
                    () ->
                            swerve.frontClosestToAngle(
                                    FieldHelpers.flipAngleIfRed(
                                            Field.CoralStation.rightFaceRobotPovDegrees)));

    public static final Trigger isFrontClosestToNet =
            new Trigger(
                    () ->
                            swerve.frontClosestToAngle(Field.Barge.netRobotPovDegrees)
                                    == Zones.blueFieldSide.getAsBoolean());

    public static final Trigger isAlignedToReef = new Trigger(() -> Zones.atReef());

    protected static void setupDefaultCommand() {
        swerve.setDefaultCommand(pilotSteerCommand);
    }

    protected static void setStates() {

        pilot.steer.whileTrue(
                swerve.getDefaultCommand()); // Force back to manual steering when we steer

        // When driving and have never steered, it doesn't lock
        // When driving, and we stop steering it locks
        // When not driving it stops locking
        // pilot.steer.and(pilot.driving).onTrue(steeringLock.setTrue());
        // pilot.driving.onFalse(steeringLock.setFalse());
        // steeringLock
        //         .and(pilot.steer.not())
        //         .onTrue(log(lockToClosestFieldAngleDrive().withName("Swerve.FieldAngleLock")));

        pilot.fpv_LS.whileTrue(log(fpvDrive()));

        pilot.upReorient.onTrue(log(reorientForward()));
        pilot.leftReorient.onTrue(log(reorientLeft()));
        pilot.downReorient.onTrue(log(reorientBack()));
        pilot.rightReorient.onTrue(log(reorientRight()));

        // // vision aim
        // pilot.reefAim_A.whileTrue(log(reefAimDrive()));
        // pilot.reefVision_A.whileTrue(log(reefAimDriveVisionTA()));
        pilot.reefVision_A.whileTrue(log(reefAimDriveVisionXY()));
        pilot.reefAlignScore_B.whileTrue(log(reefAimDriveVisionXY()));

        // Pose2d backReefOffset = Field.Reef.getOffsetPosition(21, Units.inchesToMeters(24));
        // pilot.cageAim_B.whileTrue(
        //         alignDrive(
        //                 backReefOffset::getX,
        //                 backReefOffset::getY,
        //                 () -> Math.toRadians(180))); // alignToYDrive(() -> Field.fieldWidth /
        // 2));
    }

    /** Pilot Commands ************************************************************************ */
    /** Drive the robot using left stick and control orientation using the right stick. */
    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
    }

    public static Command autonAlgaeDriveIntake(double timeout) {
        return fpvAimDrive(() -> 0.75, () -> 0, FieldHelpers::getReefTagAngle).withTimeout(timeout);
    }

    public static Command reefAimDriveVisionTA() {
        return fpvAimDrive(
                        SwerveStates::getTagDistanceVelocity,
                        SwerveStates::getTagTxVelocity,
                        () -> FieldHelpers.getReefTagAngle())
                .withName("Swerve.reefAimDriveVisionTA");
    }

    public static Command autonAlgaeReefAimDriveVisionXY() {
        return alignDrive(
                        () -> FieldHelpers.getReefOffsetFromTagX() + 0.05,
                        () -> FieldHelpers.getReefOffsetFromTagY(),
                        () -> FieldHelpers.getReefTagAngle())
                .withName("Swerve.reefAimDriveVisionXY");
    }

    public static Command reefAimDriveVisionXY() {
        return alignDrive(
                        () -> FieldHelpers.getReefOffsetFromTagX(),
                        () -> FieldHelpers.getReefOffsetFromTagY(),
                        () -> FieldHelpers.getReefTagAngle())
                .withName("Swerve.reefAimDriveVisionXY");
    }

    public static Command reefAimDrive() {
        return alignDrive(
                        () -> zones.getScoreReefPoseX(),
                        () -> zones.getScoreReefPoseY(),
                        () -> zones.getScoreReefPoseAngle())
                .withName("Swerve.reefAimDrive");
    }

    public static Command alignToXDrive(DoubleSupplier xGoalMeters) {
        return resetXController()
                .andThen(
                        drive(
                                getAlignToX(xGoalMeters),
                                pilot::getDriveLeftPositive,
                                pilot::getDriveCCWPositive));
    }

    public static Command alignToYDrive(DoubleSupplier yGoalMeters) {
        return resetYController()
                .andThen(
                        drive(
                                pilot::getDriveFwdPositive,
                                getAlignToY(yGoalMeters),
                                pilot::getDriveCCWPositive));
    }

    public static Command alignXYDrive(DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters) {
        return resetXController()
                .alongWith(resetYController())
                .andThen(
                        drive(
                                getAlignToX(xGoalMeters),
                                getAlignToY(yGoalMeters),
                                pilot::getDriveCCWPositive));
    }

    public static Command alignDrive(
            DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters, DoubleSupplier headingRadians) {
        if (Field.isRed()) {
            return resetXController()
                    .andThen(
                            resetYController(),
                            resetTurnController(),
                            drive(
                                    () -> -getAlignToX(xGoalMeters).getAsDouble(),
                                    () -> -getAlignToY(yGoalMeters).getAsDouble(),
                                    () -> getAlignHeading(headingRadians).getAsDouble()));
        }

        return resetXController()
                .andThen(
                        resetYController(),
                        resetTurnController(),
                        drive(
                                getAlignToX(xGoalMeters),
                                getAlignToY(yGoalMeters),
                                getAlignHeading(headingRadians)));
    }

    private static double getTagTxVelocity() {
        if (Robot.getVision().tagsInView()) {
            return swerve.calculateTagCenterAlignController(
                    () -> 0, () -> Robot.getVision().getTagTX());
        }
        return 0;
    }

    private static double getTagDistanceVelocity() {
        TagProperties[] tagAreaOffsets = HomeOffsets.getReefTagOffsets();
        int tagIndex = Robot.getVision().getClosestTagID();
        if (tagIndex < 0) {
            return 0.0;
        } else if (tagIndex >= 17) {
            tagIndex -= 17;
        }

        final double tagAreaOffset = tagAreaOffsets[tagIndex].getTaGoal();

        // System.out.println("Tag Area Offset: " + tagAreaOffset);
        SmartDashboard.putNumber("Tag Area Offset: ", tagAreaOffset);
        return swerve.calculateTagDistanceAlignController(() -> tagAreaOffset);
    }

    private static DoubleSupplier getAlignToX(DoubleSupplier xGoalMeters) {
        return swerve.calculateXController(xGoalMeters);
    }

    private static DoubleSupplier getAlignToY(DoubleSupplier yGoalMeters) {
        return swerve.calculateYController(yGoalMeters);
    }

    private static DoubleSupplier getAlignHeading(DoubleSupplier headingRadians) {
        return () -> swerve.calculateRotationController(headingRadians);
    }

    protected static Command snapSteerDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    protected static Command fpvDrive() {
        return fpvDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotFPVDrive");
    }

    protected static Command pilotAimDrive(DoubleSupplier targetDegrees) {
        return aimDrive(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive, targetDegrees)
                .withName("Swerve.PilotAimDrive");
    }

    protected static Command snakeDrive() {
        return aimDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getPilotStickAngle)
                .withName("Swerve.SnakeDrive");
    }

    protected static Command headingLockDrive() {
        return headingLock(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    protected static Command lockToClosest45degDrive() {
        return lockToClosest45deg(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockTo45degDrive");
    }

    protected static Command lockToClosestFieldAngleDrive() {
        return lockToClosestFieldAngle(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockToFieldAngleDrive");
    }

    /** Turn the swerve wheels to an X to prevent the robot from moving */
    protected static Command xBrake() {
        return swerve.applyRequest(SwerveRequest.SwerveDriveBrake::new).withName("Swerve.Xbrake");
    }

    /**
     * ************************************************************************* Helper Commands
     * ************************************************************************
     */
    protected static Command resetXController() {
        return swerve.runOnce(() -> swerve.resetXController()).withName("ResetXController");
    }

    protected static Command resetYController() {
        return swerve.runOnce(() -> swerve.resetYController()).withName("ResetYController");
    }

    protected static Command resetTurnController() {
        return swerve.runOnce(() -> swerve.resetRotationController())
                .withName("ResetTurnController");
    }

    protected static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.runOnce(() -> config.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    private static final SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Uses m/s and rad/s
    private static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                fieldCentricDrive
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.drive");
    }

    private static final SwerveRequest.RobotCentric robotCentric =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static Command fpvDrive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                robotCentric
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.fpvDrive");
    }

    protected static Command fpvAimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(fpvDrive(velocityX, velocityY, getAlignHeading(targetRadians)))
                .withName("Swerve.fpvAimDrive");
    }

    /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc.
     */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(drive(velocityX, velocityY, getAlignHeading(targetRadians)))
                .withName("Swerve.aimDrive");
    }

    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engage if you are driving x or y.
     */
    protected static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getRotation().getRadians()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> config.getTargetHeading())))
                .withName("Swerve.HeadingLock");
    }

    protected static Command lockToClosest45deg(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getClosest45()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> swerve.getClosest45())))
                .withName("Swerve.LockTo45deg");
    }

    protected static Command lockToClosestFieldAngle(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getClosestFieldAngle()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> swerve.getClosestFieldAngle())))
                .withName("Swerve.LockToFieldAngle");
    }

    private static DoubleSupplier rotateToHeadingWhenMoving(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier heading) {
        return () -> {
            if (Math.abs(velocityX.getAsDouble()) < 0.5
                    && Math.abs(velocityY.getAsDouble()) < 0.5) {
                return 0;
            } else {
                return getAlignHeading(heading::getAsDouble).getAsDouble();
            }
        };
    }

    /**
     * *******************************************************************************************
     * Reorient Commands
     */
    protected static Command reorientForward() {
        return swerve.reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    protected static Command reorientLeft() {
        return swerve.reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    protected static Command reorientBack() {
        return swerve.reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    protected static Command reorientRight() {
        return swerve.reorientPilotAngle(270).withName("Swerve.reorientRight");
    }

    protected static Command cardinalReorient() {
        return swerve.cardinalReorient().withName("Swerve.cardinalReorient");
    }

    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
