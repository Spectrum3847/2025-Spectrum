package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class SwerveStates {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();

    static Command pilotSteerCommand =
            log(pilotDrive().withName("SwerveCommands.pilotSteer").ignoringDisable(true));
    static SpectrumState steeringLock = new SpectrumState("SteeringLock");

    protected static void setupDefaultCommand() {
        swerve.setDefaultCommand(pilotSteerCommand);
    }

    protected static void setStates() {

        pilot.steer.whileTrue(
                swerve.getDefaultCommand()); // Force back to manual steering when we steer

        // When driving and have never steered, it doesn't lock
        // When driving, and we stop steering it locks
        // When not driving it stops locking
        pilot.steer.and(pilot.driving).onTrue(steeringLock.setTrue());
        pilot.driving.onFalse(steeringLock.setFalse());
        steeringLock
                .and(pilot.steer.not())
                .onTrue(log(lockToClosestFieldAngleDrive().withName("Swerve.FieldAngleLock")));

        pilot.fpv_LS.whileTrue(log(fpvDrive()));

        pilot.upReorient.onTrue(log(reorientForward()));
        pilot.leftReorient.onTrue(log(reorientLeft()));
        pilot.downReorient.onTrue(log(reorientBack()));
        pilot.rightReorient.onTrue(log(reorientRight()));

        // // vision aim
        pilot.reefAim_A.whileTrue(log(reefAimDrive()));

        RobotStates.autoAlign.whileTrue(autonSwerveAlign());
        RobotStates.clearOverrideFeedBack.onTrue(clearFeedBack());
    }

    /* Pilot Commands ************************************************************************ */

    // TODO: actually test
    public static Command autonSwerveAlign() {
        return new RunCommand(
                        () -> {
                            // Calculates velocities continuously
                            double tagAngleDegrees = Robot.getVision().getReefTagAngle();
                            Rotation2d tagAngle = Rotation2d.fromDegrees(tagAngleDegrees);

                            // Changes ROBOT RELATIVE velocities to FIELD RELATIVE velocities
                            double fieldXVelocity =
                                    getTagDistanceVelocity() * tagAngle.getCos()
                                            - getTagTxVelocity() * tagAngle.getSin();
                            double fieldYVelocity =
                                    getTagDistanceVelocity() * tagAngle.getSin()
                                            + getTagTxVelocity() * tagAngle.getCos();

                            // Flips feedback if on the red alliance
                            Optional<Alliance> alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()
                                    && alliance.get() == DriverStation.Alliance.Red) {
                                fieldXVelocity = -fieldXVelocity;
                                fieldYVelocity = -fieldYVelocity;
                            }

                            final double finalFieldVelocityX = fieldXVelocity;
                            final double finalFieldVelocityY = fieldYVelocity;

                            // Updates the feedback system continuously
                            PPHolonomicDriveController.overrideXFeedback(() -> finalFieldVelocityX);
                            PPHolonomicDriveController.overrideYFeedback(() -> finalFieldVelocityY);

                            System.out.println(
                                    "!! X Override: "
                                            + finalFieldVelocityX
                                            + " | Y Override: "
                                            + finalFieldVelocityY
                                            + " !!");
                        })
                .withName("Swerve.autonAlign");
    }

    public static Command clearFeedBack() {
        return (new InstantCommand(() -> PPHolonomicDriveController.clearFeedbackOverrides()))
                .withName("Swerve.clearFeedbackOverrides");
    }

    public static Command reefAimDrive() {
        return fpvAimDrive(
                        SwerveStates::getTagDistanceVelocity,
                        SwerveStates::getTagTxVelocity,
                        Robot.getVision()::getReefTagAngle)
                .withName("Swerve.reefAimDrive");
    }

    public static Command alignToXDrive(DoubleSupplier xGoalMeters) {
        return drive(
                () -> getAlignToX(xGoalMeters),
                pilot::getDriveLeftPositive,
                pilot::getDriveCCWPositive);
    }

    public static Command alignToYDrive(DoubleSupplier yGoalMeters) {
        return drive(
                pilot::getDriveFwdPositive,
                () -> getAlignToY(yGoalMeters),
                pilot::getDriveCCWPositive);
    }

    public static Command alignXYDrive(DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters) {
        return drive(
                () -> getAlignToX(xGoalMeters),
                () -> getAlignToY(yGoalMeters),
                pilot::getDriveCCWPositive);
    }

    public static Command alignDrive(
            DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters, DoubleSupplier headingRadians) {
        return drive(
                () -> getAlignToX(xGoalMeters),
                () -> getAlignToY(yGoalMeters),
                () -> getAlignHeading(headingRadians));
    }

    private static double getTagTxVelocity() {
        if (Robot.getVision().tagsInView()) {
            return swerve.calculateTagCenterAlignController(
                    () -> 0, () -> Robot.getVision().getTagTX());
        }
        return 0;
    }

    private static double getTagDistanceVelocity() {
        int tagID = Robot.getVision().getClosestTagID();
        if (tagID < 0) {
            return 0;
        }
        double[][] tagIDAreas = {
            {17, config.getEventTag17TAGoal()},
            {18, config.getEventTag18TAGoal()},
            {19, config.getEventTag19TAGoal()},
            {20, config.getEventTag20TAGoal()},
            {21, config.getEventTag21TAGoal()},
            {22, config.getEventTag22TAGoal()},
            {6, config.getEventTag6TAGoal()},
            {7, config.getEventTag7TAGoal()},
            {8, config.getEventTag8TAGoal()},
            {9, config.getEventTag9TAGoal()},
            {10, config.getEventTag10TAGoal()},
            {11, config.getEventTag11TAGoal()}
        };
        if (tagID >= 17) {
            tagID -= 17;
        }
        if (tagID < 0) {
            tagID = 0;
        }
        if (tagID >= tagIDAreas.length) {
            tagID = 11;
        }

        final int finalTagID = tagID;
        try {
            return swerve.calculateTagDistanceAlignController(() -> tagIDAreas[finalTagID][1]);
        } catch (Exception e) {
            Telemetry.print("Error in getTagDistanceVelocity: " + finalTagID);
            return config.getEventLlAimTAgoal();
        }
        // return swerve.calculateTagDistanceAlignController(() -> tagIDAreas[finalTagID][1]);
    }

    private static double getAlignToX(DoubleSupplier xGoalMeters) {
        return swerve.calculateXController(xGoalMeters);
    }

    private static double getAlignToY(DoubleSupplier yGoalMeters) {
        return swerve.calculateYController(yGoalMeters);
    }

    private static double getAlignHeading(DoubleSupplier headingRadians) {
        return swerve.calculateRotationController(headingRadians);
    }

    protected static Command snapSteerDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
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
                .andThen(fpvDrive(velocityX, velocityY, () -> getAlignHeading(targetRadians)))
                .withName("Swerve.fpvAimDrive");
    }

    /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc.
     */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(drive(velocityX, velocityY, () -> getAlignHeading(targetRadians)))
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
                return getAlignHeading(heading::getAsDouble);
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
