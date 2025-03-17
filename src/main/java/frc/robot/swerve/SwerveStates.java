package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class SwerveStates {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();

    static Command pilotSteerCommand =
            log(pilotDrive().withName("SwerveCommands.pilotSteer").ignoringDisable(true));
    static SpectrumState steeringLock = new SpectrumState("SteeringLock");

    protected static void setupDefaultCommand() {
        // TODO: change this back
        /*
        if (Rio.id == Rio.PHOTON_2024) {
            // Use this to set a different command based on robotType
            // Robot.swerve.setDefaultCommand(PhotonPilotCommands.pilotDrive());
            // return;
            // swerve.setDefaultCommand(pilotSteerCommand);
        } else {
            swerve.setDefaultCommand(pilotSteerCommand);
        }
        */
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
        pilot.visionAim_A.whileTrue(log(reefAimDrive()));

        RobotStates.autoAlign.onTrue(autonSwerveAlign());
        RobotStates.clearOverrideFeedBack.onTrue(clearFeedBack());
    }

    /* Pilot Commands ************************************************************************ */

    public static Command autonSwerveAlign() {
        return (new InstantCommand(
                        () -> {
                                PPHolonomicDriveController.overrideXFeedback(SwerveStates::getTagDistanceVelocity);
                                PPHolonomicDriveController.overrideYFeedback(SwerveStates::getTagTxVelocity);
                                }
                        ))
                .withName("autonAlign");
    }

    public static Command clearFeedBack() {
        return new InstantCommand(() -> PPHolonomicDriveController.clearOverrideFeedBack());
    }

    public static Command reefAimDrive() {
        return fpvAimDrive(
                        SwerveStates::getTagDistanceVelocity,
                        SwerveStates::getTagTxVelocity,
                        Robot.getVision()::getReefTagAngle)
                .withName("Swerve.reefAimDrive");
    }

    private static double getTagTxVelocity() {
        if (Robot.getVision().tagsInView()) {
            return swerve.calculateTagCenterAlignController(
                    () -> 0, () -> Robot.getVision().getTagTX());
        }
        return 0;
    }

    private static double getTagDistanceVelocity() {
        return swerve.calculateTagDistanceAlignController(() -> config.getHomeLlAimTAgoal());
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

    // TODO: Snake Drive, where the robot moves in the direction of the left stick, but the
    // orientation is controlled by the direction the left stick is pointing, so intake is always
    // pointing where the robot is moving

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
                .andThen(
                        fpvDrive(
                                velocityX,
                                velocityY,
                                () -> swerve.calculateRotationController(targetRadians)))
                .withName("Swerve.fpvAimDrive");
    }

    /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc.
     */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(
                        drive(
                                velocityX,
                                velocityY,
                                () -> swerve.calculateRotationController(targetRadians)))
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
                return swerve.calculateRotationController(heading::getAsDouble);
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
