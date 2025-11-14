package frc.robot.intake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.intake.Intake.IntakeConfig;
import frc.spectrumLib.Telemetry;

public class IntakeStates {
    private static Intake intake = Robot.getIntake();
    private static IntakeConfig config = Robot.getConfig().intake;

    public static final Trigger hasGamePiece = new Trigger(intake::hasIntakeGamePiece);
    public static final Trigger hasCoral =
            hasGamePiece.and(intake.aboveVelocityRPM(() -> 0, () -> 0));
    public static final Trigger hasAlgae =
            netAlgae.not().and(intake.aboveCurrent(config::getHasAlgaeCurrent, () -> 0));
    // hasGamePiece.and(intake.belowVelocityRPM(() -> 0, () -> 0));

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(
                intake.stopMotor().ignoringDisable(true).withName("Intake.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(intake.runVoltage(() -> 0).withName("Intake.neutral"));
    }

    public static void intakeCoral() {
        scheduleIfNotRunning(
                intake.runTorqueFOC(config::getCoralGroundTorqueCurrent)
                        .withName("Intake.intakeCoral"));
    }

    public static void intakeAlgae() {
        scheduleIfNotRunning(
                intake.runTorqueFOC(config::getAlgaeIntakeTorqueCurrent)
                        .withName("Intake.intakeAlgae"));
    }

    public static void holdCoral() {
        scheduleIfNotRunning(
                intake.runTorqueFOC(config::getCoralHoldTorqueCurrent)
                        .withName("Intake.holdCoral"));
    }

    public static void l1Score() {
        scheduleIfNotRunning(
                intake.runTorqueFOC(config::getCoralL1ScoreTorqueCurrent)
                        .withName("Intake.l1Score"));
    }

    public static void scoreCoral() {
        scheduleIfNotRunning(
                Commands.sequence(
                                new WaitCommand(config.getScoreDelay()),
                                intake.runTorqueFOC(config::getCoralScoreTorqueCurrent))
                        .withName("Intake.scoreCoral"));
    }

    public static void scoreAlgae() {
        scheduleIfNotRunning(
                intake.runTorqueFOC(config::getAlgaeScoreTorqueCurrent)
                        .withName("Intake.scoreAlgae"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(intake.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(intake.ensureBrakeMode());
    }

    // private static Command runVoltageCurrentLimits(
    //         DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
    //     return intake.runVoltageCurrentLimits(voltage, supplyCurrent, torqueCurrent);
    // }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for a subsystem only if it's not already the running command
     *
     * @param subsystem the subsystem the command requires
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(intake);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
