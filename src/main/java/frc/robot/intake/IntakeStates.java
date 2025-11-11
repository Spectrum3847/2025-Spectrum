package frc.robot.intake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.intake.Intake.IntakeConfig;
import frc.spectrumLib.Telemetry;

public class IntakeStates {
    private static Intake intake = Robot.getIntake();
    private static IntakeConfig config = Robot.getConfig().intake;

    private static final Trigger photonAlgaeRemoval =
            Robot.getPilot()
                    .photonRemoveL2Algae
                    .or(Robot.getPilot().photonRemoveL3Algae)
                    .and(photon);

    public static final Trigger hasGamePiece = new Trigger(intake::hasIntakeGamePiece);
    public static final Trigger hasCoral =
            hasGamePiece.and(intake.aboveVelocityRPM(() -> 0, () -> 0));
    public static final Trigger hasAlgae =
            algae.and(netAlgae.not(), intake.aboveCurrent(config::getHasAlgaeCurrent, () -> 0));
    // hasGamePiece.and(intake.belowVelocityRPM(() -> 0, () -> 0));

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(
                intake.defaultHoldOrStop().ignoringDisable(true).withName("Intake.default"));
    }

    public static void setStates() {
        // intakeRunning.onFalse(intake.getDefaultCommand());
        Robot.getPilot()
                .home_select
                .or(Robot.getOperator().home_select)
                .onTrue(intake.runVoltage(() -> 0));

        stationIntaking.or(photonAlgaeRemoval).onFalse(intake.getDefaultCommand());

        netAlgae.and(actionState)
                .whileTrue(
                        // runVoltageCurrentLimits(
                        //         config::getAlgaeScoreVoltage,
                        //         config::getAlgaeScoreSupplyCurrent,
                        //         config::getAlgaeScoreTorqueCurrent));
                        intake.runTorqueFOC(config::getAlgaeScoreTorqueCurrent));

        // hasGamePiece.onTrue(intake.getDefaultCommand());

        stationIntaking
                .or(photonAlgaeRemoval)
                .whileTrue(
                        // intake.intakeCoral(
                        //                 config::getCoralIntakeTorqueCurrent,
                        //                 config::getCoralIntakeSupplyCurrent)
                        //         .withName("Intake.StationIntaking"));
                        intake.runTorqueFOC(config::getCoralIntakeTorqueCurrent));

        groundCoral.whileTrue(
                // intake.intakeCoral(
                //                 config::getCoralGroundTorqueCurrent,
                //                 config::getCoralGroundSupplyCurrent)
                //         .withName("Intake.GroundCoral"));
                intake.runTorqueFOC(config::getCoralGroundTorqueCurrent));

        algae.and(photon.not())
                .whileTrue(
                        // intake.intakeAlgae(
                        //                 config::getAlgaeIntakeTorqueCurrent,
                        //                 config::getAlgaeIntakeSupplyCurrent)
                        //         .withName("Intake.Algae"));
                        intake.runTorqueFOC(config::getAlgaeIntakeTorqueCurrent));

        L1Coral.and(actionState)
                .whileTrue(
                        // runVoltageCurrentLimits(
                        //         config::getCoralL1ScoreVoltage,
                        //         config::getCoralL1ScoreSupplyCurrent,
                        //         config::getCoralL1ScoreTorqueCurrent));
                        intake.runTorqueFOC(config::getCoralL1ScoreTorqueCurrent));

        Robot.getOperator()
                .processorScore_LT
                .whileTrue(intake.runTorqueFOC(config::getCoralIntakeTorqueCurrent));

        branch.and(actionState, L4Coral.not())
                .onTrue(
                        new WaitCommand(config.getScoreDelay())
                                .andThen(
                                        // runVoltageCurrentLimits(
                                        //         config::getCoralScoreVoltage,
                                        //         config::getCoralScoreSupplyCurrent,
                                        //         config::getCoralScoreTorqueCurrent));
                                        intake.runTorqueFOC(config::getCoralScoreTorqueCurrent)));

        // coastMode.whileTrue(log(coastMode()));
        // coastMode.onFalse(log(ensureBrakeMode()));
    }

    public static void neutral() {
        scheduleIfNotRunning(intake.runVoltage(() -> 0));
    }

    public static void intakeCoral() {
        scheduleIfNotRunning(intake.runTorqueFOC(config::getCoralGroundTorqueCurrent));
    }

    public static void intakeAlgae() {
        scheduleIfNotRunning(intake.runTorqueFOC(config::getAlgaeIntakeTorqueCurrent));
    }

    public static void holdCoral() {
        scheduleIfNotRunning(intake.runTorqueFOC(config::getCoralHoldTorqueCurrent));
    }

    public static void l1Score() {
        scheduleIfNotRunning(intake.runTorqueFOC(config::getCoralL1ScoreTorqueCurrent));
    }

    public static void scoreCoral() {
        scheduleIfNotRunning(intake.runTorqueFOC(config::getCoralScoreTorqueCurrent));
    }

    public static void scoreAlgae() {
        scheduleIfNotRunning(intake.runTorqueFOC(config::getAlgaeScoreTorqueCurrent));
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
