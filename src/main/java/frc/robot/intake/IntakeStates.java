package frc.robot.intake;

import static frc.robot.RobotStates.*;
import static frc.robot.auton.Auton.autonScore;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.intake.Intake.IntakeConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class IntakeStates {
    private static Intake intake = Robot.getIntake();
    private static IntakeConfig config = Robot.getConfig().intake;

    private static final Trigger photonAlgaeRemoval =
            (Robot.getPilot().photonRemoveL2Algae.or(Robot.getPilot().photonRemoveL3Algae))
                    .and(photon);

    public static final Trigger hasGamePiece = new Trigger(intake::hasIntakeGamePiece);

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
                        runVoltageCurrentLimits(
                                config::getAlgaeScoreVoltage,
                                config::getAlgaeScoreSupplyCurrent,
                                config::getAlgaeScoreTorqueCurrent));

        // hasGamePiece.onTrue(intake.getDefaultCommand());

        stationIntaking
                .or(photonAlgaeRemoval)
                // .whileTrue(runVoltageCurrentLimits(
                //         config::getCoralIntakeVoltage,
                //         config::getCoralIntakeSupplyCurrent,
                //         config::getCoralIntakeTorqueCurrent));
                .whileTrue(
                        intake.intakeCoral(
                                        config::getCoralIntakeTorqueCurrent,
                                        config::getCoralIntakeSupplyCurrent)
                                .withName("Intake.StationIntaking"));

        groundCoral.whileTrue(
                intake.intakeCoral(
                                config::getCoralGroundTorqueCurrent,
                                config::getCoralGroundSupplyCurrent)
                        .withName("Intake.GroundCoral"));

        algae.and(photon.not())
                .whileTrue(
                        intake.intakeAlage(
                                        config::getAlgaeIntakeTorqueCurrent,
                                        config::getAlgaeIntakeSupplyCurrent)
                                .withName("Intake.Algae"));

        L1Coral.and(actionState)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getCoralL1ScoreVoltage,
                                config::getCoralL1ScoreSupplyCurrent,
                                config::getCoralL1ScoreTorqueCurrent));
        branch.and(actionState)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getCoralScoreVoltage,
                                config::getCoralScoreSupplyCurrent,
                                config::getCoralScoreTorqueCurrent));

        autonScore
                .and(photon)
                .onTrue(
                        new WaitCommand(2.0)
                                .andThen(
                                        runVoltageCurrentLimits(
                                                        config::getCoralScoreVoltage,
                                                        config::getCoralScoreSupplyCurrent,
                                                        config::getCoralScoreTorqueCurrent)
                                                .repeatedly()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        Robot.getPilot()
                .testTune_tA
                .whileTrue(
                        intake.intakeCoral(
                                        config::getCoralIntakeTorqueCurrent,
                                        config::getCoralIntakeSupplyCurrent)
                                .withName("Intake.StationIntaking"));
    }

    private static Command coastMode() {
        return intake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return intake.ensureBrakeMode();
    }

    private static Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return intake.runVoltageCurrentLimits(voltage, supplyCurrent, torqueCurrent);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
