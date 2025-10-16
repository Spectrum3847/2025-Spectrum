package frc.robot.claw;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.claw.Claw.ClawConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ClawStates {
    private static Claw claw = Robot.getClaw();
    private static ClawConfig config = Robot.getConfig().claw;

    public static final Trigger hasGamePiece = new Trigger(claw::hasIntakeGamePiece);
    public static final Trigger hasCoral =
            hasGamePiece.and(claw.aboveVelocityRPM(() -> 0, () -> 0));
    public static final Trigger hasAlgae =
            algae.and(netAlgae.not(), claw.aboveCurrent(config::getHasAlgaeCurrent, () -> 0));

    public static void setupDefaultCommand() {
        claw.setDefaultCommand(
                claw.defaultHoldOrStop().ignoringDisable(true).withName("Claw.default"));
    }

    public static void setStates() {
        // clawRunning.onFalse(claw.getDefaultCommand());
        Robot.getPilot()
                .home_select
                .or(Robot.getOperator().home_select)
                .onTrue(claw.runVoltage(() -> 0));

        stationIntaking.onFalse(claw.getDefaultCommand());

        lollipopCoral.whileTrue(claw.runTorqueCurrentFoc(config::getCoralIntakeSupplyCurrent));

        netAlgae.and(actionState)
                .whileTrue(
                        // runVoltageCurrentLimits(
                        //         config::getAlgaeScoreVoltage,
                        //         config::getAlgaeScoreSupplyCurrent,
                        //         config::getAlgaeScoreTorqueCurrent));
                        claw.runTorqueFOC(config::getAlgaeScoreTorqueCurrent));

        // hasGamePiece.onTrue(claw.getDefaultCommand());

        stationIntaking.whileTrue(
                // claw.clawCoral(
                //                 config::getCoralClawTorqueCurrent,
                //                 config::getCoralClawSupplyCurrent)
                //         .withName("Claw.StationIntaking"));
                claw.runTorqueFOC(config::getCoralIntakeSupplyCurrent));

        algae.whileTrue(
                // claw.clawAlgae(
                //                 config::getAlgaeClawTorqueCurrent,
                //                 config::getAlgaeClawSupplyCurrent)
                //         .withName("Claw.Algae"));
                claw.runTorqueFOC(config::getAlgaeIntakeTorqueCurrent));

        Robot.getOperator()
                .processorScore_LT
                .whileTrue(claw.runTorqueFOC(config::getCoralIntakeTorqueCurrent));

        handOff.whileTrue(claw.runTorqueCurrentFoc(config::getCoralIntakeTorqueCurrent));

        branch.and(actionState)
                .onTrue(
                        new WaitCommand(config.getScoreDelay())
                                .andThen(
                                        // runVoltageCurrentLimits(
                                        //         config::getCoralScoreVoltage,
                                        //         config::getCoralScoreSupplyCurrent,
                                        //         config::getCoralScoreTorqueCurrent));
                                        claw.runTorqueFOC(config::getCoralScoreTorqueCurrent)));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command coastMode() {
        return claw.coastMode();
    }

    private static Command ensureBrakeMode() {
        return claw.ensureBrakeMode();
    }

    private static Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return claw.runVoltageCurrentLimits(voltage, supplyCurrent, torqueCurrent);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
