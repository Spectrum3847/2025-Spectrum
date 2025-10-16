package frc.robot.groundIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.groundIntake.GroundIntake.GroundIntakeConfig;
import frc.robot.shoulder.ShoulderStates;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class GroundIntakeStates {
    private static GroundIntake groundIntake = Robot.getGroundIntake();
    private static GroundIntakeConfig config = Robot.getConfig().groundIntake;

    public static final Trigger hasGamePiece = new Trigger(groundIntake::hasIntakeGamePiece);
    public static final Trigger hasCoral =
            hasGamePiece.and(groundIntake.aboveVelocityRPM(() -> 0, () -> 0));

    public static void setupDefaultCommand() {
        groundIntake.setDefaultCommand(
                groundIntake
                        .defaultHoldOrStop()
                        .ignoringDisable(true)
                        .withName("GroundIntake.default"));
    }

    public static void setStates() {
        Robot.getPilot()
                .home_select
                .or(Robot.getOperator().home_select)
                .onTrue(groundIntake.runVoltage(() -> 0));

        groundCoral.whileTrue(
                groundIntake
                        .intakeCoral(
                                config::getCoralIntakeTorqueCurrent,
                                config::getCoralIntakeSupplyCurrent)
                        .withName("GroundIntake.GroundCoral"));

        handOff.and(ShoulderStates.isHandOff)
                .debounce(0.25)
                .whileTrue(
                        groundIntake
                                .runTorqueFOC(config::getCoralHandoffTorqueCurrent)
                                .withName("GroundIntake.HandOff"));

        L1Coral.and(actionState)
                .whileTrue(
                        groundIntake
                                .runTorqueFOC(config::getCoralL1ScoreTorqueCurrent)
                                .withName("GroundIntake.L1Score"));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command coastMode() {
        return groundIntake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return groundIntake.ensureBrakeMode();
    }

    private static Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return groundIntake.runVoltageCurrentLimits(voltage, supplyCurrent, torqueCurrent);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
