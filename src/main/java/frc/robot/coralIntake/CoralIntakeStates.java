package frc.robot.coralIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.coralIntake.CoralIntake.CoralIntakeConfig;
import frc.robot.elbow.ElbowStates;
import frc.spectrumLib.Telemetry;

public class CoralIntakeStates {
    private static CoralIntake coralIntake = Robot.getCoralIntake();
    private static CoralIntakeConfig config = Robot.getConfig().coralIntake;

    public static void setupDefaultCommand() {
        // intake.setDefaultCommand(
        //         log(
        //                 intake.runVelocity(() -> config.getSlowIntake())
        //                         .ignoringDisable(false)
        //                         .withName("Intake.default")));
        coralIntake.setDefaultCommand(
                log(coralIntake.runStop().ignoringDisable(true).withName("intake.default")));
    }

    public static void setStates() {
        stationIntaking.whileTrue(log(intake()));
        ejecting.whileTrue(log(eject()));
        score.whileTrue(log(intake()));
        handOffAlgae.whileTrue(log(handOffAlgae()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command handOffAlgae() {
        return coralIntake
                .runStop()
                .withName("coralIntake.handOffAlgaeWait")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 95.0)
                .andThen(intake())
                .withName("coralIntake.handOffAlgae");
    }

    private static Command intake() {
        return coralIntake.runVelocityTcFocRpm(config::getIntake).withName("coralIntake.intake");
    }

    private static Command eject() {
        return coralIntake.runVelocityTcFocRpm(config::getEject).withName("coralIntake.eject");
    }

    private static Command coastMode() {
        return coralIntake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return coralIntake.ensureBrakeMode();
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
