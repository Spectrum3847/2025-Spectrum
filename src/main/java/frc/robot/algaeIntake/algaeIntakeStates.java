package frc.robot.algaeIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.algaeIntake.algaeIntake.algaeIntakeConfig;
import frc.spectrumLib.Telemetry;

public class algaeIntakeStates {
    private static algaeIntake algaeIntake = Robot.getAlgaeIntake();
    private static algaeIntakeConfig config = Robot.getConfig().algaeIntake;

    public static void setupDefaultCommand() {
        // intake.setDefaultCommand(
        //         log(
        //                 intake.runVelocity(() -> config.getSlowIntake())
        //                         .ignoringDisable(false)
        //                         .withName("Intake.default")));
        algaeIntake.setDefaultCommand(
                log(algaeIntake.runStop().ignoringDisable(true).withName("intake.default")));
    }

    public static void setStates() {
        intaking.whileTrue(log(intake()));
        ejecting.whileTrue(log(eject()));
        score.whileTrue(log(intake()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command intake() {
        return algaeIntake.runVelocityTcFocRpm(config::getIntake).withName("algaeIntake.intake");
    }

    private static Command eject() {
        return algaeIntake.runVelocityTcFocRpm(config::getEject).withName("algaeIntake.eject");
    }

    private static Command coastMode() {
        return algaeIntake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return algaeIntake.ensureBrakeMode();
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}