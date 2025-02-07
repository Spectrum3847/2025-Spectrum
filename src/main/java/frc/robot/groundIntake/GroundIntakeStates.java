package frc.robot.groundIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.groundIntake.GroundIntake.GroundIntakeConfig;
import frc.spectrumLib.Telemetry;

public class GroundIntakeStates {
    private static GroundIntake groundIntake = Robot.getGroundIntake();
    private static GroundIntakeConfig config = Robot.getConfig().groundIntake;

    public static void setupDefaultCommand() {
        // intake.setDefaultCommand(
        //         log(
        //                 intake.runVelocity(() -> config.getSlowIntake())
        //                         .ignoringDisable(false)
        //                         .withName("Intake.default")));
        groundIntake.setDefaultCommand(
                log(groundIntake.runStop().ignoringDisable(true).withName("GroundIntake.default")));
    }

    public static void setStates() {
        algaeFloorIntake.whileTrue(log(intake()));
        coralFloorIntake.whileTrue(log(intake()));
        ejecting.whileTrue(log(eject()));
        score.whileTrue(log(intake()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command intake() {
        return groundIntake.runVelocityTcFocRpm(config::getIntake).withName("GroundIntake.intake");
    }

    private static Command eject() {
        return groundIntake.runVelocityTcFocRpm(config::getEject).withName("GroundIntake.eject");
    }

    private static Command coastMode() {
        return groundIntake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return groundIntake.ensureBrakeMode();
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
