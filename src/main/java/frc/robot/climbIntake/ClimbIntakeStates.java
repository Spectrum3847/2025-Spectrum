package frc.robot.climbIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.climbIntake.ClimbIntake.ClimbIntakeConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ClimbIntakeStates {
    private static ClimbIntake climbIntake = Robot.getClimbIntake();
    private static ClimbIntakeConfig config = Robot.getConfig().climbIntake;

    public static void setupDefaultCommand() {
        climbIntake.setDefaultCommand(
                climbIntake.defaultStop().ignoringDisable(true).withName("Intake.default"));
    }

    public static void setStates() {
        // intakeRunning.onFalse(intake.getDefaultCommand());
        Robot.getPilot()
                .home_select
                .or(Robot.getOperator().home_select)
                .onTrue(climbIntake.runVoltage(() -> 0));

        Robot.getOperator()
                .climbPrep_start
                .whileTrue(climbIntake.runTorqueCurrentFoc(config::getIntakeTorqueCurrent));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command coastMode() {
        return climbIntake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return climbIntake.ensureBrakeMode();
    }

    private static Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return climbIntake.runVoltageCurrentLimits(voltage, supplyCurrent, torqueCurrent);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
