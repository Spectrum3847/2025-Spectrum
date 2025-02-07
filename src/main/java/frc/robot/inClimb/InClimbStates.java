package frc.robot.inClimb;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.elbow.ElbowStates;
import frc.robot.inClimb.InClimb.InClimbConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class InClimbStates {
    private static InClimb inClimb = Robot.getInClimb();
    private static InClimbConfig config = Robot.getConfig().inClimb;

    public static void setupDefaultCommand() {
        inClimb.setDefaultCommand(
                log(inClimb.runHoldInClimb().ignoringDisable(true).withName("InClimb.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        algaeFloorIntake.whileTrue(log(algaeFloorIntake()));
        climbPrep.whileTrue(log(climbPrep()));
        homeAll.onTrue(log(home()));
        coralFloorIntake.whileTrue(log(coralIntake()));
        processorScore.whileTrue(log(processorScore()));
    }

    public static Command runInClimb(DoubleSupplier speed) {
        return inClimb.runPercentage(speed).withName("InClimb.runInClimb");
    }

    // missing auton InClimb commands, add when auton is added

    public static Command intake() {
        return inClimb.moveToPercentage(config::getIntake).withName("InClimb.intake");
    }

    public static Command home() {
        return inClimb.moveToPercentage(config::getHome).withName("InClimb.home");
    }

    public static Command climbPrep() {
        return inClimb.moveToPercentage(config::getPrepClimber).withName("InClimb.prepClimber");
    }

    public static Command algaeFloorIntake() {
        return inClimb.runHoldInClimb()
                .withName("InClimb.waitingForElbow")
                .until(() -> (ElbowStates.getPosition().getAsDouble() < 10.0))
                .andThen(
                        inClimb.moveToPercentage(config::getFloorIntake)
                                .withName("InClimb.floorIntake"));
    }

    public static Command coralIntake() {
        return inClimb.moveToPercentage(config::getCoralIntake).withName("InClimb.coralIntake");
    }

    public static Command processorScore() {
        return inClimb.moveToPercentage(config::getProcessorScore)
                .withName("InClimb.processorScore");
    }

    public static DoubleSupplier getPosition() {
        return (() -> inClimb.getPositionPercentage());
    }

    public static Command coastMode() {
        return inClimb.coastMode().withName("InClimb.CoastMode");
    }

    public static Command stopMotor() {
        return inClimb.runStop().withName("InClimb.stop");
    }

    public static Command ensureBrakeMode() {
        return inClimb.ensureBrakeMode().withName("InClimb.BrakeMode");
    }

    // Tune value command
    public static Command tuneInClimb() {
        // return pivot.moveToPercentage(new TuneValue("Tune InClimb", 0).getSupplier())
        //         .withName("InClimb.Tune");
        return inClimb.moveToPercentage(config::getTuneInClimb);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
