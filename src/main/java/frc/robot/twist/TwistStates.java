package frc.robot.twist;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.twist.Twist.TwistConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class TwistStates {
    private static Twist twist = Robot.getTwist();
    private static TwistConfig config = Robot.getConfig().twist;

    public static void setupDefaultCommand() {
        twist.setDefaultCommand(
                log(twist.runHoldTwist().ignoringDisable(true).withName("Twist.default")));
    }

    public static void setStates() {
        stationIntaking.whileTrue(log(coralIntake()));
        // algaeFloor.whileTrue(tuneTwist());
        algaeFloorIntake.whileTrue(log(floorIntake()));
        L2Algae.whileTrue(log(l2Algae()));
        L3Algae.whileTrue(log(l3Algae()));
        L2Coral.whileTrue(log(l2Coral()));
        L3Coral.whileTrue(log(l3Coral()));
        L4Coral.whileTrue(log(l4Coral()));
        barge.whileTrue(log(barge()));
        homeAll.whileTrue(log(home()));
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    public static Command runTwist(DoubleSupplier speed) {
        return twist.runPercentage(speed).withName("Twist.runTwist");
    }

    public static Command home() {
        return twist.moveToPercentage(config::getHome).withName("Twist.home");
    }

    public static Command moveToPercentage(DoubleSupplier percent) {
        return twist.moveToPercentage(percent).withName("Twist.moveToPercentage");
    }

    /* Scoring positions */

    public static Command l2Algae() {
        return twist.moveToPercentage(config::getL2Algae).withName("Twist.l2Algae");
    }

    public static Command l3Algae() {
        return twist.moveToPercentage(config::getL3Algae).withName("Twist.l3Algae");
    }

    public static Command l1Coral() {
        return twist.moveToPercentage(config::getL1Coral).withName("Twist.l1Coral");
    }

    public static Command l2Coral() {
        return twist.moveToPercentage(config::getL2Coral).withName("Twist.l2Coral");
    }

    public static Command l3Coral() {
        return twist.moveToPercentage(config::getL3Coral).withName("Twist.l3Coral");
    }

    public static Command l4Coral() {
        return twist.moveToPercentage(config::getL4Coral).withName("Twist.l4Coral");
    }

    public static Command floorIntake() {
        return twist.moveToPercentage(config::getFloorIntake).withName("Twist.floorIntake");
    }

    public static Command coralIntake() {
        return twist.moveToPercentage(config::getCoralIntake).withName("Twist.coralIntake");
    }

    public static Command coastMode() {
        return twist.coastMode().withName("Twist.CoastMode");
    }

    public static Command stopMotor() {
        return twist.runStop().withName("Twist.stop");
    }

    public static Command ensureBrakeMode() {
        return twist.ensureBrakeMode().withName("Twist.BrakeMode");
    }

    public static Command barge() {
        return twist.moveToPercentage(config::getBarge).withName("Twist.barge");
    }

    // Tune value command
    public static Command tuneTwist() {
        return twist.moveToPercentage(config::getTuneTwist).withName("Twist.tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
