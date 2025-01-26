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
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        L2Algae.whileTrue(log(L2Algae()));
        // L3Algae.whileTrue(log(L3Algae()));
        // L2Coral.whileTrue(log(L2Coral()));
        // L3Coral.whileTrue(log(L3Coral()));
        // L4Coral.whileTrue(log(L4Coral()));
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

    public static Command L2Algae() {
        System.out.println("TwistStates.L2Algae");
        return twist.moveToPercentage(config::getL2Algae).withName("Twist.L2Algae");
    }

    public static Command L3Algae() {
        return twist.moveToPercentage(config::getL3Algae).withName("Twist.L3Algae");
    }

    public static Command L2Coral() {
        return twist.moveToPercentage(config::getL2Coral).withName("Twist.L2Coral");
    }

    public static Command L3Coral() {
        return twist.moveToPercentage(config::getL3Coral).withName("Twist.L3Coral");
    }

    public static Command L1Coral() {
        return twist.moveToPercentage(config::getL1).withName("Twist.L1Coral");
    }

    public static Command L4Coral() {
        return twist.moveToPercentage(config::getL4).withName("Twist.L4Coral");
    }

    public static Command intake() {
        return twist.moveToPercentage(config::getIntake).withName("Twist.intake");
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

    // Tune value command
    public static Command tuneTwist() {
        return twist.moveToPercentage(config::getTuneTwist).withName("Twist.tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
