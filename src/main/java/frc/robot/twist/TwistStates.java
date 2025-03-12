package frc.robot.twist;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.twist.Twist.TwistConfig;
import frc.spectrumLib.Telemetry;

public class TwistStates {
    private static Twist twist = Robot.getTwist();
    private static TwistConfig config = Robot.getConfig().twist;

    public static void setupDefaultCommand() {
        twist.setDefaultCommand(log(twist.runHoldTwist().withName("Twist.default")));
        // twist.runStop());
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        homeAll.onTrue(twist.twistHome());

        // Robot.getPilot().reZero_start.onTrue(twist.resetToInitialPos());

        stationIntaking.whileTrue(
                twist.moveToDegrees((config::getStationIntake)).withName("Twist.stationIntake"));

        algae.or(groundAlgae, stagedAlgae)
                .whileTrue(twist.moveToDegrees(config::getAlgaeIntake).withName("Twist.Algae"));

        Robot.getPilot()
                .groundAlgae_RT
                .whileTrue(
                        twist.moveToDegrees(config::getAlgaeIntake).withName("Twist.AlgaeIntake"));

        L1Coral.or(groundCoral)
                .whileTrue(
                        twist.moveToDegrees(config::getL1Coral).withName("Twist.l1CoralOrGround"));

        netAlgae.whileTrue(twist.moveToDegrees(config::getNet).withName("Twist.Net"));

        branch.and(rightScore)
                .whileTrue(twist.moveToDegrees(config::getRightCoral).withName("Twist.rightCoral"));
        branch.and(rightScore.not())
                .whileTrue(twist.moveToDegrees(config::getLeftCoral).withName("Twist.leftCoral"));
    }

    public static Command coastMode() {
        return twist.coastMode().withName("Twist.CoastMode");
    }

    public static Command ensureBrakeMode() {
        return twist.ensureBrakeMode().withName("Twist.BrakeMode");
    }

    public static Command stopMotor() {
        return twist.runStop().withName("Twist.stop");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    // Negate position command
    protected static Command reverse(Command cmd) {
        return cmd.deadlineFor(
                Commands.startEnd(() -> config.setReversed(true), () -> config.setReversed(false)));
    }
}
