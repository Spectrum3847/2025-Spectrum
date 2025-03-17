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
        twist.setDefaultCommand(log(twist.runHoldTwist().withName("Twist.default")));
        // twist.runStop());
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        homeAll.onTrue(twist.twistHome());

        // Robot.getPilot().reZero_start.onTrue(twist.resetToInitialPos());

        stationIntaking.whileTrue(move(config::getStationIntake, "Twist.stationIntake"));

        stagedAlgae.whileTrue(move(config::getAlgaeIntake, "Twist.Algae"));

        Robot.getPilot()
                .groundAlgae_RT
                .whileTrue(move(config::getGroundAlgaeIntake, "Twist.AlgaeIntake"));

        groundCoral.whileTrue(move(config::getGroundCoralIntake, "Twist.GroundCoralIntake"));

        L1Coral.whileTrue(move(config::getL1Coral, "Twist.l1Coral"));

        netAlgae.whileTrue(move(config::getNet, "Twist.Net"));

        branch.and(rightScore, actionPrepState)
                .whileTrue(move(config::getRightCoral, config::getStageDelay, "Twist.rightCoral"));
        branch.and(rightScore.not(), actionPrepState)
                .whileTrue(move(config::getLeftCoral, config::getStageDelay, "Twist.leftCoral"));
    }

    public static Command move(DoubleSupplier degrees, String name) {
        return twist.move(degrees).withName(name);
    }

    public static Command move(DoubleSupplier degrees, DoubleSupplier delay, String name) {
        return new WaitCommand(delay.getAsDouble()).andThen(move(degrees, name).withName(name));
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
}
