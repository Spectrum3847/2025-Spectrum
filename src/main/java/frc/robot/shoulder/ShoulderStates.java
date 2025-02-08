package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ShoulderStates {
    private static Shoulder shoulder = Robot.getShoulder();
    private static ShoulderConfig config = Robot.getConfig().shoulder;

    public static void setupDefaultCommand() {
        shoulder.setDefaultCommand(
                log(shoulder.runHoldShoulder().ignoringDisable(true).withName("Shoulder.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()).ignoringDisable(true));
        coastMode.onFalse(log(ensureBrakeMode()));
        stationIntaking.whileTrue(log(coralIntake())); // using intake button to test
        scoreState.whileTrue(home());
        L2Algae.whileTrue(log(l2Algae()));
        L3Algae.whileTrue(log(l3Algae()));
        L2Coral.whileTrue(log(l2Coral()));
        L3Coral.whileTrue(log(l3Coral()));
        L4Coral.whileTrue(log(l4Coral()));
        barge.whileTrue(log(barge()));
        homeAll.whileTrue(home());
    }

    public static Command runShoulder(DoubleSupplier speed) {
        return shoulder.runPercentage(speed).withName("Shoulder.runShoulder");
    }

    public static Command home() {
        return shoulder.moveToPercentage(config::getHome).withName("Shoulder.home");
    }

    public static DoubleSupplier getPosition() {
        return () -> shoulder.getPositionPercentage();
    }

    public static Command climbHome() {
        return shoulder.moveToPercentage(config::getClimbHome).withName("Shoulder.climbHome");
    }

    public static Command handOffAlgae() {
        return shoulder.moveToPercentage(config::getHandAlgae).withName("Elbow.handOffAlgae");
    }

    /* Scoring positions */

    public static Command l2Algae() {
        return shoulder.moveToPercentage(config::getL2Algae).withName("Shoulder.l2Algae");
    }

    public static Command l3Algae() {
        return shoulder.moveToPercentage(config::getL3Algae).withName("Shoulder.l3Algae");
    }

    public static Command l1Coral() {
        return shoulder.moveToPercentage(config::getL1Coral).withName("Twist.L1Coral");
    }

    public static Command l2Coral() {
        return shoulder.moveToPercentage(config::getL2Coral).withName("Shoulder.l2Coral");
    }

    public static Command l3Coral() {
        return shoulder.moveToPercentage(config::getL3Coral).withName("Shoulder.l3Coral");
    }

    public static Command l4Coral() {
        return shoulder.moveToPercentage(config::getL4Coral).withName("Shoulder.l4Coral");
    }

    public static Command barge() {
        return shoulder.moveToPercentage(config::getBarge).withName("Shoulder.barge");
    }

    // missing auton Shoulder commands, add when auton is added

    public static Command floorIntake() {
        return shoulder.moveToPercentage(config::getFloorIntake).withName("Shoulder.floorIntake");
    }

    public static Command coralIntake() {
        return shoulder.moveToPercentage(config::getCoralIntake).withName("Shoulder.coralIntake");
    }

    public static Command coastMode() {
        return shoulder.coastMode().withName("Shoulder.CoastMode");
    }

    public static Command stopMotor() {
        return shoulder.runStop().withName("Shoulder.stop");
    }

    public static Command ensureBrakeMode() {
        return shoulder.ensureBrakeMode().withName("Shoulder.BrakeMode");
    }

    // Tune value command
    public static Command tuneShoulder() {
        // return shoulder.moveToPercentage(new TuneValue("Tune Shoulder", 0).getSupplier())
        //         .withName("Shoulder.Tune");
        return shoulder.moveToPercentage(config::getTuneShoulder);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
