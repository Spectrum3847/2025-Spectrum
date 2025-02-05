package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ElbowStates {
    private static Elbow elbow = Robot.getElbow();
    private static ElbowConfig config = Robot.getConfig().elbow;

    public static void setupDefaultCommand() {
        elbow.setDefaultCommand(
                log(elbow.runHoldElbow().ignoringDisable(true).withName("Elbow.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        stationIntaking.whileTrue(log(coralIntake()));
        algaeFloorIntake.whileTrue(log(floorIntake()));
        L2Algae.whileTrue(log(l2Algae()));
        L3Algae.whileTrue(log(l3Algae()));
        L2Coral.whileTrue(log(l2Coral()));
        L3Coral.whileTrue(log(l3Coral()));
        L4Coral.whileTrue(log(l4Coral()));
        barge.whileTrue(log(barge()));
        handOffAlgae.whileTrue(log(handOffAlgae()));
        homeAll.whileTrue(log(home()));
    }

    public static DoubleSupplier getPosition() {
        return () -> elbow.getPositionPercentage();
    }

    public static Command runElbow(DoubleSupplier speed) {
        return elbow.runPercentage(speed).withName("Elbow.runElbow");
    }

    public static Command home() {
        return elbow.moveToPercentage(config::getHome).withName("Elbow.home");
    }

    public static Command handOffAlgae() {
        return elbow.moveToPercentage(config::getHandAlgae).withName("Elbow.handOffAlgae");
    }

    /* Scoring positions */
    public static Command l2Algae() {
        return elbow.moveToPercentage(config::getL2Algae).withName("Elbow.l2Algae");
    }

    public static Command l3Algae() {
        return elbow.moveToPercentage(config::getL3Algae).withName("Elbow.l3Algae");
    }

    public static Command l1Coral() {
        return elbow.moveToPercentage(config::getL1Coral).withName("Twist.L1Coral");
    }

    public static Command l2Coral() {
        return elbow.moveToPercentage(config::getL2Coral).withName("Elbow.l2Coral");
    }

    public static Command l3Coral() {
        return elbow.moveToPercentage(config::getL3Coral).withName("Elbow.l3Coral");
    }

    public static Command l4Coral() {
        return elbow.moveToPercentage(config::getL4Coral).withName("Elbow.l4Coral");
    }

    public static Command barge() {
        return elbow.moveToPercentage(config::getBarge).withName("Elbow.barge");
    }
    // missing auton Elbow commands, add when auton is added

    public static Command floorIntake() {
        return elbow.moveToPercentage(config::getFloorIntake).withName("Elbow.FloorIntake");
    }

    public static Command coralIntake() {
        return elbow.moveToPercentage(config::getCoralIntake).withName("Elbow.CoralIntake");
    }

    public static Command coastMode() {
        return elbow.coastMode().withName("Elbow.CoastMode");
    }

    public static Command stopMotor() {
        return elbow.runStop().withName("Elbow.stop");
    }

    public static Command ensureBrakeMode() {
        return elbow.ensureBrakeMode().withName("Elbow.BrakeMode");
    }

    // Tune value command
    public static Command tuneElbow() {
        // return elbow.moveToPercentage(new TuneValue("Tune Elbow", 0).getSupplier())
        //         .withName("Elbow.Tune");
        return elbow.moveToPercentage(config::getTuneElbow);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
