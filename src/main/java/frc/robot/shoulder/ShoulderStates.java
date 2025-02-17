package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        stationIntaking.and(backwardMode.not()).whileTrue(log(coralIntake()));
        stationIntaking.and(backwardMode).whileTrue(log(reverse(coralIntake())));
        stationExtendedIntake.and(backwardMode.not()).whileTrue(log(coralExtendedIntake()));
        stationExtendedIntake.and(backwardMode).whileTrue(log(reverse(coralExtendedIntake())));

        L2Algae.and(backwardMode.not()).whileTrue(log(l2Algae()));
        L2Algae.and(backwardMode).whileTrue(log(reverse(l2Algae())));
        L3Algae.and(backwardMode.not()).whileTrue(log(l3Algae()));
        L3Algae.and(backwardMode).whileTrue(log(reverse(l3Algae())));
        barge.and(backwardMode.not()).whileTrue(log(barge()));
        barge.and(backwardMode).whileTrue(log(reverse(barge())));

        L1Coral.and(backwardMode.not()).whileTrue(log(l1Coral()));
        L1Coral.and(backwardMode).whileTrue(log(reverse(l1Coral())));
        L2Algae.and(backwardMode.not()).whileTrue(log(l2Algae()));
        L2Algae.and(backwardMode).whileTrue(log(reverse(l2Algae())));
        L3Algae.and(backwardMode.not()).whileTrue(log(l3Algae()));
        L3Algae.and(backwardMode).whileTrue(log(reverse(l3Algae())));
        L2Coral.and(backwardMode.not()).whileTrue(log(l2Coral()));
        L2Coral.and(backwardMode).whileTrue(log(reverse(l2Coral())));
        L3Coral.and(backwardMode.not()).whileTrue(log(l3Coral()));
        L3Coral.and(backwardMode).whileTrue(log(reverse(l3Coral())));
        L4Coral.and(backwardMode.not()).whileTrue(log(l4Coral()));
        L4Coral.and(backwardMode).whileTrue(log(reverse(l4Coral())));

        algaeHandoff.whileTrue(log(handOffAlgae()));
        coralHandoff.whileTrue(log(handOffAlgae()));

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
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getL2Algae))
                .withName("Shoulder.l2Algae");
    }

    public static Command l3Algae() {
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getL3Algae))
                .withName("Shoulder.l3Algae");
    }

    public static Command l1Coral() {
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getL1Coral))
                .withName("Twist.L1Coral");
    }

    public static Command l2Coral() {
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getL2Coral))
                .withName("Shoulder.l2Coral");
    }

    public static Command l3Coral() {
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getL3Coral))
                .withName("Shoulder.l3Coral");
    }

    public static Command l4Coral() {
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getL4Coral))
                .withName("Shoulder.l4Coral");
    }

    public static Command barge() {
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getBarge))
                .withName("Shoulder.barge");
    }

    // missing auton Shoulder commands, add when auton is added

    public static Command floorIntake() {
        return shoulder.moveToPercentage(config::getFloorIntake).withName("Shoulder.floorIntake");
    }

    public static Command coralIntake() {
        return shoulder.moveToPercentage(() -> shoulder.checkReversed(config::getCoralIntake))
                .withName("Shoulder.coralIntake");
    }

    public static Command coralExtendedIntake() {
        return shoulder.moveToPercentage(
                        () -> shoulder.checkReversed(config::getCoralExtendedIntake))
                .withName("Shoulder.coralExtendedIntake");
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

    // Check robot side command
    protected static Command reverse(Command cmd) {
        return cmd.deadlineFor(
                Commands.startEnd(() -> config.setReversed(true), () -> config.setReversed(false)));
    }
}
