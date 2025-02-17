package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        stationIntaking.and(backwardMode.not()).whileTrue(log(coralIntake()));
        stationIntaking.and(backwardMode).whileTrue(log(reverse(coralIntake())));
        stationExtendedIntake.and(backwardMode.not()).whileTrue(log(coralIntake()));
        stationExtendedIntake.and(backwardMode).whileTrue(log(reverse(coralIntake())));

        scoreState.and(L2Coral).and(backwardMode.not()).onTrue(log(score2()));
        scoreState.and(L2Coral).and(backwardMode).onTrue(log(reverse(score2())));
        scoreState.and(L3Coral).and(backwardMode.not()).onTrue(log(score3()));
        scoreState.and(L3Coral).and(backwardMode).onTrue(log(reverse(score3())));
        scoreState.and(L4Coral).and(backwardMode.not()).onTrue(log(score4()));
        scoreState.and(L4Coral).and(backwardMode).onTrue(log(reverse(score4())));

        L2Algae.and(backwardMode.not()).whileTrue(log(l2Algae()));
        L2Algae.and(backwardMode).whileTrue(log(reverse(l2Algae())));
        L3Algae.and(backwardMode.not()).whileTrue(log(l3Algae()));
        L3Algae.and(backwardMode).whileTrue(log(reverse(l3Algae())));

        L1Coral.and(backwardMode.not()).whileTrue(log(l1Coral()));
        L1Coral.and(backwardMode).whileTrue(log(reverse(l1Coral())));
        L2Coral.and(backwardMode.not()).whileTrue(log(l2Coral()));
        L2Coral.and(backwardMode).whileTrue(log(reverse(l2Coral())));
        L3Coral.and(backwardMode.not()).whileTrue(log(l3Coral()));
        L3Coral.and(backwardMode).whileTrue(log(reverse(l3Coral())));
        L4Coral.and(backwardMode.not()).whileTrue(log(l4Coral()));
        L4Coral.and(backwardMode).whileTrue(log(reverse(l4Coral())));

        barge.and(backwardMode.not()).whileTrue(log(barge()));
        barge.and(backwardMode).whileTrue(log(reverse(barge())));
        homeAll.whileTrue(log(home()));

        algaeHandoff.whileTrue(log(handOffAlgae()));
        coralHandoff.whileTrue(log(handOffAlgae()));
    }

    public static DoubleSupplier getPosition() {
        return () -> elbow.getPositionPercentage();
    }

    public static Command score2() {
        double newPos = 15 + config.getL2Coral();
        return elbow.moveToPercentage(() -> elbow.checkReversed(() -> newPos))
                .withName("Elbow.score2");
    }

    public static Command score3() {
        double newPos = 15 + config.getL3Coral();
        return elbow.moveToPercentage(() -> elbow.checkReversed(() -> newPos))
                .withName("Elbow.score3");
    }

    public static Command score4() {
        double newPos = 20 + config.getL4Coral();
        return elbow.moveToPercentage(() -> elbow.checkReversed(() -> newPos))
                .withName("Elbow.score4");
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
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getL2Algae))
                .withName("Elbow.l2Algae");
    }

    public static Command l3Algae() {
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getL3Algae))
                .withName("Elbow.l3Algae");
    }

    public static Command barge() {
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getBarge))
                .withName("Elbow.barge");
    }

    public static Command l1Coral() {
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getL1Coral))
                .withName("Twist.L1Coral");
    }

    public static Command l2Coral() {
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getL2Coral))
                .withName("Elbow.l2Coral");
    }

    public static Command l3Coral() {
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getL3Coral))
                .withName("Elbow.l3Coral");
    }

    public static Command l4Coral() {
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getL4Coral))
                .withName("Elbow.l4Coral");
    }

    // missing auton Elbow commands, add when auton is added

    public static Command floorIntake() {
        return elbow.moveToPercentage(config::getFloorIntake).withName("Elbow.FloorIntake");
    }

    public static Command coralIntake() {
        return elbow.moveToPercentage(() -> elbow.checkReversed(config::getCoralIntake))
                .withName("Elbow.CoralIntake");
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

    // Negate position command
    protected static Command reverse(Command cmd) {
        return cmd.deadlineFor(
                Commands.startEnd(() -> config.setReversed(true), () -> config.setReversed(false)));
    }
}
