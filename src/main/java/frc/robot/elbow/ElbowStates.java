package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.robot.elevator.ElevatorStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;

public class ElbowStates {
    private static Elbow elbow = Robot.getElbow();
    private static ElbowConfig config = Robot.getConfig().elbow;

    public static final Trigger isHome = elbow.atDegrees(config::getHome, config::getTolerance);
    public static final Trigger pastElevator =
            elbow.aboveDegrees(() -> (config.getClearElevator() + 360), config::getTolerance)
                    .or(elbow.belowDegrees(() -> -config.getClearElevator(), config::getTolerance));
    public static final Trigger aboveFloor =
            elbow.aboveDegrees(() -> (config.getGroundCoralIntake() + 10), config::getTolerance);
    public static final Trigger scoreL4 =
            elbow.atDegrees(() -> (config.getL4Coral() - 12), config::getTolerance);

    public static final Trigger isL1Coral =
            elbow.atDegrees(config::getExL1Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            elbow.atDegrees(() -> -config.getExL1Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL2Coral =
            elbow.atDegrees(config::getExL2Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            elbow.atDegrees(() -> -config.getExL2Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL3Coral =
            elbow.atDegrees(config::getExL3Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            elbow.atDegrees(() -> -config.getExL3Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL4Coral =
            elbow.atDegrees(config::getExL4Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            elbow.atDegrees(() -> -config.getExL4Coral(), config::getTolerance)
                                    .and(reverse));

    public static final Trigger isL2Algae =
            elbow.atDegrees(config::getL2Algae, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            elbow.atDegrees(() -> -config.getL2Algae(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL3Algae =
            elbow.atDegrees(config::getL3Algae, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            elbow.atDegrees(() -> -config.getL3Algae(), config::getTolerance)
                                    .and(reverse));

    public static final Trigger atTarget = elbow.atTargetPosition(() -> 0.001);

    public static void setupDefaultCommand() {
        elbow.setDefaultCommand(log(elbow.runHoldElbow().withName("Elbow.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        homeAll.and(Util.autoMode.not()).whileTrue(home());
        homeAll.and(Util.autoMode).whileTrue(slowHome());

        stationIntaking
                .and(actionState.not())
                .whileTrue(
                        move(
                                config::getStationIntake,
                                // config::getStationExtendedIntake,
                                "Elbow.StationIntake"));

        Robot.getPilot()
                .groundCoral_LB_LT
                .and(actionState.not())
                .whileTrue(groundMove(config::getGroundCoralIntake, "Elbow.GroundCoral"));

        Robot.getPilot()
                .groundAlgae_RT
                .and(actionState.not())
                .whileTrue(move(config::getGroundAlgaeIntake, "Elbow.GroundAlgae"));

        Robot.getOperator().antiSecretClimb_LTRSup.whileTrue(home()); // Stick the Elbow Vertical

        // stages elbow
        stagedCoral
                .and(
                        actionPrepState.not().debounce(getActionPrepToActionTime()),
                        actionState.not(),
                        Util.autoMode.not())
                .whileTrue(move(config::getStage, "Elbow.Stage"));

        L1Coral.and(actionPrepState)
                .whileTrue(move(config::getL1Coral, config::getExL1Coral, "Elbow.L1Coral"));

        L2Coral.and(actionPrepState, ElevatorStates.isL2Coral)
                .whileTrue(move(config::getL2Coral, config::getExL2Coral, "Elbow.l2Coral"));
        L2Coral.and(actionState)
                .whileTrue(move(config::getL2Score, config::getExL2Score, "Elbow.l2Score"));

        L3Coral.and(actionPrepState, ElevatorStates.isL3Coral)
                .whileTrue(move(config::getL3Coral, config::getExL3Coral, "Elbow.l3Coral"));

        L3Coral.and(actionState)
                .whileTrue(move(config::getL3Score, config::getExL3Score, "Elbow.l3Score"));

        L4Coral.and(actionPrepState, ElevatorStates.isL4Coral)
                .whileTrue(move(config::getL4Coral, config::getExL4Coral, "Elbow.l4Coral"));
        L4Coral.and(actionState)
                .whileTrue(move(config::getL4Score, config::getExL4Score, "Elbow.l4Score"));

        L4Coral.and(actionPrepState, ElevatorStates.isL4Coral, Util.autoMode)
                .whileTrue(slowMove(config::getExL4Coral, "Elbow.slowL4Coral"));
        // L4Coral.and(actionState, Util.autoMode)
        //         .whileTrue(slowMove(config::getExL4Score, "Elbow.l4Score"));

        // Algae
        processorAlgae
                .and(actionPrepState)
                .whileTrue(move(config::getProcessorAlgae, "Elbow.processorAlgae"));
        processorAlgae
                .and(actionState)
                .whileTrue(move(config::getHome, "Elbow.processorAlgaeHome"));
        L2Algae.and(actionPrepState).whileTrue(move(config::getL2Algae, "Elbow.l2Algae"));
        L2Algae.and(actionState).whileTrue(move(config::getHome, "Elbow.l2AlgaeHome"));
        L3Algae.and(actionPrepState).whileTrue(move(config::getL3Algae, "Elbow.l3Algae"));
        L3Algae.and(actionState).whileTrue(move(config::getHome, "Elbow.l3AlgaeHome"));

        netAlgae.and(Util.autoMode.not()).whileTrue(move(config::getNet, "Elbow.netAlgae"));
        netAlgae.and(Util.autoMode).whileTrue(slowMove(config::getNet, "Elbow.slowNetAlgae"));

        climbPrep.whileTrue(move(config::getClimbPrep, "Elbow.climbPrep"));
    }

    private static Command home() {
        return move(config::getHome, "Elbow.home");
    }

    private static Command slowHome() {
        return slowMove(config::getHome, "Elbow.slowHome");
    }

    // missing auton Elbow commands, add when auton is added

    public static Command move(DoubleSupplier degrees, String name) {
        return elbow.move(degrees, degrees).withName(name);
    }

    public static Command move(DoubleSupplier degrees, DoubleSupplier exDegrees, String name) {
        return elbow.move(degrees, exDegrees).withName(name);
    }

    public static Command slowMove(DoubleSupplier degrees, String name) {
        return elbow.slowMove(degrees).withName(name);
    }

    public static Command groundMove(DoubleSupplier degrees, String name) {
        return elbow.groundMove(degrees).withName(name);
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

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
