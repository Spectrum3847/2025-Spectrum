package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.robot.elevator.ElevatorStates;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ElbowStates {
    private static Elbow elbow = Robot.getElbow();
    private static ElbowConfig config = Robot.getConfig().elbow;

    public static final Trigger isHome =
            elbow.atDegrees(() -> (config.getHome() + config.getOffset()), config::getTolerance);
    public static final Trigger pastElevator =
            elbow.aboveDegrees(() -> (config.getClearElevator() + 360), config::getTolerance)
                    .or(elbow.belowDegrees(() -> -config.getClearElevator(), config::getTolerance));
    public static final Trigger aboveFloor =
            elbow.aboveDegrees(() -> (config.getGroundCoralIntake() + 10), config::getTolerance);
    public static final Trigger scoreL4 =
            elbow.atDegrees(() -> (config.getL4Coral() - 12), config::getTolerance);

    public static final Trigger atTarget = elbow.atTargetPosition(() -> 0.001);

    public static void setupDefaultCommand() {
        elbow.setDefaultCommand(log(elbow.runHoldElbow().withName("Elbow.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        homeAll.whileTrue(home());

        stationIntaking
                .and(actionState.not())
                .whileTrue(
                        move(
                                config::getStationIntake,
                                config::getStationExtendedIntake,
                                "Elbow.StationIntake"));
        groundCoral
                .and(actionState.not())
                .whileTrue(move(config::getGroundCoralIntake, "Elbow.GroundCoral"));

        groundAlgae
                .and(actionState.not())
                .whileTrue(move(config::getGroundAlgaeIntake, "Elbow.GroundAlgae"));

        // stages elbow
        stagedCoral.and(actionState.not()).whileTrue(move(config::getStage, "Elbow.Stage"));

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

        // Algae
        L2Algae.and(actionState.or(actionPrepState))
                .whileTrue(move(config::getL2Algae, "Elbow.l2Algae"));
        L3Algae.and(actionState.or(actionPrepState))
                .whileTrue(move(config::getL3Algae, "Elbow.l3Algae"));

        netAlgae.whileTrue(move(config::getNet, "Elbow.netAlgae"));
    }

    private static Command home() {
        return move(config::getHome, "Elbow.home");
    }

    // missing auton Elbow commands, add when auton is added

    public static Command move(DoubleSupplier degrees, String name) {
        return elbow.move(degrees, degrees).withName(name);
    }

    public static Command move(DoubleSupplier degrees, DoubleSupplier exDegrees, String name) {
        return elbow.move(degrees, exDegrees).withName(name);
    }

    public static Command stationIntake() {
        return elbow.moveToDegrees(config::getStationIntake).withName("Elbow.StationIntake");
    }

    public static Command stationExtendedIntake() {
        return elbow.moveToDegrees(config::getStationExtendedIntake)
                .withName("Shoulder.stationExtendedIntake");
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
