package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.TuneValue;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();
    private static ElevatorConfig config = Robot.getConfig().elevator;
    private static Pilot pilot = Robot.getPilot();
    private static Operator operator = Robot.getOperator();

    /* Check Elevator States */
    public static final Trigger isUp =
            elevator.atPercentage(config::getElevatorUpHeight, config::getTolerance);
    public static final Trigger isHome =
            elevator.atPercentage(config::getHome, config::getTolerance);

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(
                holdPosition().ignoringDisable(true).withName("Elevator.default"));
    }

    public static void setStates() {
        score.onFalse(home()); // Return home when we stop the scoring action

        // Elevator Extends when the climber is at mid climb
        // Test Mode Buttons
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        score.whileTrue(home());
        L2Algae.whileTrue(l2());
        L2Coral.whileTrue(l2());
        L3Algae.whileTrue(l3());
        L3Coral.whileTrue(l3());
        L4Coral.whileTrue(l4());
        barge.whileTrue(barge());
        handOffAlgae.whileTrue(handOff());
        homeAll.whileTrue(home());
    }

    private static Command runElevator(DoubleSupplier speed) {
        return elevator.runPercentage(speed).withName("Elevator.runElevator");
    }

    public static DoubleSupplier getPosition() {
        return () -> elevator.getPositionRotations();
    }

    public static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
    }

    private static Command fullExtend() {
        return elevator.moveToRotations(config::getFullExtend).withName("Elevator.fullExtend");
    }

    private static Command home() {
        return elevator.moveToRotations(config::getHome)
                .alongWith(elevator.checkMaxCurrent(() -> 100))
                .withName("Elevator.home");
    }

    private static Command handOff() {
        return elevator.moveToRotations(config::getL3)
                .withName("Elevator.handOffUp")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 90.0)
                .andThen(elevator.moveToRotations(config::getL2))
                .withName("Elevator.handOffDown");
    }

    private static Command l2() {
        return elevator.moveToRotations(config::getL2).withName("Elevator.l2");
    }

    private static Command l3() {
        return elevator.moveToRotations(config::getL3).withName("Elevator.l3");
    }

    private static Command l4() {
        return elevator.moveToRotations(config::getL4).withName("Elevator.l4");
    }

    private static Command barge() {
        return elevator.moveToRotations(config::getBarge).withName("Elevator.barge");
    }

    private static Command zero() {
        return elevator.zeroElevatorRoutine().withName("Zero Elevator");
    }

    private static Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    private static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    // Example of a TuneValue that is used to tune a single value in the code
    private static Command tuneElevator() {
        return elevator.moveToRotations(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
