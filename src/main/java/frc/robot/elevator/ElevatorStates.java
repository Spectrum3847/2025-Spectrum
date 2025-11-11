package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();
    private static ElevatorConfig config = Robot.getConfig().elevator;

    /* Check Elevator States */
    public static final Trigger isUp =
            elevator.atPercentage(config::getElevatorIsUpHeight, config::getTriggerTolerance);
    public static final Trigger isHigh =
            elevator.atPercentage(config::getElevatorIsHighHeight, config::getTriggerTolerance);
    public static final Trigger isHome =
            elevator.atRotations(config::getHome, config::getTriggerTolerance);

    public static final Trigger isL1Coral =
            elevator.atRotations(config::getExL1Coral, config::getTriggerTolerance);
    public static final Trigger isL2Coral =
            elevator.atRotations(config::getExL2Coral, config::getTriggerTolerance);
    public static final Trigger isL3Coral =
            elevator.atRotations(config::getExL3Coral, config::getTriggerTolerance);
    public static final Trigger isL4Coral =
            elevator.atRotations(config::getExL4Coral, config::getTriggerTolerance);

    public static final Trigger isL2Algae =
            elevator.atRotations(config::getL2Algae, config::getTriggerTolerance);
    public static final Trigger isL3Algae =
            elevator.atRotations(config::getL3Algae, config::getTriggerTolerance);

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(holdPosition().withName("Elevator.default"));
        // Removed run when disabled, so that the elevator doesn't jump up on enable
    }

    public static void setStates() {
        Robot.getOperator()
                .antiSecretClimb_LTRSup
                .whileTrue(move(config::getFullExtend, "Elevator.fullExtend"));
    }

    // -------------------- State Commands --------------------
    public static void home() {
        scheduleIfNotRunning(move(config::getHome, "Elevator.home"));
    }

    public static void groundCoral() {
        scheduleIfNotRunning(move(config::getClawGroundCoralIntake, "Elevator.groundAlgae"));
    }

    public static void humanCoral() {
        scheduleIfNotRunning(move(config::getStationIntake, "Elevator.humanCoral"));
    }

    public static void groundAlgae() {
        scheduleIfNotRunning(move(config::getClawGroundAlgaeIntake, "Elevator.groundAlgae"));
    }

    public static void L1Coral() {
        scheduleIfNotRunning(
                move(config::getL1Coral, config::getExL1Coral, "Elevator.stationIntake"));
    }

    public static void L2CoralPrep() {
        scheduleIfNotRunning(
                move(config::getL2Coral, config::getExL2Coral, "Elevator.L2CoralPrep"));
    }

    public static void L2CoralRelease() {
        scheduleIfNotRunning(
                move(config::getL2Score, config::getExL2Score, "Elevator.L2CoralRelease"));
    }

    public static void L3CoralPrep() {
        scheduleIfNotRunning(
                move(config::getL3Coral, config::getExL3Coral, "Elevator.L3CoralPrep"));
    }

    public static void L3CoralRelease() {
        scheduleIfNotRunning(
                move(config::getL3Score, config::getExL3Score, "Elevator.L3CoralRelease"));
    }

    public static void L4CoralPrep() {
        scheduleIfNotRunning(
                move(config::getL4Coral, config::getExL4Coral, "Elevator.L4CoralPrep"));
    }

    public static void L4CoralRelease() {
        scheduleIfNotRunning(
                move(config::getL4Score, config::getExL4Score, "Elevator.L4CoralRelease"));
    }

    public static void L2Algae() {
        scheduleIfNotRunning(move(config::getL2Algae, "Elevator.L2Algae"));
    }

    public static void L3Algae() {
        scheduleIfNotRunning(move(config::getL3Algae, "Elevator.L3Algae"));
    }

    public static void netAlgae() {
        scheduleIfNotRunning(move(config::getNetAlgae, "Elevator.netAlgae"));
    }

    private static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
    }

    public static Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    public static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    public static Command move(DoubleSupplier rotations, String name) {
        return elevator.move(rotations, rotations).withName(name);
    }

    public static Command move(DoubleSupplier rotations, DoubleSupplier exRotaitons, String name) {
        return elevator.move(rotations, exRotaitons).withName(name);
    }

    public static Command slowMove(DoubleSupplier rotations, String name) {
        return elevator.slowMove(rotations, rotations).withName(name);
    }

    public static Command slowMove(
            DoubleSupplier rotations, DoubleSupplier exRotaitons, String name) {
        return elevator.slowMove(rotations, exRotaitons).withName(name);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for a subsystem only if it's not already the running command
     *
     * @param subsystem the subsystem the command requires
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(elevator);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
