package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ShoulderStates {
    private static Shoulder shoulder = Robot.getShoulder();
    private static ShoulderConfig config = Robot.getConfig().shoulder;
    public static final Trigger isHome = shoulder.atDegrees(config::getHome, config::getTolerance);
    public static final Trigger isNetPosition = shoulder.atDegrees(config::getNetAlgae, () -> 90);
    public static final Trigger isAutonNetPosition =
            shoulder.aboveDegrees(config::getAutonShoulderNetChecker, config::getTolerance);

    public static final Trigger isL1Coral =
            shoulder.atDegrees(config::getExL1Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getExL1Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL2Coral =
            shoulder.atDegrees(config::getExL2Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getExL2Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL3Coral =
            shoulder.atDegrees(config::getExL3Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getExL3Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL4Coral =
            shoulder.atDegrees(config::getExL4Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getExL4Coral(), config::getTolerance)
                                    .and(reverse));

    public static final Trigger isL2Algae =
            shoulder.atDegrees(config::getL2Algae, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getL2Algae(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL3Algae =
            shoulder.atDegrees(config::getL3Algae, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getL3Algae(), config::getTolerance)
                                    .and(reverse));

    public static void setupDefaultCommand() {
        shoulder.setDefaultCommand(
                log(shoulder.runHoldShoulder().withName("Shoulder.HoldDefault")));
        // shoulder.runStop());
    }

    public static void setStates() {
        Robot.getOperator().antiSecretClimb_LTRSup.whileTrue(shoulder.move(config::getNetAlgae));
    }

    // -------------------- State Commands --------------------
    public static void home() {
        scheduleIfNotRunning(move(config::getHome, "Shoulder.home"));
    }

    public static void groundCoral() {
        scheduleIfNotRunning(move(config::getGroundCoralIntake, "Shoulder.groundCoral"));
    }

    public static void humanCoral() {
        scheduleIfNotRunning(move(config::getStationIntake, "Shoulder.humanCoral"));
    }

    public static void groundAlgae() {
        scheduleIfNotRunning(move(config::getGroundAlgaeIntake, "Shoulder.groundAlgae"));
    }

    public static void L1Coral() {
        scheduleIfNotRunning(
                move(config::getL1Coral, config::getExL1Coral, "Shoulder.stationIntake"));
    }

    public static void L2CoralPrep() {
        scheduleIfNotRunning(
                move(config::getL2Coral, config::getExL2Coral, "Shoulder.L2CoralPrep"));
    }

    public static void L2CoralRelease() {
        scheduleIfNotRunning(
                move(
                        config::getL2Score,
                        config::getExL2Score,
                        config::getPrescoreDelay,
                        "Shoulder.L2CoralRelease"));
    }

    public static void L3CoralPrep() {
        scheduleIfNotRunning(
                move(config::getL3Coral, config::getExL3Coral, "Shoulder.L3CoralPrep"));
    }

    public static void L3CoralRelease() {
        scheduleIfNotRunning(
                move(
                        config::getL3Score,
                        config::getExL3Score,
                        config::getPrescoreDelay,
                        "Shoulder.L3CoralRelease"));
    }

    public static void L4CoralPrep() {
        scheduleIfNotRunning(
                move(config::getL4Coral, config::getExL4Coral, "Shoulder.L4CoralPrep"));
    }

    public static void L4CoralRelease() {
        scheduleIfNotRunning(
                move(
                        config::getL4CoralScore,
                        config::getExL4Score,
                        config::getPrescoreDelay,
                        "Shoulder.L4CoralRelease"));
    }

    public static void L2Algae() {
        scheduleIfNotRunning(move(config::getL2Algae, "Shoulder.L2Algae"));
    }

    public static void L3Algae() {
        scheduleIfNotRunning(move(config::getL3Algae, "Shoulder.L3Algae"));
    }

    public static void netAlgae() {
        scheduleIfNotRunning(move(config::getNetAlgae, "Shoulder.netAlgae"));
    }

    public static void climbPrep() {
        scheduleIfNotRunning(move(config::getClimbPrep, "Shoulder.climbPrep"));
    }

    public static void slowHome() {
        scheduleIfNotRunning(shoulder.slowMove(config::getHome).withName("Shoulder.slowHome"));
    }

    public static Command move(DoubleSupplier degrees, String name) {
        return shoulder.move(degrees, degrees).withName(name);
    }

    public static Command slowMove(DoubleSupplier degrees, String name) {
        return shoulder.slowMove(degrees).withName(name);
    }

    public static Command move(DoubleSupplier degrees, DoubleSupplier exDegrees, String name) {
        return shoulder.move(degrees, exDegrees).withName(name);
    }

    public static Command move(
            DoubleSupplier degrees, DoubleSupplier exDegrees, DoubleSupplier delay, String name) {
        return new WaitCommand(delay.getAsDouble())
                .andThen(move(degrees, exDegrees, name))
                .withName(name);
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
        Command current = commandScheduler.requiring(shoulder);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
