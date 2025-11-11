package frc.robot.twist;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.shoulder.ShoulderStates;
import frc.robot.twist.Twist.TwistConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;

public class TwistStates {
    private static Twist twist = Robot.getTwist();
    private static TwistConfig config = Robot.getConfig().twist;

    public static final Trigger isLeft =
            twist.atDegrees(config::getLeftCoral, config::getTriggerTolerance)
                    .and(reverse.not())
                    .or(
                            twist.atDegrees(config::getRightCoral, config::getTriggerTolerance)
                                    .and(reverse));
    public static final Trigger isRight =
            twist.atDegrees(config::getRightCoral, config::getTriggerTolerance)
                    .and(reverse.not())
                    .or(
                            twist.atDegrees(config::getLeftCoral, config::getTriggerTolerance)
                                    .and(reverse));

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

        processorAlgae.whileTrue(move(config::getProcessorAlgae, "Twist.l1Algae"));

        netAlgae.and(ShoulderStates.isNetPosition, Util.teleop)
                .whileTrue(twist.netTurret().withName("Twist.NetAlgae"));
        netAlgae.and(ShoulderStates.isAutonNetPosition, Util.autoMode)
                .whileTrue(move(config::getNet, "Twist.NetAlgae"));

        branch.and(
                        rightScore.or(Robot.getOperator().rightScore),
                        actionPrepState,
                        twistAtReef.not())
                .whileTrue(move(config::getRightCoral, config::getStageDelay, "Twist.rightCoral"));
        branch.and(
                        rightScore.or(Robot.getOperator().rightScore),
                        actionPrepState,
                        twistAtReef,
                        toggleReverse.not())
                .whileTrue(moveAwayFromBranch(config::getRightCoral, "Twist.rightCoralOverBranch"));

        branch.and(
                        rightScore.not().or(Robot.getOperator().leftScore),
                        actionPrepState,
                        twistAtReef.not())
                .whileTrue(move(config::getLeftCoral, config::getStageDelay, "Twist.leftCoral"));
        branch.and(
                        rightScore.not().or(Robot.getOperator().leftScore),
                        actionPrepState,
                        twistAtReef,
                        toggleReverse.not())
                .whileTrue(moveAwayFromBranch(config::getLeftCoral, "Twist.leftCoralOverBranch"));

        branch.and(rightScore, actionPrepState, twistAtReef.not(), Util.autoMode)
                .whileTrue(moveAwayFromBranch(config::getRightCoral, "Twist.rightCoral"));

        branch.and(rightScore.not(), actionPrepState, twistAtReef.not(), Util.autoMode)
                .whileTrue(moveAwayFromBranch(config::getLeftCoral, "Twist.leftCoral"));

        climbPrep.whileTrue(move(config::getClimbPrep, "Twist.climbPrep"));
    }

    // -------------------- State Commands --------------------
    public static void home() {
        scheduleIfNotRunning(move(config::getHome, "Twist.home"));
    }

    public static void groundCoral() {
        scheduleIfNotRunning(move(config::getGroundCoralIntake, "Twist.groundCoralIntake"));
    }

    public static void groundAlgae() {
        scheduleIfNotRunning(move(config::getGroundAlgaeIntake, "Twist.groundAlgaeIntake"));
    }

    public static void humanIntake() {
        scheduleIfNotRunning(move(config::getStationIntake, "Twist.humanIntake"));
    }

    public static void coralRight() {
        scheduleIfNotRunning(move(config::getRightCoral, "Twist.coralRight"));
    }

    public static void coralLeft() {
        scheduleIfNotRunning(move(config::getLeftCoral, "Twist.coralLeft"));
    }

    public static void l1Coral() {
        scheduleIfNotRunning(move(config::getL1Coral, "Twist.l1Coral"));
    }

    public static void intakeAlgae() {
        scheduleIfNotRunning(move(config::getAlgaeIntake, "Twist.intakeAlgae"));
    }

    public static void netAlgae() {
        scheduleIfNotRunning(move(config::getNet, "Twist.algaeNet"));
    }

    public static void climbPrep() {
        scheduleIfNotRunning(move(config::getClimbPrep, "Twist.climbPrep"));
    }

    public static Command move(DoubleSupplier degrees, String name) {
        // return twist.move(degrees).withName(name);
        return moveAwayFromElevator(degrees, name);
    }

    public static Command move(DoubleSupplier degrees, boolean clockwise, String name) {
        return twist.move(degrees, clockwise).withName(name);
    }

    public static Command move(DoubleSupplier degrees, DoubleSupplier delay, String name) {
        // return new WaitCommand(delay.getAsDouble()).andThen(move(degrees, name).withName(name));
        return new WaitCommand(delay.getAsDouble())
                .andThen(moveAwayFromElevator(degrees, name).withName(name));
    }

    public static Command moveAwayFromElevator(DoubleSupplier degrees, String name) {
        return twist.moveAwayFromElevatorCheckReverse(degrees).withName(name);
    }

    public static Command moveAwayFromBranch(DoubleSupplier degrees, String name) {
        return twist.moveAwayFromBranchCheckReversed(degrees).withName(name);
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

    /**
     * Schedules a command for a subsystem only if it's not already the running command
     *
     * @param subsystem the subsystem the command requires
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(twist);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
