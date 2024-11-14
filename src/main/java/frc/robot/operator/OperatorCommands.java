package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/** This class should have any command calls that directly call the Operator */
public class OperatorCommands {
    private static Operator operator = Robot.getOperator();

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        operator.setDefaultCommand(rumble(0, 99999).repeatedly().withName("Operator.default"));
    }

    /** Command that can be used to rumble the Operator controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return operator.rumbleCommand(intensity, durationSeconds);
    }

    // public static Command cancelCommands() {
    //     return new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    //             .withName("OperatorCancelAll");
    // }

    // public static Command killTheRobot() {
    //     return new InstantCommand(() -> operatorError()).ignoringDisable(true);
    // }

    // public static void operatorError() {
    //     throw new KillRobotException("The robot was killed by operator");
    // }

    // public static Command manualPivot() {
    //     return pivot.runManualOutput(() -> -operator.controller.getRightY() * 0.5);
    // }

    // public static Command manualClimber() {
    //     // return new FunctionalCommand(
    //     //         () -> climber.toggleReverseSoftLimit(false),
    //     //         () -> climber.setPercentOutput(-operator.controller.getRightY()),
    //     //         (b) -> climber.toggleReverseSoftLimit(true),
    //     //         () -> false,
    //     //         Robot.climber);
    //     return climber.runPercentage(() -> -operator.controller.getRightY());
    // }

    // public static Command manualElevator() {
    //     return elevator.runPercentage(() -> -operator.controller.getLeftY());
    // }
}