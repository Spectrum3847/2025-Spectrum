package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.climb.ClimbStates;
import frc.robot.intake.IntakeStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.util.Util;

public class LedStates {
    private static LedFull leds = Robot.getLeds();
    private static LedRight right = leds.getRight();
    private static LedLeft left = leds.getLeft();

    static void bindTriggers() {
        disabledPattern(Util.disabled.and(Util.dsAttached));
        teleopPattern(Util.teleop.and(Util.dsAttached));
        autoPattern(Util.autoMode.and(Util.dsAttached));
        testModePattern(Util.testMode.and(Util.dsAttached));

        // General Led Commands
        homeFinishLED(RobotStates.isAtHome.and(Util.teleop), 8);

        // Coral and Algae Led Commands
        coralModeLED(RobotStates.coral.and(Util.teleop), 6);
        algaeModeLED(RobotStates.algae.and(Util.teleop), 6);
        hasCoralLED(IntakeStates.hasCoral.and(Util.teleop), 7);
        hasAlgaeLED(IntakeStates.hasAlgae.and(Util.teleop), 7);

        // Elevator Led Commands
        // elevatorUpLED(ElevatorStates.isUp.and(Util.teleop), 6);

        // Climb Led Commands
        climbReadyLED(ClimbStates.isLatched.and(RobotStates.climbPrep, Util.teleop), 6);
    }

    /** Default LED commands for each mode */
    private static Trigger ledDefaultCommand(
            String name, SpectrumLEDs sLeds, LEDPattern pattern, Trigger trigger) {
        int priority = -1;
        return trigger.and(sLeds.checkPriority(priority), sLeds.defaultTrigger)
                .onTrue(sLeds.setPattern(pattern, priority).withName(name));
    }

    static void disabledPattern(Trigger trigger) {
        ledDefaultCommand(
                "right.disabledPattern", right, right.ombre(right.purple, right.white), trigger);
        ledDefaultCommand(
                "left.disabledPattern", left, left.ombre(left.purple, left.white), trigger);
    }

    static void teleopPattern(Trigger trigger) {
        ledDefaultCommand("right.teleopPattern", right, right.bounce(right.purple, 3), trigger);
        ledDefaultCommand("left.teleopPattern", left, left.bounce(left.purple, 3), trigger);
    }

    static void autoPattern(Trigger trigger) {
        ledDefaultCommand(
                "right.autoPattern", right, right.countdown(Timer::getFPGATimestamp, 15), trigger);

        ledDefaultCommand(
                "left.autoPattern", left, left.countdown(Timer::getFPGATimestamp, 15), trigger);
    }

    static void testModePattern(Trigger trigger) {
        ledDefaultCommand("right.testModePattern", right, right.chase(Color.kRed, 0.2, 1), trigger);
        ledDefaultCommand("left.testModePattern", left, left.chase(Color.kRed, 0.2, 1), trigger);
    }

    /** LED non-default Commands, set the priority value to see which command takes precedence */
    private static Trigger ledCommand(
            String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
        return trigger.and(sLed.checkPriority(priority))
                .whileTrue(log(sLed.setPattern(pattern, priority).withName(name)));
    }

    private static Trigger withReverseLedCommand(
            String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
        if (sLed == left) {
            return checkReversedFrontLedCommand(name, sLed, pattern, priority, trigger);
        } else {
            return checkReversedRearLedCommand(name, sLed, pattern, priority, trigger);
        }
    }

    private static Trigger checkReversedFrontLedCommand(
            String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
        return trigger.and(sLed.checkPriority(priority))
                .whileTrue(
                        Commands.either(
                                sLed.setPattern(pattern, priority).withName(name),
                                sLed.setPattern(pattern.blink(Seconds.of(1)), priority)
                                        .withName(name),
                                RobotStates.reverse));
    }

    private static Trigger checkReversedRearLedCommand(
            String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
        return trigger.and(sLed.checkPriority(priority))
                .whileTrue(
                        Commands.either(
                                sLed.setPattern(pattern.blink(Seconds.of(1)), priority)
                                        .withName(name),
                                sLed.setPattern(pattern, priority).withName(name),
                                RobotStates.reverse));
    }

    static void homeFinishLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.HomeFinish",
                right,
                right.bounce(right.purple, 3).blend(right.solid(right.purple)),
                priority,
                trigger);
        withReverseLedCommand(
                "left.HomeFinish",
                left,
                right.bounce(right.purple, 3).blend(right.solid(right.purple)),
                priority,
                trigger);
    }

    static void elevatorUpLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.ElevatorUp", right, right.blink(Color.kBlue, 0.25), priority, trigger);
        withReverseLedCommand(
                "left.ElevatorUp", left, left.blink(Color.kBlue, 0.25), priority, trigger);
    }

    static void climbReadyLED(Trigger trigger, int priority) {
        ledCommand("right.ClimbReady", right, right.scrollingRainbow(), priority, trigger);
        ledCommand("left.ClimbReady", left, left.scrollingRainbow(), priority, trigger);
    }

    static void coralModeLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.CoralMode", right, right.solid(Color.kCoral), priority, trigger);
        withReverseLedCommand("left.CoralMode", left, left.solid(Color.kCoral), priority, trigger);
    }

    static void algaeModeLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.AlgaeMode", right, right.solid(Color.kMediumSeaGreen), priority, trigger);
        withReverseLedCommand(
                "left.AlgaeMode", left, left.solid(Color.kMediumSeaGreen), priority, trigger);
    }

    static void hasCoralLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.HasCoral", right, right.breathe(Color.kCoral, 1), priority, trigger);
        withReverseLedCommand(
                "left.HasCoral", left, left.breathe(Color.kCoral, 1), priority, trigger);
    }

    static void hasAlgaeLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.HasAlgae",
                right,
                right.breathe(Color.kMediumSeaGreen, 1),
                priority,
                trigger);
        withReverseLedCommand(
                "left.HasAlgae", left, left.breathe(Color.kMediumSeaGreen, 1), priority, trigger);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
