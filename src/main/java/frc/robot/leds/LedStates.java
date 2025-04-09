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
import frc.robot.vision.VisionStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.util.Util;
import java.util.function.BooleanSupplier;

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
        // homeFinishLED(RobotStates.isAtHome.and(Util.teleop, RobotStates.staged.not()), 8);

        // // Coral and Algae Led Commands
        coralModeLED(RobotStates.coral.and(Util.teleop), 6);
        algaeModeLED(RobotStates.algae.and(Util.teleop), 6);
        // coralStagedLED(RobotStates.stagedCoral.and(Util.teleop), 7);
        // algaeStagedLED(RobotStates.stagedAlgae.and(Util.teleop), 7);
        // hasCoralLED(IntakeStates.hasCoral.and(Util.teleop), 7);
        // // hasAlgaeLED(IntakeStates.hasAlgae.and(Util.teleop), 7);
        rightCoralLED(RobotStates.rightScore.and(Util.teleop), 8);

        // Elevator Led Commands
        // elevatorUpLED(ElevatorStates.isUp.and(Util.teleop), 6);

        // Climb Led Commands
        // climbReadyLED(ClimbStates.isLatched.and(RobotStates.climbPrep, Util.teleop), 6);

        // Limelight Led Commands
        seesTagDefault(VisionStates.seeingTag.and(Util.teleop), 5);
        seesTagAndCoralMode(VisionStates.seeingTag.and(RobotStates.coral, Util.teleop), 7);
        seesTagAndAlgaeMode(VisionStates.seeingTag.and(RobotStates.algae, Util.teleop), 7);
        seesTagAndRightCoral(VisionStates.seeingTag.and(RobotStates.rightScore, Util.teleop), 9);
    }

    /** Default LED commands for each mode */
    private static Trigger ledDefaultCommand(
            String name, SpectrumLEDs sLeds, LEDPattern pattern, Trigger trigger) {
        int priority = -1;
        return trigger.and(sLeds.checkPriority(priority), sLeds.defaultTrigger)
                // .onTrue(sLeds.setPattern(pattern, priority).withName(name));
                .onTrue(sLeds.setPattern(pattern, priority));
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
        return trigger.and(sLed.checkPriority(priority))
                .whileTrue(
                        log(
                                setPatternWithReverseCheck(
                                        name, sLed, pattern, priority, () -> sLed == left)));
    }

    private static Command setPatternWithReverseCheck(
            String name,
            SpectrumLEDs sLed,
            LEDPattern pattern,
            int priority,
            BooleanSupplier front) {
        return Commands.either(
                        sLed.setPattern(pattern, priority)
                                .until(
                                        () ->
                                                RobotStates.reverse.getAsBoolean()
                                                                != front.getAsBoolean()
                                                        && !RobotStates.photon.getAsBoolean()),
                        // sLed.setPattern(pattern.atBrightness(Percent.of(25)), priority)
                        sLed.setPattern(sLed.edges(Color.kOrange, 3).overlayOn(pattern), priority)
                                .until(
                                        () ->
                                                RobotStates.reverse.getAsBoolean()
                                                        == front.getAsBoolean()),
                        () ->
                                (RobotStates.reverse.getAsBoolean() == front.getAsBoolean())
                                        || RobotStates.photon.getAsBoolean())
                .repeatedly();
    }

    static void homeFinishLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.HomeFinish",
                right,
                right.bounce(right.purple, 3)
                        .blend(right.solid(right.purple).atBrightness(Percent.of(75))),
                priority,
                trigger);
        withReverseLedCommand(
                "left.HomeFinish",
                left,
                right.bounce(right.purple, 3)
                        .blend(right.solid(right.purple).atBrightness(Percent.of(75))),
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
        // withReverseLedCommand(
        //         "right.CoralMode", right, right.solid(Color.kCoral), priority, trigger);
        // withReverseLedCommand("left.CoralMode", left, left.solid(Color.kCoral), priority,
        // trigger);
        ledCommand("right.CoralMode", right, right.solid(Color.kCoral), priority, trigger);
        ledCommand("left.CoralMode", left, left.solid(Color.kCoral), priority, trigger);
    }

    static void algaeModeLED(Trigger trigger, int priority) {
        // withReverseLedCommand(
        //         "right.AlgaeMode", right, right.solid(Color.kMediumSeaGreen), priority, trigger);
        // withReverseLedCommand(
        //         "left.AlgaeMode", left, left.solid(Color.kMediumSeaGreen), priority, trigger);
        ledCommand("right.AlgaeMode", right, right.solid(Color.kMediumSeaGreen), priority, trigger);
        ledCommand("left.AlgaeMode", left, left.solid(Color.kMediumSeaGreen), priority, trigger);
    }

    static void coralStagedLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.CoralStaged", right, right.breathe(Color.kCoral, 1), priority, trigger);
        withReverseLedCommand(
                "left.CoralStaged", left, left.breathe(Color.kCoral, 1), priority, trigger);
    }

    static void algaeStagedLED(Trigger trigger, int priority) {
        withReverseLedCommand(
                "right.AlgaeStaged",
                right,
                right.breathe(Color.kMediumSeaGreen, 1),
                priority,
                trigger);
        withReverseLedCommand(
                "left.AlgaeStaged",
                left,
                left.breathe(Color.kMediumSeaGreen, 1),
                priority,
                trigger);
    }

    static void rightCoralLED(Trigger trigger, int priority) {
        // withReverseLedCommand(
        //         "right.RightCoral",
        //         right,
        //         right.edges(Color.kGreen, 5).overlayOn(right.breathe(Color.kCoral, 1)),
        //         priority,
        //         trigger);
        // withReverseLedCommand(
        //         "left.RightCoral",
        //         left,
        //         left.edges(Color.kGreen, 5).overlayOn(left.breathe(Color.kCoral, 1)),
        //         priority,
        //         trigger);
        ledCommand("right.RightCoral", right, right.solid(Color.kGreen), priority, trigger);
        ledCommand("left.LeftCoral", left, left.solid(Color.kGreen), priority, trigger);
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

    static void seesTagDefault(Trigger trigger, int priority) {
        ledCommand("right.SeesTag", right, right.bounce(Color.kYellow, 3), priority, trigger);
        ledCommand("left.SeesTag", left, left.bounce(Color.kYellow, 3), priority, trigger);
    }

    static void seesTagAndCoralMode(Trigger trigger, int priority) {
        ledCommand(
                "right.SeesTagAndCoralMode",
                right,
                right.edges(Color.kYellow, 5).overlayOn(right.solid(Color.kCoral)),
                priority,
                trigger);
        ledCommand(
                "left.SeesTagAndCoralMode",
                left,
                left.edges(Color.kYellow, 5).overlayOn(left.solid(Color.kCoral)),
                priority,
                trigger);
    }

    static void seesTagAndAlgaeMode(Trigger trigger, int priority) {
        ledCommand(
                "right.SeesTagAndAlgaeMode",
                right,
                right.edges(Color.kYellow, 5).overlayOn(right.solid(Color.kMediumSeaGreen)),
                priority,
                trigger);
        ledCommand(
                "left.SeesTagAndAlgaeMode",
                left,
                left.edges(Color.kYellow, 5).overlayOn(left.solid(Color.kMediumSeaGreen)),
                priority,
                trigger);
    }

    static void seesTagAndRightCoral(Trigger trigger, int priority) {
        ledCommand(
                "right.SeesTagAndRightCoral",
                right,
                right.edges(Color.kYellow, 5).overlayOn(right.solid(Color.kGreen)),
                priority,
                trigger);
        ledCommand(
                "left.SeesTagAndRightCoral",
                left,
                left.edges(Color.kYellow, 5).overlayOn(left.solid(Color.kGreen)),
                priority,
                trigger);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
