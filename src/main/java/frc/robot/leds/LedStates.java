package frc.robot.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elevator.ElevatorStates;
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

        // Elevator Led Commands
        elevatorUpLED(ElevatorStates.isUp.and(Util.teleop), 6);
        elevatorHomeLED(ElevatorStates.isHome.and(Util.teleop), 17);
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

    static void elevatorUpLED(Trigger trigger, int priority) {
        ledCommand("right.ElevatorUp", right, right.blink(Color.kBlue, 0.25), priority, trigger);
        ledCommand("left.ElevatorUp", left, left.blink(Color.kBlue, 0.25), priority, trigger);
    }

    static void elevatorHomeLED(Trigger trigger, int priority) {
        ledCommand("right.ElevatorHome", right, right.blink(Color.kPurple, 2), priority, trigger);
        ledCommand("left.ElevatorHome", left, left.blink(Color.kPurple, 2), priority, trigger);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
