package frc.spectrumLib.framework;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;

/**
 * The base robot class for Spectrum robots. Extends WPILib's TimedRobot and configures joystick
 * connection warning silencing and extends the loop overrun watchdog timeout to 200 ms.
 */
public class SpectrumRobot extends TimedRobot {

    /**
     * Loop length above which WPILib prints "Loop time of Xs overrun" plus a per-section epoch
     * breakdown to the Driver Station.
     *
     * <p>0.20 s keeps the console quiet in matches. It also hides everything below it: in the
     * 2026-09-05 logs 60 to 90 percent of enabled loops ran over 25 ms and none of that reached the
     * console. The {@code Scheduler/*} timers in the wpilog are the primary record of loop time;
     * drop this to 0.04 for a diagnostic build when the per-section epoch print is wanted, bearing
     * in mind each print is itself work the loop has to do.
     */
    public static final double LOOP_OVERRUN_WARNING_SECONDS = 0.20;

    /**
     * Constructs a SpectrumRobot, silencing joystick connection warnings and setting the loop
     * overrun watchdog to {@link #LOOP_OVERRUN_WARNING_SECONDS}.
     */
    public SpectrumRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(LOOP_OVERRUN_WARNING_SECONDS);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(LOOP_OVERRUN_WARNING_SECONDS);
    }
}
