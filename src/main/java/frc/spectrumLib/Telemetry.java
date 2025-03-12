package frc.spectrumLib;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Telemetry extends DogLog implements Subsystem {

    private static final Map<String, String[]> previousAlerts = new HashMap<>();

    // TODO: Create default Fault Types
    public enum Fault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    /**
     * Priority levels for printing to the console NORMAL: Low priority, only print if enabled HIGH:
     * High priority, always print
     */
    public enum PrintPriority {
        NORMAL,
        HIGH
    }

    private static PrintPriority priority = PrintPriority.HIGH;

    public Telemetry() {
        super();
        register();
    }

    @Override
    public void periodic() {
        logAlerts();
    }

    public static void start(boolean ntPublish, boolean captureNt, PrintPriority priority) {
        setPriority(priority);
        Telemetry.setOptions(
                new DogLogOptions()
                        .withNtPublish(ntPublish)
                        .withCaptureDs(true)
                        .withCaptureNt(captureNt)
                        .withCaptureConsole(false)
                        .withLogExtras(false));
        Telemetry.setPdh(new PowerDistribution());
        /* Display the currently running commands on SmartDashboard*/
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    private static void setPriority(PrintPriority priority) {
        Telemetry.priority = priority;
    }

    public static Command log(Command cmd) {
        return cmd.deadlineFor(
                        Commands.startEnd(
                                () -> log("Commands", "Init: " + cmd.getName()),
                                () -> log("Commands", "End: " + cmd.getName())))
                .ignoringDisable(cmd.runsWhenDisabled())
                .withName(cmd.getName());
    }

    // Boolean logger
    public static void log(String key, Boolean value) {
        var now = HALUtil.getFPGATime();
        logger.queueLog(now, key, value);
    }

    // double logger
    public static void log(String key, double value) {
        var now = HALUtil.getFPGATime();
        logger.queueLog(now, key, value);
    }

    public static void log(String key, double[] value) {
        var now = HALUtil.getFPGATime();
        logger.queueLog(now, key, value);
    }

    /** Print a statement if they are enabled */
    public static void print(String output, PrintPriority priority) {
        String out = "TIME: " + String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output;
        if (priority == PrintPriority.HIGH || Telemetry.priority == PrintPriority.NORMAL) {
            System.out.println(out);
        }
        log("Prints", out);
    }

    public static void print(String output) {
        print(output, PrintPriority.NORMAL);
    }

    // New method to log alerts from NetworkTables
    public static void logAlerts() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        logAlertType(ntInstance, "errors", "ERROR");
        logAlertType(ntInstance, "warnings", "WARNING");
        logAlertType(ntInstance, "infos", "INFO");
    }

    private static void logAlertType(NetworkTableInstance ntInstance, String key, String prefix) {
        String[] alertStrings =
                ntInstance
                        .getTable("SmartDashboard/Alerts")
                        .getEntry(key)
                        .getStringArray(new String[0]);

        String[] previousAlertStrings = previousAlerts.getOrDefault(key, new String[0]);

        for (String alert : alertStrings) {
            if (!Arrays.asList(previousAlertStrings).contains(alert)) {
                log("Alerts", prefix + ": " + alert);
            }
        }

        previousAlerts.put(key, alertStrings);
    }
}
