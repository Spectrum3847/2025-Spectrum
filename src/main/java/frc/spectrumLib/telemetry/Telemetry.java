package frc.spectrumLib.telemetry;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.spectrumLib.framework.RobotLoop;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Telemetry and logging utility. Extends DogLog to provide structured logging and console output
 * with priority levels.
 *
 * <h2>Three tiers</h2>
 *
 * <ul>
 *   <li>{@link #log} writes to the wpilog only. DogLog skips a record when the value has not
 *       changed, so a boolean or a state string costs nothing while it is steady; a double that
 *       moves every loop costs a record every loop.
 *   <li>{@link #logDash} also publishes the value to NetworkTables at 10 Hz, for the Driver Station
 *       dashboard and the robot app. Use it for exactly the keys a dashboard shows; the whole log
 *       is no longer mirrored to NetworkTables (see {@link #start}).
 *   <li>{@link #slowLogThisLoop()} is true every fifth loop. Wrap logs that do not need loop-rate
 *       resolution (currents, temperatures, vision status) in it.
 * </ul>
 *
 * <p>On 2026-09-05 the robot wrote 1800 to 2800 records a second, every one of them also published
 * to NetworkTables and flushed every 20 ms, on a roboRIO whose CPU sat at 92 to 95 percent and
 * whose main loop ran at 30 ms. The log thread and the NetworkTables server were a full-time job
 * for one of the two cores. These tiers exist to bring that down to what is actually read.
 */
public class Telemetry extends DogLog implements Subsystem {

    /**
     * Tracks the most recent set of active alerts for each severity key to avoid duplicate log
     * entries.
     */
    private static final Map<String, String[]> previousAlerts = new HashMap<>();

    /** Live subscribers for each alert severity, created on first use. */
    private static final Map<String, StringArraySubscriber> alertSubscribers = new HashMap<>();

    /** Named fault conditions that can be surfaced as structured log entries. */
    public enum Fault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    /**
     * Priority levels for printing to the console.
     *
     * <ul>
     *   <li>{@link #NORMAL} — only printed when the global priority is also {@code NORMAL}.
     *   <li>{@link #HIGH} — always printed regardless of the global priority setting.
     * </ul>
     */
    public enum PrintPriority {
        NORMAL,
        HIGH
    }

    /** Minimum priority level a message must have to be written to the console. */
    private static PrintPriority priority = PrintPriority.HIGH;

    // ── NetworkTables mirror ─────────────────────────────────────────────────

    /**
     * SmartDashboard key of the switch that mirrors every log entry to NetworkTables, for live
     * AdvantageScope sessions in the shop. Off by default; always off when the FMS is attached.
     */
    public static final String NT_MIRROR_SWITCH_KEY = "Telemetry/MirrorLogsToNT";

    private static BooleanEntry ntMirrorSwitch;

    /**
     * Cached switch position, read once per loop on the main thread, consumed on the log thread.
     */
    private static volatile boolean ntMirrorEnabled = false;

    // ── Slow tier ────────────────────────────────────────────────────────────

    /** Loops between slow-tier publishes: every fifth 20 ms loop is 10 Hz. */
    public static final int SLOW_LOG_EVERY_LOOPS = 5;

    /**
     * Returns {@code true} on the loops the slow telemetry tier publishes on. Every caller in the
     * same loop agrees, so related values land on the same records.
     *
     * @return whether slow-tier values should be logged this loop
     */
    public static boolean slowLogThisLoop() {
        return RobotLoop.every(SLOW_LOG_EVERY_LOOPS);
    }

    /**
     * Creates a Telemetry instance and registers it as a WPILib subsystem so its {@link
     * #periodic()} method is called every loop cycle.
     */
    public Telemetry() {
        super();
        register();
    }

    /** Called every robot loop cycle. Reads the mirror switch and logs any newly active alerts. */
    @Override
    public void periodic() {
        refreshNtMirrorSwitch();
        logAlerts();
    }

    /** Caches the mirror switch position for the log thread. */
    private static void refreshNtMirrorSwitch() {
        if (ntMirrorSwitch != null) {
            ntMirrorEnabled = ntMirrorSwitch.get();
        }
    }

    /**
     * Entries the log queue may hold before it starts dropping them.
     *
     * <p>The queue exists to absorb the gap between a robot thread that produces records in bursts
     * and a log thread that has to get scheduled to drain them. DogLog's default of 1000 was about
     * a third of a second at the 3000 records per second these logs were running at; 5000 covers a
     * stall of about one and a half seconds for roughly 400 KB of heap in the worst case, and only
     * while a burst is actually queued.
     *
     * <p>Headroom, not a fix. The queue filled on 2026-09-05 because the log thread was starved of
     * CPU, not because the disk was slow; cutting what is logged is the real lever.
     */
    private static final int LOG_ENTRY_QUEUE_CAPACITY = 5000;

    /**
     * Start the telemetry system.
     *
     * <p>Values a dashboard needs are published to NetworkTables individually through {@link
     * #logDash}; everything else stays in the wpilog. The old behaviour of mirroring every entry is
     * behind the {@value #NT_MIRROR_SWITCH_KEY} switch on SmartDashboard, and is forced off
     * whenever the FMS is attached regardless of the switch.
     *
     * @param ntMirrorDefault Initial position of the mirror-everything switch. {@code false} for
     *     the robot; flip it in Elastic when AdvantageScope needs the full live stream.
     * @param captureDs Whether to capture Driver Station data (joysticks, mode) in the log.
     * @param captureNt Whether to capture NetworkTables entries in the log.
     * @param captureConsole Whether to capture console output in the log.
     * @param logExtras Whether to log extra data, like PDH currents, CAN usage, radio connection
     *     status, etc.
     * @param tunableOnFMS Whether tunable values should be read from NetworkTables.
     * @param priority The minimum priority level for console output.
     */
    public static void start(
            boolean ntMirrorDefault,
            boolean captureDs,
            boolean captureNt,
            boolean captureConsole,
            boolean logExtras,
            boolean tunableOnFMS,
            PrintPriority priority) {
        setPriority(priority);

        ntMirrorEnabled = ntMirrorDefault;
        ntMirrorSwitch =
                NetworkTableInstance.getDefault()
                        .getTable("SmartDashboard")
                        .getBooleanTopic(NT_MIRROR_SWITCH_KEY)
                        .getEntry(ntMirrorDefault);
        ntMirrorSwitch.set(ntMirrorDefault);

        Telemetry.setOptions(
                new DogLogOptions()
                        .withNtPublish(Telemetry::mirrorLogsToNt)
                        .withCaptureDs(captureDs)
                        .withCaptureNt(captureNt)
                        .withCaptureConsole(captureConsole)
                        .withNtTunables(tunableOnFMS)
                        .withLogEntryQueueCapacity(LOG_ENTRY_QUEUE_CAPACITY)
                        .withLogExtras(logExtras));
        Telemetry.setPdh(new PowerDistribution());
        /* Display the currently running commands on SmartDashboard*/
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    /**
     * Whether DogLog should mirror log entries to NetworkTables right now. Evaluated by the log
     * thread for every record, so it reads two cached booleans and nothing else.
     */
    private static boolean mirrorLogsToNt() {
        return ntMirrorEnabled && !DriverStation.isFMSAttached();
    }

    /**
     * Sets the priority.
     *
     * @param priority the priority
     */
    private static void setPriority(PrintPriority priority) {
        Telemetry.priority = priority;
    }

    // ── Dashboard tier ───────────────────────────────────────────────────────
    //
    // logDash writes the record on every call and publishes it to NetworkTables on slow-tier
    // loops, so a value that moves every loop reaches the dashboard at 10 Hz and the log at loop
    // rate. The NetworkTables topic only changes when a slow-tier call publishes, so a boolean or a
    // string that flips between ticks still arrives within 100 ms. logDashAlways publishes on every
    // call, for values logged on their own slower cadence that might never land on a tick.

    /**
     * Logs a double and shows it on the dashboard.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDash(String key, double value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    /**
     * Logs a double with a unit and shows it on the dashboard.
     *
     * @param key the log key
     * @param value the value
     * @param unit the unit label recorded as metadata
     */
    public static void logDash(String key, double value, String unit) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value, unit);
        } else {
            log(key, value, unit);
        }
    }

    /**
     * Logs a boolean and shows it on the dashboard.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDash(String key, boolean value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    /**
     * Logs a string and shows it on the dashboard.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDash(String key, String value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    /**
     * Logs an integer and shows it on the dashboard.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDash(String key, long value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    /**
     * Logs a double and publishes it to the dashboard on this very call. For values logged on their
     * own cadence (a timer, a state change) that must not wait for a slow-tier loop.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDashAlways(String key, double value) {
        forceNt.log(key, value);
    }

    /**
     * Logs a double with a unit and publishes it to the dashboard on this very call.
     *
     * @param key the log key
     * @param value the value
     * @param unit the unit label recorded as metadata
     */
    public static void logDashAlways(String key, double value, String unit) {
        forceNt.log(key, value, unit);
    }

    /**
     * Logs a boolean and publishes it to the dashboard on this very call.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDashAlways(String key, boolean value) {
        forceNt.log(key, value);
    }

    /**
     * Logs a string and publishes it to the dashboard on this very call.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDashAlways(String key, String value) {
        forceNt.log(key, value);
    }

    /**
     * Logs an integer and publishes it to the dashboard on this very call.
     *
     * @param key the log key
     * @param value the value
     */
    public static void logDashAlways(String key, long value) {
        forceNt.log(key, value);
    }

    // ── Loop timers ──────────────────────────────────────────────────────────

    /** Start times of open {@link #time} spans, in FPGA microseconds. */
    private static final Map<String, Long> epochStartMicros = new HashMap<>();

    /**
     * Starts a timed span. Pair with {@link #timeEnd(String)} on the same key.
     *
     * <p>Replaces DogLog's timer so the result reaches the dashboard: the {@code Scheduler/*} spans
     * are on the Elastic layout and are the primary record of loop time.
     *
     * @param key the log key the elapsed time will be written to
     */
    public static void time(String key) {
        epochStartMicros.put(key, RobotController.getFPGATime());
    }

    /**
     * Ends a timed span and logs its length in seconds, matching the units DogLog's timer used, so
     * existing analysis of the {@code Scheduler/*} keys keeps working.
     *
     * @param key the key passed to {@link #time(String)}
     */
    public static void timeEnd(String key) {
        Long start = epochStartMicros.remove(key);
        if (start == null) {
            return;
        }
        logDash(key, (RobotController.getFPGATime() - start) / 1_000_000.0, "seconds");
    }

    // ── Commands, prints, alerts ─────────────────────────────────────────────

    /**
     * Wraps a command so that its initialization and end are logged to the "Commands" key.
     *
     * @param cmd The command to wrap
     * @return a decorated command that logs lifecycle events and preserves the original name
     */
    public static Command log(Command cmd) {
        return cmd.deadlineFor(
                        Commands.startEnd(
                                () -> log("Commands", "Init: " + cmd.getName()),
                                () -> log("Commands", "End: " + cmd.getName())))
                .ignoringDisable(cmd.runsWhenDisabled())
                .withName(cmd.getName());
    }

    /** Print a statement if they are enabled */
    public static void print(String output, PrintPriority priority) {
        String out = "TIME: " + String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output;
        if (priority == PrintPriority.HIGH || Telemetry.priority == PrintPriority.NORMAL) {
            System.out.println(out);
        }
        log("Prints", out);
    }

    /**
     * Prints a message at {@link PrintPriority#NORMAL} priority. The message is always written to
     * the DogLog "Prints" key but only echoed to stdout when the global priority allows it.
     *
     * @param output The string to print
     */
    public static void print(String output) {
        print(output, PrintPriority.NORMAL);
    }

    /**
     * Logs any alert newly published under {@code SmartDashboard/Alerts} (errors, warnings, infos)
     * to the "Alerts" DogLog key.
     *
     * <p>Reads the subscribers' change queues rather than the current arrays, so a loop on which
     * nothing changed does no NetworkTables read and allocates nothing. The old version pulled
     * three string arrays out of NetworkTables every loop to compare against last loop's.
     */
    public static void logAlerts() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        logAlertType(ntInstance, "errors", "ERROR");
        logAlertType(ntInstance, "warnings", "WARNING");
        logAlertType(ntInstance, "infos", "INFO");
    }

    /** Logs the alert type. */
    private static void logAlertType(NetworkTableInstance ntInstance, String key, String prefix) {
        StringArraySubscriber subscriber =
                alertSubscribers.computeIfAbsent(
                        key,
                        k ->
                                ntInstance
                                        .getTable("SmartDashboard/Alerts")
                                        .getStringArrayTopic(k)
                                        .subscribe(new String[0]));

        for (String[] alertStrings : subscriber.readQueueValues()) {
            String[] previousAlertStrings = previousAlerts.getOrDefault(key, new String[0]);
            for (String alert : alertStrings) {
                if (!Arrays.asList(previousAlertStrings).contains(alert)) {
                    log("Alerts", prefix + ": " + alert);
                }
            }
            previousAlerts.put(key, alertStrings);
        }
    }
}
