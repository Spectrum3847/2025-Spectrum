package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.spectrumLib.Telemetry;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/** Tracks start-up and caught crash events, logging them to a file which doesn't roll over */
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    private static String filePath = "/home/lvuser/crash_tracking.txt";

    /**
     * Logs details of a Throwable exception to a designated file. This method captures the unique
     * run instance UUID, the type of marker (in this case, "Exception"), the current date and time,
     * and the stack trace of the Throwable, if present. The log entry is appended to the file
     * '/home/lvuser/crash_tracking.txt', ensuring that each incident is recorded sequentially
     * without overwriting previous entries. This method is typically used to record unexpected
     * exceptions or crashes that occur during the runtime of the application, aiding in post-event
     * analysis and debugging.
     *
     * @param throwable The Throwable exception to log. Can be any subclass of Throwable, capturing
     *     errors and exceptions that occur during the application's execution.
     */
    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    // used to just log a message to the file
    @SuppressWarnings("unused")
    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filePath, true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            if (e instanceof FileNotFoundException) {
                if (RobotBase.isSimulation()) {
                    Telemetry.print(
                            "CrashTracker failed to save crash file to robot: running in simulation mode");
                } else {
                    Telemetry.print(
                            "CrashTracker failed to save crash file to robot: path `/home/lvuser/crash_tracking.txt` not found");
                }
            } else {
                e.printStackTrace();
            }
        }
    }
}
