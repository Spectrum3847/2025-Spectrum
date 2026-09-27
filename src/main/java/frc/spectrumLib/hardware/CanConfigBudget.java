package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleFunction;

/**
 * A cap on how long boot may spend on CAN device configuration that is not working.
 *
 * <p>Every Phoenix configuration call blocks until the device answers or the timeout expires. That
 * is invisible when the bus is healthy and quadratic in frustration when it is not: each of the
 * roughly two dozen CANivore devices is configured with several such calls, plus retry loops on top
 * of them, and every one of those waits out its full timeout against a device that is not there.
 *
 * <p>In the 2026-09-19 Chezy practice match the CANivore bus went down mid-match on a cut wire. The
 * roboRIO was restarted from the Driver Station and the code looked like it never came back: robot
 * init took <b>64.0 s</b> against 16.9 s in the match before it, and the rio then sat at 93-99% CPU
 * with 58-77% of loops over 25 ms. It had in fact come back -- it was just unusable, and the match
 * was over before anyone could tell the difference.
 *
 * <p>This class bounds that. Callers run their configuration through {@link #run} and consult
 * {@link #exhausted()} before retrying or before making calls that are merely nice to have. Once
 * the cumulative time lost to failed configuration passes {@link #BUDGET_SECONDS}, the budget is
 * spent: retries stop, optional calls are skipped, and an alert names it. Boot then completes with
 * a dead bus in roughly the time it takes to fail once per device instead of ten times.
 *
 * <p><b>Open question, 2026-09-19.</b> This bounds the boot, but it is not the whole of what
 * happened after that restart. With the bus already dead, CPU sat at a normal 40 to 50 percent for
 * 20 s after init, then went to 95 to 100 percent at exactly 30.0 s after init completed and stayed
 * there, with 58 to 77 percent of loops over 25 ms. Nothing in the Java loop changed at that moment
 * (vision and the scheduler were each under 20 ms) and the DataLog writer thread was starved too,
 * so the load is native or a background service, not the robot loop. Bench test to find it: unplug
 * the CANivore, boot, watch {@code System/CpuPercent} at plus 30 s, then try disabling the Phoenix
 * diagnostics server ({@code Unmanaged.setPhoenixDiagnosticsStartTime(-1)}) and the SignalLogger to
 * see which one it is. Log: {@code logs/matches/FRC_20260919_034444.wpilog}.
 *
 * <p>The cap is deliberately cause-agnostic. A cut wire, a powered-down bus, a wrong bus name and a
 * missing CANivore all present identically here, and the correct response to all of them is the
 * same: stop waiting and come up.
 */
public final class CanConfigBudget {

    private CanConfigBudget() {}

    /**
     * Cumulative seconds of failed configuration allowed before retries are abandoned.
     *
     * <p>Generous enough that a single slow device, or a handful of first-call timeouts on a
     * healthy bus, never trips it; small enough that a fully dead bus costs a few seconds rather
     * than the better part of a minute.
     */
    public static final double BUDGET_SECONDS = 3.0;

    /**
     * Timeout for a single boot configuration call, in seconds.
     *
     * <p>Phoenix's own default is 0.050 s. This is deliberately the same: the fix here is not a
     * shorter individual wait, which would risk failing on a healthy-but-busy bus, but refusing to
     * repeat the wait once it is clear nothing is answering.
     */
    public static final double CALL_TIMEOUT_SECONDS = 0.050;

    /** Retry count used while the budget holds. */
    public static final int MAX_ATTEMPTS = 10;

    private static double spentSeconds = 0;
    private static int failedCalls = 0;

    private static final Alert exhaustedAlert = new Alert("", AlertType.kError);

    /**
     * Whether the budget is spent and callers should stop retrying.
     *
     * @return true once failed configuration has cost more than {@link #BUDGET_SECONDS}
     */
    public static boolean exhausted() {
        return spentSeconds >= BUDGET_SECONDS;
    }

    /**
     * Attempts to retry, given how many attempts a caller should make for one configuration.
     *
     * @return {@link #MAX_ATTEMPTS} while the budget holds, otherwise 1
     */
    public static int maxAttempts() {
        return exhausted() ? 1 : MAX_ATTEMPTS;
    }

    /**
     * Runs one configuration call with the bounded timeout and charges its cost to the budget when
     * it fails.
     *
     * <p>A successful call costs nothing, however long it took: time spent talking to a device that
     * is really there is time well spent. Only failures are charged, because only failures repeat.
     *
     * @param name the device or mechanism being configured, for the alert text
     * @param call the configuration call, taking the timeout in seconds
     * @return the status the call returned
     */
    public static StatusCode run(String name, DoubleFunction<StatusCode> call) {
        double start = Timer.getFPGATimestamp();
        StatusCode result = call.apply(CALL_TIMEOUT_SECONDS);
        if (!result.isOK()) {
            boolean wasExhausted = exhausted();
            spentSeconds += Timer.getFPGATimestamp() - start;
            failedCalls++;
            if (!wasExhausted && exhausted()) {
                exhaustedAlert.setText(
                        String.format(
                                "CAN config budget spent: %.1f s lost over %d failed device"
                                        + " configs (first noticed at %s). Retries are now off so"
                                        + " boot can finish -- expect mechanisms to be"
                                        + " unconfigured. Check the CAN bus wiring and power.",
                                spentSeconds, failedCalls, name));
                exhaustedAlert.set(true);
            }
        }
        return result;
    }

    /**
     * Seconds charged to the budget so far. Logged so a boot that nearly tripped the cap is visible
     * before the one that trips it.
     *
     * @return cumulative seconds lost to failed configuration
     */
    public static double getSpentSeconds() {
        return spentSeconds;
    }

    /**
     * Number of failed configuration calls so far.
     *
     * @return the count of failed calls
     */
    public static int getFailedCalls() {
        return failedCalls;
    }
}
