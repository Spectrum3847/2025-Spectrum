package frc.spectrumLib.framework;

/**
 * Counts robot loops so work can be done "once per loop" or "every Nth loop" without a timer.
 *
 * <p>{@link frc.robot.Robot#robotPeriodic()} calls {@link #next()} first thing. Anything that wants
 * to do something once per loop remembers the count it last ran at and compares; anything that
 * wants a slower cadence uses {@link #every(int)}. Both are a long compare, so they are safe to
 * call from tight per-loop paths such as status-signal reads.
 *
 * <p>Main robot thread only. The count is not touched in unit tests, so once-per-loop caches there
 * refresh exactly once.
 */
public final class RobotLoop {
    private static long count = 0;

    private RobotLoop() {}

    /** Advances to the next loop. Called once, at the top of {@code robotPeriodic()}. */
    public static void next() {
        count++;
    }

    /**
     * Returns the current loop number. Starts at 0 and increases by one per {@link #next()}.
     *
     * @return the current loop count
     */
    public static long count() {
        return count;
    }

    /**
     * Returns {@code true} on every {@code n}th loop, so a 20 ms loop with {@code n = 5} is 10 Hz.
     * All callers in the same loop agree, which keeps related values on the same records.
     *
     * @param n loops between {@code true} results; values below 1 mean every loop
     * @return whether this is a loop on which the periodic work should run
     */
    public static boolean every(int n) {
        return n <= 1 || count % n == 0;
    }
}
