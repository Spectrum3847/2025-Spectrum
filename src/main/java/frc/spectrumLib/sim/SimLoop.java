package frc.spectrumLib.sim;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.DoubleConsumer;

/**
 * Shared high-frequency simulation loop. Mechanism sims register a step callback and are driven
 * from a single {@link Notifier} thread far faster than the 50 Hz main loop, so the simulated
 * TalonFX closed loops advance close to their real ~1 kHz cadence and PID gains behave like
 * hardware. This mirrors the swerve drivetrain's sim thread.
 *
 * <p>Sim-only: registration is a no-op on real hardware, so no thread is started on the roboRIO.
 */
public final class SimLoop {

    private static final double PERIOD_SECS = 1.0 / 200.0; // 200 Hz

    private static final List<DoubleConsumer> steps = new CopyOnWriteArrayList<>();
    private static Notifier notifier;
    private static double lastTime;
    /** Creates a new SimLoop instance. */
    private SimLoop() {}

    /**
     * Registers a step callback, invoked with the real elapsed time (seconds) since the previous
     * tick. The first registration starts the shared thread. No-op outside simulation.
     *
     * @param step the per-tick update to run
     */
    public static synchronized void register(DoubleConsumer step) {
        if (!Utils.isSimulation()) {
            return;
        }
        steps.add(step);
        if (notifier == null) {
            lastTime = Utils.getCurrentTimeSeconds();
            Notifier started = new Notifier(SimLoop::tick);
            started.startPeriodic(PERIOD_SECS);
            notifier = started;
        }
    }
    /** Tick. */
    private static void tick() {
        double now = Utils.getCurrentTimeSeconds();
        double dt = now - lastTime;
        lastTime = now;
        for (DoubleConsumer step : steps) {
            step.accept(dt);
        }
    }
}
