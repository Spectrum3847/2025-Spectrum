package frc.spectrumLib.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * Caches a DoubleSupplier value so it is computed at most once per scheduler iteration. Note:
 * Subsystem periodic() is typically called after triggers are polled each iteration.
 */
public class CachedDouble extends SubsystemBase implements DoubleSupplier {
    private boolean cached = false;
    private double value;
    private final DoubleSupplier source;

    /**
     * Creates a CachedDouble wrapping the given supplier.
     *
     * @param source the underlying supplier whose value is cached each scheduler iteration
     */
    public CachedDouble(DoubleSupplier source) {
        this.source = source;
    }

    /**
     * Called by the scheduler each iteration to invalidate the cached value so the next {@link
     * #getAsDouble()} call re-queries the source.
     */
    @Override
    public void periodic() {
        cached = false;
    }

    /**
     * Returns the cached value of the source supplier, querying the supplier at most once per
     * scheduler iteration.
     *
     * @return the supplier's value for the current iteration
     */
    @Override
    public double getAsDouble() {
        if (!cached) {
            value = source.getAsDouble();
            cached = true;
        }
        return value;
    }
}
