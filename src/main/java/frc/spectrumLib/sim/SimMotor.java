package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/** Shared wiring between a {@link TalonFX} and the WPILib physics sims in this package. */
public final class SimMotor {
    /** Creates a new SimMotor instance. */
    private SimMotor() {}

    /**
     * Returns the motor's sim state, flipping {@code Orientation} when the sim's travel runs
     * opposite the motor's positive direction.
     *
     * <p>The physics sims here are driven by {@code getMotorVoltage()} and feed position back
     * through {@code setRawRotorPosition()}; {@code Orientation} negates both as a matched pair, so
     * flipping it mirrors the whole loop and leaves it self-consistent.
     *
     * <p>Deliberately <em>not</em> derived from the motor's invert — CTRE documents {@code
     * Orientation} as describing mechanical linkage, not the invert, and the invert alone does not
     * determine the sign. What matters is the direction each sim's geometry produces:
     *
     * <ul>
     *   <li>{@code ElevatorSim} travels {@code [minHeight, max]} with min normally 0, so its rotor
     *       term is always positive.
     *   <li>{@code ArmSim} feeds back {@code angle - startingAngle}, which is negative whenever the
     *       arm starts at its maximum.
     * </ul>
     *
     * <p>So an inverted motor is fine on a sim whose geometry already runs negative, and broken on
     * one that runs positive: the model clamps at its limit, feeds the same position back, and the
     * mechanism never moves. Only the mechanism author knows which case applies, hence the explicit
     * per-sim flag.
     *
     * @param motor the motor driving the simulated mechanism
     * @param reversedLinkage {@code true} when the sim's travel opposes the motor's positive
     *     direction
     * @return the motor's (cached) sim state, with orientation set
     */
    public static TalonFXSimState simState(TalonFX motor, boolean reversedLinkage) {
        TalonFXSimState simState = motor.getSimState();
        simState.Orientation =
                reversedLinkage
                        ? ChassisReference.Clockwise_Positive
                        : ChassisReference.CounterClockwise_Positive;
        return simState;
    }
}
