package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

/**
 * Configuration data for a roller mechanism simulation. Stores physical properties, display colors,
 * canvas position, and optional mount attachment used by {@link RollerSim}.
 */
public class RollerConfig {
    /** Outer diameter of the roller in inches, used for physics and visual scaling. */
    @Getter private double rollerDiameterInches = 2;
    /** Number of radial lines used to draw the roller circle in the Mechanism2d canvas. */
    @Getter private int backgroundLines = 36;
    /** Gear ratio between the motor and the roller output shaft. */
    @Getter private double gearRatio = 5;
    /** Moment of inertia of the roller used by the flywheel physics simulation (kg·m²). */
    @Getter private double simMOI = 0.01;
    /** Color displayed when the roller is stationary or below the velocity threshold. */
    @Getter private Color8Bit offColor = new Color8Bit(Color.kBlack);
    /** Color displayed when the roller is spinning in the forward direction. */
    @Getter private Color8Bit fwdColor = new Color8Bit(Color.kGreen);
    /** Color displayed when the roller is spinning in the reverse direction. */
    @Getter private Color8Bit revColor = new Color8Bit(Color.kRed);
    /** Initial X position of the roller axle in the Mechanism2d canvas (metres). */
    @Getter private double initialX = 0;
    /** Initial Y position of the roller axle in the Mechanism2d canvas (metres). */
    @Getter private double initialY = 0;
    /** Whether this roller is attached to a parent {@link Mount}. */
    @Getter private boolean mounted = false;
    /** The parent mount this roller is attached to, or {@code null} if not mounted. */
    @Getter private Mount mount;
    /** X position of the mount at simulation start (metres). */
    @Getter private double initMountX;
    /** Y position of the mount at simulation start (metres). */
    @Getter private double initMountY;
    /** Angle of the mount at simulation start (radians). */
    @Getter private double initMountAngle;

    /**
     * Creates a RollerConfig for a roller with the given diameter.
     *
     * @param diameterInches outer diameter of the roller in inches
     */
    public RollerConfig(double diameterInches) {
        rollerDiameterInches = diameterInches;
    }

    /**
     * Sets the gear ratio between the motor and the roller output shaft.
     *
     * @param ratio gear ratio (motor rotations per roller rotation)
     * @return this config for chaining
     */
    /** True when the sim geometry travels opposite the motor's positive direction. */
    @Getter private boolean reversedLinkage = false;

    /**
     * Mirrors the simulated motor so the sim's travel matches the motor's positive direction. See
     * {@link SimMotor#simState(com.ctre.phoenix6.hardware.TalonFX, boolean)}.
     *
     * @param reversedLinkage true when the sim travels opposite the motor
     * @return this config, for chaining
     */
    public RollerConfig setReversedLinkage(boolean reversedLinkage) {
        this.reversedLinkage = reversedLinkage;
        return this;
    }
    /**
     * Sets the gear ratio.
     *
     * @param ratio the ratio of motor speed to roller speed
     * @return this config, for chaining
     */
    public RollerConfig setGearRatio(double ratio) {
        gearRatio = ratio;
        return this;
    }

    /**
     * Sets the moment of inertia used by the flywheel physics simulation.
     *
     * @param moi moment of inertia in kg·m²
     * @return this config for chaining
     */
    public RollerConfig setSimMOI(double moi) {
        simMOI = moi;
        return this;
    }

    /**
     * Sets the initial position of the roller axle in the Mechanism2d canvas.
     *
     * @param x initial X position in metres
     * @param y initial Y position in metres
     * @return this config for chaining
     */
    public RollerConfig setPosition(double x, double y) {
        initialX = x;
        initialY = y;
        return this;
    }

    /**
     * Attaches this roller to a {@link LinearSim} mount so its axle tracks the linear stage's
     * position.
     *
     * @param sim the linear stage to mount onto, or {@code null} to leave unmounted
     * @return this config for chaining
     */
    public RollerConfig setMount(LinearSim sim) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
        }

        return this;
    }

    /**
     * Attaches this roller to an {@link ArmSim} mount so its axle tracks the arm tip's position.
     *
     * @param sim the parent arm to mount onto, or {@code null} to leave unmounted
     * @return this config for chaining
     */
    public RollerConfig setMount(ArmSim sim) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
        }

        return this;
    }
}
