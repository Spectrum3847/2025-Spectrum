package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/**
 * Configuration data for an arm simulation. Stores physical properties, display settings, and
 * optional mount attachment used by {@link ArmSim}.
 */
public class ArmConfig {

    /** Number of Kraken X60 motors driving the arm. */
    @Getter @Setter private int numMotors = 1;
    /** Initial X position of the arm pivot in the Mechanism2d canvas (metres). */
    @Getter @Setter private double initialX;
    /** Initial Y position of the arm pivot in the Mechanism2d canvas (metres). */
    @Getter @Setter private double initialY;
    /** Current X position of the arm pivot used during simulation updates (metres). */
    @Getter @Setter private double pivotX;
    /** Current Y position of the arm pivot used during simulation updates (metres). */
    @Getter @Setter private double pivotY;
    /** Motor rotations required for one full revolution of the arm mechanism. */
    @Getter @Setter private double ratio;
    /** Visual length of the arm ligament in the Mechanism2d canvas (metres). */
    @Getter @Setter private double length;
    /** Moment of inertia used by the physics simulation (kg·m²). */
    @Getter @Setter private double simMOI = 1.2;
    /**
     * Distance from the pivot to the arm's centre of gravity used by the physics simulation
     * (metres).
     */
    @Getter @Setter private double simCGLength = 0.2;
    /** Minimum allowable arm angle (radians). */
    @Getter @Setter private double minAngle;
    /** Maximum allowable arm angle (radians). */
    @Getter @Setter private double maxAngle;
    /** Arm angle at the start of the simulation (radians). */
    @Getter @Setter private double startingAngle;
    /** Whether the physics simulation should apply gravitational force to the arm. */
    @Getter @Setter private boolean simulateGravity = true;
    /** Whether this arm is attached to a parent {@link Mount}. */
    @Getter private boolean mounted = false;
    /** The parent mount this arm is attached to, or {@code null} if not mounted. */
    @Getter private Mount mount;
    /** X position of the mount at simulation start (metres). */
    @Getter private double initMountX;
    /** Y position of the mount at simulation start (metres). */
    @Getter private double initMountY;
    /** Angle of the mount at simulation start (radians). */
    @Getter private double initMountAngle;
    /**
     * When {@code true} the arm's visual angle is expressed in absolute robot-frame degrees; when
     * {@code false} it is relative to the parent mount's current angle.
     */
    @Getter private boolean absAngle;
    /** Color used to draw the arm ligament in the Mechanism2d canvas. */
    @Getter private Color8Bit color = new Color8Bit(Color.kBlue);

    /**
     * Creates an ArmConfig with the required physical and display parameters. Angle arguments are
     * specified in degrees and stored internally as radians.
     *
     * @param initialX initial X position of the pivot in the Mechanism2d canvas (metres)
     * @param initialY initial Y position of the pivot in the Mechanism2d canvas (metres)
     * @param ratio motor rotations per one full arm revolution
     * @param length visual arm length in the Mechanism2d canvas (metres)
     * @param minAngleDegrees minimum allowable arm angle in degrees
     * @param maxAngleDegrees maximum allowable arm angle in degrees
     * @param startingAngleDegrees initial arm angle in degrees
     */
    public ArmConfig(
            double initialX,
            double initialY,
            double ratio,
            double length,
            double minAngleDegrees,
            double maxAngleDegrees,
            double startingAngleDegrees) {
        this.ratio = ratio;
        this.length = length;
        this.minAngle = Math.toRadians(minAngleDegrees);
        this.maxAngle = Math.toRadians(maxAngleDegrees);
        this.startingAngle = Math.toRadians(startingAngleDegrees);
        this.initialX = initialX;
        this.initialY = initialY;
        this.pivotX = initialX;
        this.pivotY = initialY;
    }

    /**
     * Sets the arm's display color in the Mechanism2d canvas.
     *
     * @param color the color to use
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
    public ArmConfig setReversedLinkage(boolean reversedLinkage) {
        this.reversedLinkage = reversedLinkage;
        return this;
    }
    /**
     * Sets the color.
     *
     * @param color the color
     * @return this config, for chaining
     */
    public ArmConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }
    /**
     * Sets whether simulated gravity is applied.
     *
     * @param simulateGravity whether to apply simulated gravity
     * @return this config, for chaining
     */
    public ArmConfig setSimulatedGravity(boolean simulateGravity) {
        this.simulateGravity = simulateGravity;
        return this;
    }

    /**
     * Attaches this arm to a {@link LinearSim} mount so its pivot tracks the linear stage's
     * position.
     *
     * @param sim the linear stage to mount onto, or {@code null} to leave unmounted
     * @param fixedAngle when {@code true} the arm angle is treated as absolute; when {@code false}
     *     it is relative to the mount's current angle
     * @return this config for chaining
     */
    public ArmConfig setMount(LinearSim sim, boolean fixedAngle) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
            this.absAngle = fixedAngle;
        }
        return this;
    }

    /**
     * Attaches this arm to a parent {@link ArmSim} mount so its pivot tracks the parent arm's tip
     * position.
     *
     * @param sim the parent arm to mount onto, or {@code null} to leave unmounted
     * @param absAngle when {@code true} the arm angle is expressed in the absolute robot frame;
     *     when {@code false} it is relative to the parent arm's current angle
     * @return this config for chaining
     */
    public ArmConfig setMount(ArmSim sim, boolean absAngle) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
            this.absAngle = absAngle;
        }
        return this;
    }
}
