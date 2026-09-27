package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/**
 * Configuration data for a linear (elevator-style) mechanism simulation. Stores physical
 * properties, Mechanism2d display settings, and optional mount attachment used by {@link
 * LinearSim}.
 */
public class LinearConfig {
    /** Number of Kraken X60 motors driving the linear stage. */
    @Getter private int numMotors = 1;
    /** Gear ratio between the motor and the elevator drum. */
    @Getter private double elevatorGearing = 5;
    /** Mass of the moving carriage in kilograms, used by the physics simulation. */
    @Getter private double carriageMassKg = 1;
    /** Radius of the elevator drum in metres, used to convert rotations to linear position. */
    @Getter private double drumRadius = Units.inchesToMeters(0.955 / 2);
    /** Minimum travel height of the mechanism in metres. */
    @Getter private double minHeight = 0;
    /** Maximum travel height of the mechanism in metres. */
    @Getter
    private double maxHeight = 10000; // Units.inchesToMeters(Robot.config.elevator.maxHeight);

    // Display Config
    /**
     * Angle of the linear stage in the Mechanism2d canvas (degrees; 0 = horizontal, 90 = vertical,
     * CCW positive).
     */
    @Getter private double angle = 90; // O is horizontal, 90 is vertical, CCW is positive
    /** Color of the moving stage ligament in the Mechanism2d canvas. */
    @Getter private Color8Bit color = new Color8Bit(Color.kPurple);
    /** Stroke width of the stage ligaments in the Mechanism2d canvas. */
    @Getter private double lineWidth = 10;
    /** Initial X position of the static root in the Mechanism2d canvas (metres). */
    @Getter private double initialX = 0.5;
    /** Initial Y position of the static root in the Mechanism2d canvas (metres). */
    @Getter private double initialY = 0;
    /** Current X position of the static root, updated when mounted (metres). */
    @Getter @Setter private double staticRootX = 0.5;
    /** Current Y position of the static root, updated when mounted (metres). */
    @Getter @Setter private double staticRootY = 0;
    /**
     * Visual length of the static (non-moving) stage ligament in the Mechanism2d canvas (metres).
     */
    @Getter private double staticLength = 20;
    /** Visual length of the moving stage ligament in the Mechanism2d canvas (metres). */
    @Getter private double movingLength = 20;
    /** Whether this linear stage is attached to a parent {@link Mount}. */
    @Getter private boolean mounted = false;
    /** The parent mount this linear stage is attached to, or {@code null} if not mounted. */
    @Getter private Mount mount;
    /** X position of the mount at simulation start (metres). */
    @Getter private double initMountX;
    /** Y position of the mount at simulation start (metres). */
    @Getter private double initMountY;
    /** Angle of the mount at simulation start (radians). */
    @Getter private double initMountAngle;

    /**
     * Creates a LinearConfig with the minimum required positioning and mechanical parameters.
     *
     * @param x initial X position of the stage root in the Mechanism2d canvas (metres)
     * @param y initial Y position of the stage root in the Mechanism2d canvas (metres)
     * @param gearing gear ratio between the motor and the elevator drum
     * @param drumRadius radius of the elevator drum in metres
     */
    public LinearConfig(double x, double y, double gearing, double drumRadius) {
        this.initialX = x;
        this.initialY = y;
        staticRootX = initialX;
        staticRootY = initialY;
        elevatorGearing = gearing;
        this.drumRadius = drumRadius;
    }

    /**
     * Sets the number of motors driving this linear stage.
     *
     * @param numMotors number of Kraken X60 motors
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
    public LinearConfig setReversedLinkage(boolean reversedLinkage) {
        this.reversedLinkage = reversedLinkage;
        return this;
    }
    /**
     * Sets the number of motors.
     *
     * @param numMotors the number of motors
     * @return this config, for chaining
     */
    public LinearConfig setNumMotors(int numMotors) {
        this.numMotors = numMotors;
        return this;
    }

    /**
     * Sets the carriage mass used by the physics simulation.
     *
     * @param carriageMassKg mass of the moving carriage in kilograms
     * @return this config for chaining
     */
    public LinearConfig setCarriageMass(double carriageMassKg) {
        this.carriageMassKg = carriageMassKg;
        return this;
    }

    /**
     * Sets the orientation angle of the linear stage in the Mechanism2d canvas.
     *
     * @param angle angle in degrees (0 = horizontal, 90 = vertical, CCW positive)
     * @return this config for chaining
     */
    public LinearConfig setAngle(double angle) {
        this.angle = angle;
        return this;
    }

    /**
     * Sets the color of the moving stage ligament in the Mechanism2d canvas.
     *
     * @param color the display color
     * @return this config for chaining
     */
    public LinearConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }

    /**
     * Sets the stroke width of the stage ligaments in the Mechanism2d canvas.
     *
     * @param lineWidth stroke width in pixels
     * @return this config for chaining
     */
    public LinearConfig setLineWidth(double lineWidth) {
        this.lineWidth = lineWidth;
        return this;
    }

    /**
     * Sets the visual length of the static (non-moving) stage ligament.
     *
     * @param lengthInches length in inches; stored internally as metres
     * @return this config for chaining
     */
    public LinearConfig setStaticLength(double lengthInches) {
        this.staticLength = Units.inchesToMeters(lengthInches);
        ;
        return this;
    }

    /**
     * Sets the visual length of the moving stage ligament.
     *
     * @param lengthInches length in inches; stored internally as metres
     * @return this config for chaining
     */
    public LinearConfig setMovingLength(double lengthInches) {
        this.movingLength = Units.inchesToMeters(lengthInches);
        return this;
    }

    /**
     * Sets the maximum travel height of the mechanism.
     *
     * @param lengthInches maximum height in inches; stored internally as metres
     * @return this config for chaining
     */
    public LinearConfig setMaxHeight(double lengthInches) {
        this.maxHeight = Units.inchesToMeters(lengthInches);
        return this;
    }

    /**
     * Attaches this linear stage to a parent {@link LinearSim} mount so its root tracks the parent
     * stage's position.
     *
     * @param sim the parent linear stage to mount onto, or {@code null} to leave unmounted
     * @return this config for chaining
     */
    public LinearConfig setMount(LinearSim sim) {
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
     * Attaches this linear stage to a parent {@link ArmSim} mount so its root tracks the arm tip's
     * position.
     *
     * @param sim the parent arm to mount onto, or {@code null} to leave unmounted
     * @return this config for chaining
     */
    public LinearConfig setMount(ArmSim sim) {
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
