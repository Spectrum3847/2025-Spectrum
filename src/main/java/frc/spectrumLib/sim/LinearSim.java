package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

/**
 * WPILib-backed simulation of a linear (elevator-style) mechanism driven by one or more Kraken X60
 * motors. Updates the TalonFX sim state each robot period and animates both a static backing
 * ligament and a moving stage ligament in a {@link Mechanism2d} canvas. Implements {@link Mount} so
 * other mechanisms can be attached to the moving stage, and implements {@link Mountable} so this
 * stage can itself be attached to a parent mount.
 */
public class LinearSim implements Mount, Mountable {
    private ElevatorSim elevatorSim;

    private final MechanismRoot2d staticRoot;
    private final MechanismRoot2d root;
    private final MechanismLigament2d staticMech2d;
    private final MechanismLigament2d m_elevatorMech2d;
    /** Configuration containing physical properties and display settings for this linear stage. */
    @Getter private LinearConfig config;

    private TalonFXSimState linearMotorSim;

    /** Always {@link MountType#LINEAR}; used by child mechanisms to determine positioning logic. */
    @Getter private final MountType mountType = MountType.LINEAR;

    /**
     * Creates and registers a linear mechanism simulation.
     *
     * @param config physical and display configuration for the linear stage
     * @param mech the Mechanism2d canvas to draw the stage on
     * @param linearMotorSim the TalonFX sim state of the motor driving the stage
     * @param name unique name prefix used for Mechanism2d element labels
     */
    public LinearSim(LinearConfig config, Mechanism2d mech, TalonFX motor, String name) {
        this.config = config;
        this.linearMotorSim = SimMotor.simState(motor, config.isReversedLinkage());

        this.elevatorSim =
                new ElevatorSim(
                        DCMotor.getKrakenX60Foc(config.getNumMotors()),
                        config.getElevatorGearing(),
                        config.getCarriageMassKg(),
                        config.getDrumRadius(),
                        config.getMinHeight(),
                        config.getMaxHeight(),
                        true,
                        0);

        staticRoot =
                mech.getRoot(name + " 1StaticRoot", config.getInitialX(), config.getInitialY());
        staticMech2d =
                staticRoot.append(
                        new MechanismLigament2d(
                                name + " 1Static",
                                config.getStaticLength(),
                                config.getAngle(),
                                config.getLineWidth(),
                                new Color8Bit(Color.kOrange)));

        root = mech.getRoot(name + " Root", config.getInitialX(), config.getInitialY());
        m_elevatorMech2d =
                root.append(
                        new MechanismLigament2d(
                                name,
                                config.getMovingLength(),
                                config.getAngle(),
                                config.getLineWidth(),
                                new Color8Bit(Color.kBlack)));

        SimLoop.register(this::update);
    }
    /**
     * Returns the rotation per sec.
     *
     * @return the rotation per sec
     */
    private double getRotationPerSec() {
        return (elevatorSim.getVelocityMetersPerSecond() / (2 * Math.PI * config.getDrumRadius()))
                * config.getElevatorGearing();
    }
    /**
     * Returns the rotations.
     *
     * @return the rotations
     */
    private double getRotations() {
        return (elevatorSim.getPositionMeters() / (2 * Math.PI * config.getDrumRadius()))
                * config.getElevatorGearing();
    }

    /**
     * Advances the elevator physics simulation by one robot period, updates the TalonFX rotor
     * position and velocity, and refreshes both the static and moving Mechanism2d ligaments.
     */
    public void update(double dt) {
        elevatorSim.setInput(linearMotorSim.getMotorVoltage());
        elevatorSim.update(dt);

        linearMotorSim.setRotorVelocity(getRotationPerSec());
        linearMotorSim.setRawRotorPosition(getRotations());

        double displacement = elevatorSim.getPositionMeters();

        if (config.isMounted()) {
            double angle;

            if (config.getMount().getMountType() == MountType.ARM) {
                angle = config.getAngle() + Math.toDegrees(config.getMount().getAngle());
            } else if (config.getMount().getMountType() == MountType.LINEAR) {
                angle =
                        config.getAngle()
                                + Math.toDegrees(
                                        config.getMount().getAngle() - config.getInitMountAngle());
            } else {
                angle = config.getAngle();
            }

            config.setStaticRootX(getUpdatedX(config));
            config.setStaticRootY(getUpdatedY(config));

            staticRoot.setPosition(config.getStaticRootX(), config.getStaticRootY());
            root.setPosition(
                    config.getStaticRootX() + (displacement * Math.cos(Math.toRadians(angle))),
                    config.getStaticRootY() + (displacement * Math.sin(Math.toRadians(angle))));

            staticMech2d.setAngle(angle);
            m_elevatorMech2d.setAngle(angle);

        } else {
            root.setPosition(
                    config.getInitialX()
                            + (displacement * Math.cos(Math.toRadians(config.getAngle()))),
                    config.getInitialY()
                            + (displacement * Math.sin(Math.toRadians(config.getAngle()))));
        }
    }

    /**
     * Returns the horizontal component of the stage's current displacement from its initial
     * position, accounting for the parent mount's angle when mounted.
     *
     * @return horizontal displacement in metres
     */
    public double getDisplacementX() {
        double angle;

        if (!config.isMounted()) {
            angle = config.getAngle();
        } else if (config.getMount().getMountType() == MountType.ARM) {
            angle = config.getAngle() + Math.toDegrees(config.getMount().getAngle());
        } else if (config.getMount().getMountType() == MountType.LINEAR) {
            angle =
                    config.getAngle()
                            + Math.toDegrees(
                                    config.getMount().getAngle() - config.getInitMountAngle());
        } else {
            angle = config.getAngle();
        }

        return elevatorSim.getPositionMeters() * Math.cos(Math.toRadians(angle))
                + (config.getStaticRootX() - config.getInitialX());
    }

    /**
     * Returns the vertical component of the stage's current displacement from its initial position,
     * accounting for the parent mount's angle when mounted.
     *
     * @return vertical displacement in metres
     */
    public double getDisplacementY() {
        double angle;

        if (!config.isMounted()) {
            angle = config.getAngle();
        } else if (config.getMount().getMountType() == MountType.ARM) {
            angle = config.getAngle() + Math.toDegrees(config.getMount().getAngle());
        } else if (config.getMount().getMountType() == MountType.LINEAR) {
            angle =
                    config.getAngle()
                            + Math.toDegrees(
                                    config.getMount().getAngle() - config.getInitMountAngle());
        } else {
            angle = config.getAngle();
        }

        return elevatorSim.getPositionMeters() * Math.sin(Math.toRadians(angle))
                + (config.getStaticRootY() - config.getInitialY());
    }

    /**
     * Returns the effective absolute angle of the linear stage in radians, adding the parent
     * mount's angle when mounted.
     *
     * @return effective stage angle in radians
     */
    public double getAngle() {
        if (config.isMounted()) {
            return config.getMount().getAngle() + Math.toRadians(config.getAngle());
        } else {
            return Math.toRadians(config.getAngle());
        }
    }

    /**
     * Returns the X coordinate of the static root of this stage, used by child mechanisms as their
     * mount point.
     *
     * @return static root X position in metres
     */
    public double getMountX() {
        return config.getStaticRootX();
    }

    /**
     * Returns the Y coordinate of the static root of this stage, used by child mechanisms as their
     * mount point.
     *
     * @return static root Y position in metres
     */
    public double getMountY() {
        return config.getStaticRootY();
    }
}
