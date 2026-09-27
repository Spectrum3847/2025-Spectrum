package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import lombok.Getter;

/**
 * WPILib-backed simulation of a single-jointed arm driven by one or more Kraken X60 motors. Updates
 * the TalonFX sim state each robot period and animates the arm in a {@link Mechanism2d} canvas.
 * Implements {@link Mount} so other mechanisms can be attached to this arm's tip, and implements
 * {@link Mountable} so this arm can itself be attached to a parent mount.
 */
public class ArmSim implements Mount, Mountable {
    private SingleJointedArmSim armSim;
    /** Configuration containing physical properties and display settings for this arm. */
    @Getter private ArmConfig config;

    private MechanismRoot2d armPivot;
    private MechanismLigament2d armMech2d;
    private TalonFXSimState armMotorSim;

    /** Always {@link MountType#ARM}; used by child mechanisms to determine positioning logic. */
    @Getter private final MountType mountType = MountType.ARM;

    /**
     * Creates and registers an arm simulation.
     *
     * @param config physical and display configuration for the arm
     * @param mech the Mechanism2d canvas to draw the arm on
     * @param armMotorSim the TalonFX sim state of the motor driving the arm
     * @param name unique name prefix used for Mechanism2d element labels
     */
    public ArmSim(ArmConfig config, Mechanism2d mech, TalonFX motor, String name) {
        this.config = config;
        this.armMotorSim = SimMotor.simState(motor, config.isReversedLinkage());
        armSim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(config.getNumMotors()),
                        config.getRatio(),
                        config.getSimMOI(),
                        config.getSimCGLength(),
                        config.getMinAngle(),
                        config.getMaxAngle(),
                        config.isSimulateGravity(),
                        config.getStartingAngle());

        armPivot = mech.getRoot(name + " Arm Pivot", config.getPivotX(), config.getPivotY());
        armMech2d =
                armPivot.append(
                        new MechanismLigament2d(
                                name + " Arm",
                                config.getLength(),
                                config.getMinAngle(),
                                5.0,
                                config.getColor()));

        SimLoop.register(this::update);
    }

    /**
     * Advances the arm physics simulation by one robot period, updates the TalonFX rotor position
     * and velocity, and refreshes the Mechanism2d visualization.
     */
    public void update(double dt) {
        armSim.setInput(armMotorSim.getMotorVoltage());
        armSim.update(dt);

        armMotorSim.setRawRotorPosition(
                (Units.radiansToRotations(armSim.getAngleRads() - config.getStartingAngle()))
                        * config.getRatio());

        armMotorSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec()) * config.getRatio());

        if (config.isMounted()) {
            config.setPivotX(getUpdatedX(config));
            config.setPivotY(getUpdatedY(config));
            if (config.isAbsAngle()) {
                armMech2d.setAngle(Math.toDegrees(armSim.getAngleRads()));
            } else {
                armMech2d.setAngle(
                        Math.toDegrees(armSim.getAngleRads())
                                + Math.toDegrees(config.getMount().getAngle()));
            }
        } else {
            armMech2d.setAngle(Math.toDegrees(armSim.getAngleRads()));
        }

        armPivot.setPosition(config.getPivotX(), config.getPivotY());
    }

    /**
     * Returns the current arm angle from the WPILib physics simulation.
     *
     * @return arm angle in radians
     */
    public double getAngleRads() {
        return armSim.getAngleRads();
    }

    /**
     * Returns how far the pivot has moved horizontally from its initial position.
     *
     * @return horizontal displacement in metres
     */
    public double getDisplacementX() {
        return config.getPivotX() - config.getInitialX();
    }

    /**
     * Returns how far the pivot has moved vertically from its initial position.
     *
     * @return vertical displacement in metres
     */
    public double getDisplacementY() {
        return config.getPivotY() - config.getInitialY();
    }

    /**
     * Returns the effective arm angle accounting for the parent mount's angle when mounted and not
     * using an absolute angle reference.
     *
     * @return effective arm angle in radians
     */
    public double getAngle() {
        if (config.isMounted()) {
            if (config.isAbsAngle()) {
                return getAngleRads();
            } else {
                return getAngleRads() + config.getMount().getAngle();
            }
        }
        return getAngleRads();
    }

    /**
     * Returns the X coordinate of the arm pivot, used by child mechanisms as their mount point.
     *
     * @return pivot X position in metres
     */
    public double getMountX() {
        return config.getPivotX();
    }

    /**
     * Returns the Y coordinate of the arm pivot, used by child mechanisms as their mount point.
     *
     * @return pivot Y position in metres
     */
    public double getMountY() {
        return config.getPivotY();
    }
}
