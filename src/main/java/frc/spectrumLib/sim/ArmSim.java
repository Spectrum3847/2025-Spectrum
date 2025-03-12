package frc.spectrumLib.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import lombok.Getter;

public class ArmSim implements Mount, Mountable {
    private SingleJointedArmSim armSim;
    @Getter private ArmConfig config;

    private MechanismRoot2d armPivot;
    private MechanismLigament2d armMech2d;
    private TalonFXSimState armMotorSim;

    @Getter private final MountType mountType = MountType.ARM;

    public ArmSim(ArmConfig config, Mechanism2d mech, TalonFXSimState armMotorSim, String name) {
        this.config = config;
        this.armMotorSim = armMotorSim;
        armSim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(config.getNumMotors()),
                        config.getRatio(),
                        config.getSimMOI(),
                        config.getSimCGLength(),
                        config.getMinAngle(),
                        config.getMaxAngle(),
                        false, // Simulate gravity (change back to true)
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
    }

    public void simulationPeriodic() {
        // armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        armSim.setInput(armMotorSim.getMotorVoltage());
        armSim.update(TimedRobot.kDefaultPeriod);

        // armMotorSim.setRawRotorPosition(
        //         (armSim.getAngleRads() - config.getStartingAngle())
        //                 * config.getRatio()
        //                 / (2.0 * Math.PI));

        // armMotorSim.setRotorVelocity(
        //         armSim.getVelocityRadPerSec() * config.getRatio() / (2.0 * Math.PI));
        armMotorSim.setRawRotorPosition(
                (Units.radiansToRotations(armSim.getAngleRads() - config.getStartingAngle()))
                        * config.getRatio());

        armMotorSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec()) * config.getRatio());

        // ------ Update viz based on sim
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

    public double getAngleRads() {
        return armSim.getAngleRads();
    }

    public double getDisplacementX() {
        return config.getPivotX() - config.getInitialX();
    }

    public double getDisplacementY() {
        return config.getPivotY() - config.getInitialY();
    }

    public double getAngle() {
        if (config.isMounted()) {
            if (config.isAbsAngle()) {
                return getAngleRads(); // + config.getMount().getAngle();
            } else {
                return getAngleRads() + config.getMount().getAngle();
            }
        }
        return getAngleRads();
    }

    public double getMountX() {
        return config.getPivotX();
    }

    public double getMountY() {
        return config.getPivotY();
    }
}
