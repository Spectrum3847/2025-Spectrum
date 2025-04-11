package frc.robot.twist;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Robot;
import frc.robot.twist.Twist.TwistConfig;
import frc.spectrumLib.sim.Mount;
import frc.spectrumLib.sim.Mount.MountType;
import frc.spectrumLib.sim.Mountable;

class TwistSim implements Mountable {
    private SingleJointedArmSim armSim;
    private TalonFXSimState twistMotorSim;
    MechanismRoot2d root;
    MechanismLigament2d leftBase;
    MechanismLigament2d leftProng;
    MechanismLigament2d rightBase;
    MechanismLigament2d rightProng;
    Mount mount;
    double initMountX;
    double initMountY;
    double initMountAngle;

    private static Twist twist;
    private static TwistConfig config;

    public TwistSim(TalonFXSimState twistMotorSim, Mechanism2d mech, Twist twist) {
        TwistSim.twist = twist;
        TwistSim.config = twist.getConfig();

        armSim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(config.getNumMotors()),
                        config.getSimRatio(),
                        0.1,
                        0.1,
                        -100000000.0, // Math.toRadians(-360),
                        100000000.0, // Math.toRadians(360),
                        false, // Simulate gravity (change back to true)
                        0);
        this.twistMotorSim = twistMotorSim;

        root = mech.getRoot("Claw pivot", config.getTwistX(), config.getTwistY());
        leftBase =
                root.append(
                        new MechanismLigament2d(
                                "3LeftTwistBase",
                                config.getCoralLength(),
                                config.getCoralBaseAngle(),
                                config.getCoralLineWeight() / 2,
                                config.getCoralColor()));
        rightBase =
                root.append(
                        new MechanismLigament2d(
                                "2RightTwistBase",
                                config.getAlgaeLength(),
                                config.getAlgaeBaseAngle(),
                                config.getAlgaeLineWeight(),
                                config.getAlgaeColor()));
        leftProng =
                leftBase.append(
                        new MechanismLigament2d(
                                "3LeftTwistProng",
                                0.075,
                                -config.getCoralBaseAngle(),
                                config.getCoralLineWeight(),
                                config.getCoralColor()));
        rightProng =
                rightBase.append(
                        new MechanismLigament2d(
                                "2RightTwistProng",
                                0.075,
                                -config.getAlgaeBaseAngle(),
                                config.getAlgaeLineWeight(),
                                config.getAlgaeColor()));

        mount = Robot.getElbow().getSim();
        initMountX = Robot.getElbow().getConfig().getElbowX();
        initMountY = Robot.getElbow().getConfig().getElbowY();
        initMountAngle = Robot.getElbow().getConfig().getStartingAngle();
    }

    public void simulationPeriodic() {
        armSim.setInput(twistMotorSim.getMotorVoltage());
        armSim.update(TimedRobot.kDefaultPeriod);
        twistMotorSim.setRawRotorPosition(
                (Units.radiansToRotations(armSim.getAngleRads() - 0)) * config.getSimRatio());
        twistMotorSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec()) * config.getSimRatio());

        /* updates position and angle based on elbow */
        if (mount != null) {
            root.setPosition(
                    getUpdatedX(
                            MountType.ARM,
                            config.getTwistX(),
                            config.getTwistY(),
                            initMountX,
                            initMountY,
                            initMountAngle,
                            mount.getMountX(),
                            mount.getMountY(),
                            mount.getDisplacementX(),
                            mount.getDisplacementY(),
                            mount.getAngle()),
                    getUpdatedY(
                            MountType.ARM,
                            config.getTwistX(),
                            config.getTwistY(),
                            initMountX,
                            initMountY,
                            initMountAngle,
                            mount.getMountX(),
                            mount.getMountY(),
                            mount.getDisplacementX(),
                            mount.getDisplacementY(),
                            mount.getAngle()));
        }

        /* changes which side is closest to the viewer; the left prong is always closest to the
        viewer */
        if (getPositionPercentage() > 0 && twist.getPositionDegrees() % 360 <= 180) {
            leftBase.setColor(config.getCoralColor());
            leftProng.setColor(config.getCoralColor());
            rightBase.setColor(config.getAlgaeColor());
            rightProng.setColor(config.getAlgaeColor());
            leftBase.setLineWeight(2 * config.getCoralLineWeight() / 3);
            leftProng.setLineWeight(config.getCoralLineWeight());
            rightBase.setLineWeight(config.getAlgaeLineWeight());
            rightProng.setLineWeight(config.getAlgaeLineWeight());
            if (mount != null) {
                leftBase.setAngle(
                        calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
            } else {
                leftBase.setAngle(
                        calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage()));
            }
            leftProng.setAngle(
                    calculateBaseAngle(-config.getCoralBaseAngle(), getPositionPercentage()));
            if (mount != null) {
                rightBase.setAngle(
                        calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
            } else {
                rightBase.setAngle(
                        calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage()));
            }
            rightProng.setAngle(
                    calculateBaseAngle(-config.getAlgaeBaseAngle(), getPositionPercentage()));
            leftBase.setLength(
                    calculateBaseLength(
                            config.getCoralLength(),
                            config.getCoralBaseAngle(),
                            calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage()),
                            getPositionPercentage()));
            rightBase.setLength(
                    calculateBaseLength(
                            config.getAlgaeLength(),
                            config.getAlgaeBaseAngle(),
                            calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage()),
                            getPositionPercentage()));
        } else {
            leftBase.setColor(config.getAlgaeColor());
            leftProng.setColor(config.getAlgaeColor());
            rightBase.setColor(config.getCoralColor());
            rightProng.setColor(config.getCoralColor());
            leftBase.setLineWeight(config.getAlgaeLineWeight());
            leftProng.setLineWeight(config.getAlgaeLineWeight());
            rightBase.setLineWeight(2 * config.getCoralLineWeight() / 3);
            rightProng.setLineWeight(config.getCoralLineWeight());
            if (mount != null) {
                leftBase.setAngle(
                        calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
            } else {
                leftBase.setAngle(
                        calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage()));
            }
            leftProng.setAngle(
                    calculateBaseAngle(-config.getAlgaeBaseAngle(), getPositionPercentage()));
            if (mount != null) {
                rightBase.setAngle(
                        calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
            } else {
                rightBase.setAngle(
                        calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage()));
            }
            rightProng.setAngle(
                    calculateBaseAngle(-config.getCoralBaseAngle(), getPositionPercentage()));
            leftBase.setLength(
                    calculateBaseLength(
                            config.getAlgaeLength(),
                            config.getAlgaeBaseAngle(),
                            calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage()),
                            getPositionPercentage()));
            rightBase.setLength(
                    calculateBaseLength(
                            config.getCoralLength(),
                            config.getCoralBaseAngle(),
                            calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage()),
                            getPositionPercentage()));
        }
    }

    private double getPositionPercentage() {
        double position = twist.getPositionDegrees() % 360;
        if (position > 180) {
            position -= 180;
            position = 180 - position;
        } else if (position < -180) {
            position += 180;
            position = 180 + position;
        }

        return (position / 180) * 100;
    }

    private double calculateBaseAngle(double startingAngle, double posePercent) {
        return startingAngle * ((Math.abs(posePercent) - 50) / 50);
    }

    @SuppressWarnings("unused ")
    private double calculateBaseLength(
            double startingLength, double startingAngle, double angle, double posePercent) {
        double startingVerticalLeg =
                Math.cos(Units.degreesToRadians(startingAngle)) * startingLength;
        return startingVerticalLeg / Math.cos(Units.degreesToRadians(angle));
    }
}
