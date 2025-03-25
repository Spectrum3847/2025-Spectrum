package frc.robot.twist;

import static frc.robot.RobotStates.algae;
import static frc.robot.RobotStates.coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.Mount;
import frc.spectrumLib.sim.Mount.MountType;
import frc.spectrumLib.sim.Mountable;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Twist extends Mechanism {

    public static class TwistConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        // Positions set as percentage of Twist
        @Getter private final int initializedPosition = 20;

        @Getter private final double stageDelay = 0.1;

        /* twist positions in percentage of max rotation || 0 is horizontal */

        @Getter private final double home = 0;
        @Getter private final double coralLollipop = 90;
        @Getter private final double stationIntake = 0; // 179.9;
        @Getter private final double algaeIntake = 179.9;
        @Getter private final double groundAlgaeIntake = 0;
        @Getter private final double groundCoralIntake = 179.9;
        @Getter private final double leftCoral = 90;
        @Getter private final double rightCoral = -90;
        @Getter private final double l1Coral = 0;
        @Getter private final double net = algaeIntake;
        @Getter private final double climbPrep = 179.9;

        @Getter private final double initPosition = 0;

        /* Twist config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 10;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double positionKp = 1000;
        @Getter private final double positionKd = 70;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 1.8;
        @Getter private final double positionKa = 2;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 4.2;
        @Getter private final double mmAcceleration = 32;
        @Getter private final double mmJerk = 0;

        // Need to add auto launching positions when auton is added

        /* Cancoder config settings */
        @Getter private final double CANcoderGearRatio = 1;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        /* Sim properties */
        @Getter private double twistX = 0.525;
        @Getter private double twistY = 1.35;
        @Getter @Setter private double simRatio = 22.4;
        @Getter private double coralLength = 0.255;
        @Getter private double algaeLength = 0.205;
        @Getter private double coralLineWeight = 15.0;
        @Getter private double algaeLineWeight = 4.0;
        @Getter private double coralBaseAngle = -60;
        @Getter private double algaeBaseAngle = 50;
        @Getter private Color8Bit coralColor = new Color8Bit(Color.kBlue);
        @Getter private Color8Bit algaeColor = new Color8Bit(Color.kAqua);

        public TwistConfig() {
            super("Twist", 44, Rio.CANIVORE); // Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(22.4);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-.25, 0.75);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(true);
            configClockwise_Positive();
        }

        public TwistConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private TwistConfig config;
    private SpectrumCANcoder canCoder;
    @Getter private TwistSim sim;
    CANcoderSimState canCoderSim;

    public Twist(TwistConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            canCoder =
                    new SpectrumCANcoder(44, motor, config)
                            .setGearRatio(config.getCANcoderGearRatio())
                            .setOffset(config.getCANcoderOffset())
                            .setAttached(false);

            setInitialPosition();
        }

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        TwistStates.setStates();
    }

    public void setupDefaultCommand() {
        TwistStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty("Degrees", this::getPositionDegrees, null);
            // builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
            // builder.addDoubleProperty("Front-TX", Robot.getVision().frontLL::getTagTx, null);
            // builder.addDoubleProperty("Front-TA", Robot.getVision().frontLL::getTagTA, null);
            // builder.addDoubleProperty(
            //        "Front-Rotation", Robot.getVision().frontLL::getTagRotationDegrees, null);
            // builder.addDoubleProperty(
            //        "Front-ClosestTag", Robot.getVision().frontLL::getClosestTagID, null);
        }
    }

    private void setInitialPosition() {
        if (canCoder.isAttached()) {
            motor.setPosition(
                    canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                            * config.getGearRatio());
        } else {
            motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
        }
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the Twist. */
    public Command runHoldTwist() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Twist.holdPosition");
                addRequirements(Twist.this);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }

            @Override
            public void initialize() {
                holdPosition = getPositionRotations();
                stop();
            }

            @Override
            public void execute() {
                if (Math.abs(getVelocityRPM()) > config.holdMaxSpeedRPM) {
                    stop();
                    holdPosition = getPositionRotations();
                } else {
                    setDynMMPositionFoc(
                            () -> holdPosition,
                            () -> config.getMmCruiseVelocity(),
                            () -> config.getMmAcceleration(),
                            () -> 20);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(degrees).withName(getName() + ".runPoseDegrees");
    }

    private void setDegrees(DoubleSupplier degrees) {
        setMMPositionFoc(() -> degreesToRotations(degrees));
    }

    public Command move(DoubleSupplier targetDegrees, boolean clockwise) {
        return run(
                () -> {
                    double currentDegrees = getPositionDegrees();
                    // Normalize targetDegrees to be within 0 to 360
                    double target = (targetDegrees.getAsDouble() % 360);
                    // Normalize currentDegrees to be within 0 to 360
                    double currentMod = (currentDegrees % 360);

                    double output;

                    if (clockwise) {
                        // Calculate the closest clockwise position
                        if (currentMod > target) {
                            output = currentDegrees - (currentMod - target);
                        } else {
                            output = currentDegrees - (360 + currentMod - target);
                        }
                    } else {
                        // Calculate the closest counterclockwise position
                        if (currentMod < target) {
                            output = currentDegrees + (target - currentMod);
                        } else {
                            output = currentDegrees + (360 + target - currentMod);
                        }
                    }

                    final double out = output;
                    setDegrees(() -> out);
                });
    }

    public Command move(DoubleSupplier degrees) {
        return run(
                () -> {
                    if (RobotStates.reverse.getAsBoolean()) {
                        if (degrees.getAsDouble() + 180 >= 180
                                && degrees.getAsDouble() - 179.9 >= 0) {
                            setDegrees(() -> degrees.getAsDouble() - 179.9);
                        } else {
                            setDegrees(() -> degrees.getAsDouble() + 179.9);
                        }
                    } else {
                        setDegrees(degrees);
                    }
                });
    }

    public DoubleSupplier getIfReversedDegrees(DoubleSupplier degrees) {
        return () ->
                (RobotStates.reverse.getAsBoolean())
                        ? degrees.getAsDouble() + 180
                        : degrees.getAsDouble();
    }

    public Command twistHome() {
        return run(
                () -> {
                    if (algae.getAsBoolean()) {
                        setDegrees(config::getAlgaeIntake);
                    } else if (coral.getAsBoolean()) {
                        setDegrees(config::getLeftCoral);
                    } else {
                        setDegrees(config::getHome);
                    }
                });
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new TwistSim(motor.getSimState(), RobotSim.leftView);

            // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        }
    }

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

        public TwistSim(TalonFXSimState twistMotorSim, Mechanism2d mech) {
            armSim =
                    new SingleJointedArmSim(
                            DCMotor.getKrakenX60Foc(config.getNumMotors()),
                            config.getSimRatio(),
                            1.2,
                            config.getCoralLength(), // placeholder, should really be length of
                            // forearm
                            Math.toRadians(-360),
                            Math.toRadians(360),
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

            /* changes which side is closest to the viewer; the left prong is always closest to the viewer */
            if (getPositionPercentage() > 0 && getPositionDegrees() % 360 <= 180) {
                leftBase.setColor(config.getCoralColor());
                leftProng.setColor(config.getCoralColor());
                rightBase.setColor(config.getAlgaeColor());
                rightProng.setColor(config.getAlgaeColor());
                leftBase.setLineWeight(2 * config.getCoralLineWeight() / 3);
                leftProng.setLineWeight(config.getCoralLineWeight());
                rightBase.setLineWeight(config.getAlgaeLineWeight());
                rightProng.setLineWeight(config.getAlgaeLineWeight());
                leftBase.setAngle(
                        calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
                leftProng.setAngle(
                        calculateBaseAngle(-config.getCoralBaseAngle(), getPositionPercentage()));
                rightBase.setAngle(
                        calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
                rightProng.setAngle(
                        calculateBaseAngle(-config.getAlgaeBaseAngle(), getPositionPercentage()));
                leftBase.setLength(
                        calculateBaseLength(
                                config.getCoralLength(),
                                config.getCoralBaseAngle(),
                                calculateBaseAngle(
                                        config.getCoralBaseAngle(), getPositionPercentage()),
                                getPositionPercentage()));
                rightBase.setLength(
                        calculateBaseLength(
                                config.getAlgaeLength(),
                                config.getAlgaeBaseAngle(),
                                calculateBaseAngle(
                                        config.getAlgaeBaseAngle(), getPositionPercentage()),
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
                leftBase.setAngle(
                        calculateBaseAngle(config.getAlgaeBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
                leftProng.setAngle(
                        calculateBaseAngle(-config.getAlgaeBaseAngle(), getPositionPercentage()));
                rightBase.setAngle(
                        calculateBaseAngle(config.getCoralBaseAngle(), getPositionPercentage())
                                + Math.toDegrees(mount.getAngle()));
                rightProng.setAngle(
                        calculateBaseAngle(-config.getCoralBaseAngle(), getPositionPercentage()));
                leftBase.setLength(
                        calculateBaseLength(
                                config.getAlgaeLength(),
                                config.getAlgaeBaseAngle(),
                                calculateBaseAngle(
                                        config.getAlgaeBaseAngle(), getPositionPercentage()),
                                getPositionPercentage()));
                rightBase.setLength(
                        calculateBaseLength(
                                config.getCoralLength(),
                                config.getCoralBaseAngle(),
                                calculateBaseAngle(
                                        config.getCoralBaseAngle(), getPositionPercentage()),
                                getPositionPercentage()));
            }
        }

        private double getPositionPercentage() {
            double position = getPositionDegrees() % 360;
            if (position > 180) {
                position -= 180;
                position = 180 - position;
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
}
