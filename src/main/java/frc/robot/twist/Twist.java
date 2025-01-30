package frc.robot.twist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.Mount;
import frc.spectrumLib.sim.Mount.MountType;
import frc.spectrumLib.sim.Mountable;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Twist extends Mechanism {

    public static class TwistConfig extends Config {

        // Positions set as percentage of Twist
        @Getter private final int initializedPosition = 0;

        /* twist positions in percentage of max rotation || 0 is vertical */
        // @Getter private final double score = 100 - 65;
        // @Getter private final double home = 100 - 1;
        @Getter private final double coralLeft = 50;
        @Getter private final double algaeLeft = -50;
        @Getter private final double coralIntake = 0;
        @Getter private final double floorIntake = 0;
        @Getter private final double l1Coral = 0;
        @Getter private final double l2Algae = 50;
        @Getter private final double l3Algae = 50;
        @Getter private final double l2Coral = 50;
        @Getter private final double l3Coral = 50;
        @Getter private final double l4Coral = 50;
        @Getter private final double barge = 100;
        @Getter @Setter private double tuneTwist = 0;

        /* Twist config settings */
        @Getter private final double zeroSpeed = -0.1;

        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = .4; // 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double twistX = 0.7;
        @Getter private double twistY = 1.2;
        @Getter @Setter private double simRatio = 15.429; // TODO: Set this to actual twist ratio
        @Getter private double coralLength = 0.225;
        @Getter private double algaeLength = 0.125;

        public TwistConfig() {
            super("Twist", 44, Rio.RIO_CANBUS); // Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 73500, 80500, 0); // 147000, 161000, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-7.714285714, 7.714285714); // TODO: find minmax rotations
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
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
    private CANcoder m_CANcoder;
    @Getter private TwistSim sim;
    CANcoderSimState canCoderSim;

    public Twist(TwistConfig config) {
        super(config);
        this.config = config;

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
            builder.addDoubleProperty("Position", this::getPositionRotations, null);
            builder.addDoubleProperty("Position Percent", this::getPositionPercentage, null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
            builder.addDoubleProperty(
                    "#Tune Position Percent", config::getTuneTwist, config::setTuneTwist);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroTwistRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            m_CANcoder.setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Twist.zeroTwistRoutine");
    }

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
            public void initialize() {
                holdPosition = getPositionRotations();
            }

            @Override
            public void execute() {
                moveToRotations(() -> holdPosition);
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public boolean TwistHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    public double getClosestPosition(DoubleSupplier position1, DoubleSupplier position2) {
        double currentPosition = getPositionPercentage();
        Telemetry.print(
                "POSITION 1 DISTANCE " + Math.abs(position1.getAsDouble() - currentPosition));
        Telemetry.print(
                "POSITION 2 DISTANCE " + Math.abs(position2.getAsDouble() - currentPosition));
        if (Math.abs(position1.getAsDouble() - currentPosition)
                < Math.abs(position2.getAsDouble() - currentPosition)) {
            return position1.getAsDouble();
        } else {
            return position2.getAsDouble();
        }
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
        Color8Bit coralColor = new Color8Bit(Color.kBlue);
        Color8Bit algaeColor = new Color8Bit(Color.kAqua);
        double coralLineWeight = 12.0;
        double algaeLineWeight = 4.0;
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
                            Math.toRadians(-180),
                            Math.toRadians(180),
                            false, // Simulate gravity (change back to true)
                            0);
            this.twistMotorSim = twistMotorSim;

            root = mech.getRoot("Claw pivot", config.getTwistX(), config.getTwistY());
            leftBase =
                    root.append(
                            new MechanismLigament2d(
                                    "3LeftTwistBase",
                                    config.getCoralLength(),
                                    -90.0,
                                    4.0,
                                    coralColor));
            rightBase =
                    root.append(
                            new MechanismLigament2d(
                                    "2RightTwistBase",
                                    config.getAlgaeLength(),
                                    90.0,
                                    4.0,
                                    algaeColor));
            leftProng =
                    leftBase.append(
                            new MechanismLigament2d(
                                    "3LeftTwistProng", 0.2, 90, coralLineWeight, coralColor));
            rightProng =
                    rightBase.append(
                            new MechanismLigament2d(
                                    "2RightTwistProng", 0.2, -90, algaeLineWeight, algaeColor));

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
            rightBase.setAngle(90 + Math.toDegrees(mount.getAngle()));
            leftBase.setAngle(-90 + Math.toDegrees(mount.getAngle()));

            /* changes which side is closest to the viewer; the left prong is always closest to the viewer */
            if (getPositionPercentage() > 0) {
                leftBase.setColor(coralColor);
                leftProng.setColor(coralColor);
                rightBase.setColor(algaeColor);
                rightProng.setColor(algaeColor);
                leftProng.setLineWeight(coralLineWeight);
                rightProng.setLineWeight(algaeLineWeight);
                leftBase.setLength(
                        calculateBaseLength(config.getCoralLength(), getPositionPercentage()));
                rightBase.setLength(
                        calculateBaseLength(config.getAlgaeLength(), getPositionPercentage()));
            } else {
                leftBase.setColor(algaeColor);
                leftProng.setColor(algaeColor);
                rightBase.setColor(coralColor);
                rightProng.setColor(coralColor);
                leftProng.setLineWeight(algaeLineWeight);
                rightProng.setLineWeight(coralLineWeight);
                leftBase.setLength(
                        -1 * calculateBaseLength(config.getAlgaeLength(), getPositionPercentage()));
                rightBase.setLength(
                        -1 * calculateBaseLength(config.getCoralLength(), getPositionPercentage()));
            }
        }

        private double calculateBaseLength(double startingLength, double posePercent) {
            return startingLength * ((Math.abs(posePercent) - 50) / 50);
        }
    }
}
