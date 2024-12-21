package frc.robot.pivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import lombok.*;

public class Pivot extends Mechanism {

    public static class PivotConfig extends Config {
        /* Pivot positions in percentage of max rotation || 0 is horizontal */
        @Getter private final double score = 65;
        @Getter private final double climbHome = 3;
        @Getter private final double home = 1;
        @Getter private final double subwoofer = 81;
        @Getter private final double intoAmp = 78;
        @Getter private final double podium = 53.5;
        @Getter private final double fromAmp = 52;
        @Getter private final double ampWing = 41;
        @Getter private final double intake = 50;
        @Getter private final double manualFeed = 70;

        /* Pivot config settings */
        @Getter private final double zeroSpeed = -0.1;
        /**
         * Percentage of pivot rotation added/removed from vision launching pivot angles (percentage
         * of the CHANGE in angle you set to, not +- the angle you set to) (the actual offset to
         * angles gets bigger as you get farther away)
         */
        @Getter private final double STARTING_OFFSET = 3;

        @Getter @Setter private double OFFSET = STARTING_OFFSET; // do not change this line
        @Getter @Setter private boolean shortFeed = false;
        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        /* Cancoder config settings */
        @Getter private final int CANcoderID = 44;
        @Getter private final double CANcoderGearRatio = -35.1;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        private enum CANCoderFeedbackType {
            RemoteCANcoder,
            FusedCANcoder,
            SyncCANcoder,
        }

        private CANCoderFeedbackType pivotFeedbackSource = CANCoderFeedbackType.FusedCANcoder;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double pivotX = 0.55;
        @Getter private double pivotY = 0.1;
        @Getter @Setter private double ratio = 1; // 15;
        @Getter private double length = 0.4;

        public PivotConfig() {
            super("Pivot", 41, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(147000, 161000, 0);
            configGearRatio(1); // 172.8);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(0, 30); // .96
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive(); // might be different on actual robot
            setRatio(Math.abs(172.8)); // getGearRatio()));
        }

        public void configCANcoderOffset(double CANcoderOffset) {
            this.CANcoderOffset = CANcoderOffset;
        }

        public PivotConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();
            talonConfigMod.Feedback.FeedbackRemoteSensorID = CANcoderID;
            switch (pivotFeedbackSource) {
                case RemoteCANcoder:
                    talonConfigMod.Feedback.FeedbackSensorSource =
                            FeedbackSensorSourceValue.RemoteCANcoder;
                    break;
                case FusedCANcoder:
                    talonConfigMod.Feedback.FeedbackSensorSource =
                            FeedbackSensorSourceValue.FusedCANcoder;
                    break;
                case SyncCANcoder:
                    talonConfigMod.Feedback.FeedbackSensorSource =
                            FeedbackSensorSourceValue.SyncCANcoder;
                    break;
            }
            talonConfigMod.Feedback.RotorToSensorRatio = CANcoderGearRatio;
            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private PivotConfig config;
    private CANcoder m_CANcoder;
    @Getter private PivotSim sim;
    CANcoderSimState canCoderSim;

    public Pivot(PivotConfig config) {
        super(config);
        this.config = config;

        if (config.isCANcoderAttached()) {
            config.modifyMotorConfig(motor); // Modify configuration to use remote CANcoder fused
            m_CANcoder = new CANcoder(config.getCANcoderID(), Rio.CANIVORE);
            CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
            cancoderConfigs.MagnetSensor.MagnetOffset = config.getCANcoderOffset();
            cancoderConfigs.MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive;
            cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1);
            checkCANcoderResponse(m_CANcoder.getConfigurator().apply(cancoderConfigs));
        }

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        PivotStates.setStates();
    }

    public void setupDefaultCommand() {
        PivotStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position", this::getPositionRotations, null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
        }
    }

    // Removed tree map getters

    public void increaseOffset() {
        increaseOffset(1);
    }

    public void decreaseOffset() {
        decreaseOffset(1);
    }

    public void increaseOffset(double amount) {
        config.setOFFSET(config.getOFFSET() + amount);
        Telemetry.print("Pivot offset increased to: " + config.getOFFSET());
    }

    public void decreaseOffset(double amount) {
        config.setOFFSET(config.getOFFSET() - amount);
        Telemetry.print("Pivot offset decreased to: " + config.getOFFSET());
    }

    public void resetOffset() {
        config.setOFFSET(config.getSTARTING_OFFSET());
        Telemetry.print("Pivot offset reset to: " + config.getOFFSET());
    }

    public void switchFeedSpot() {
        config.setShortFeed(!(config.isShortFeed()));
        Telemetry.print("Feed spot switched to " + ((config.isShortFeed()) ? " short" : " long"));
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroPivotRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        (b) -> {
                            m_CANcoder.setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Pivot.zeroPivotRoutine");
    }

    /** Holds the position of the pivot. */
    public Command runHoldPivot() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Pivot.holdPosition");
                addRequirements(Pivot.this);
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

    public boolean pivotHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    public void checkCANcoderResponse(StatusCode response) {
        if (!response.isOK()) {
            Telemetry.print(
                    "Pivot CANcoder ID "
                            + config.getCANcoderID()
                            + " failed config with error "
                            + response.toString());
        }
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new PivotSim(motor.getSimState(), RobotSim.leftView);

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

    class PivotSim extends ArmSim {
        public PivotSim(TalonFXSimState pivotMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                            config.pivotX,
                            config.pivotY,
                            config.ratio,
                            config.length,
                            config.getMinRotations(),
                            80, // config.getMaxRotation() * config.getRatio(),
                            config.getMinRotations()),
                    mech,
                    pivotMotorSim,
                    config.getName());
        }
    }
}
