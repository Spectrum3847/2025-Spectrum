package frc.robot.elbow;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class Elbow extends Mechanism {

    public static class ElbowConfig extends Config {
        /* Elbow positions in percentage of max rotation || 0 is horizontal */
        @Getter private final double home = -45;
        @Getter private final double intake = -96;

        /* Elbow config settings */
        @Getter private final double zeroSpeed = -0.1;

        @Getter @Setter private boolean shortFeed = false;
        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        // Need to add auto launching positions when auton is added

        // Removed implementation of tree map

        /* Sim properties */
        @Getter private double elbowX = 0.55;
        @Getter private double elbowY = 0.1;
        @Getter @Setter private double simRatio = 172.8; // TODO: Set this to actual elbow ratio
        @Getter private double length = 0.4;

        public ElbowConfig() {
            super("Elbow", 41, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(147000, 161000, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-33.357421875, 30.88671875);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
        }

        public ElbowConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private ElbowConfig config;
    private CANcoder m_CANcoder;
    @Getter private ElbowSim sim;
    CANcoderSimState canCoderSim;

    public Elbow(ElbowConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ElbowStates.setStates();
    }

    public void setupDefaultCommand() {
        ElbowStates.setupDefaultCommand();
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

    public void switchFeedSpot() {
        config.setShortFeed(!(config.isShortFeed()));
        Telemetry.print("Feed spot switched to " + ((config.isShortFeed()) ? " short" : " long"));
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroElbowRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            m_CANcoder.setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Elbow.zeroElbowRoutine");
    }

    /** Holds the position of the Elbow. */
    public Command runHoldElbow() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Elbow.holdPosition");
                addRequirements(Elbow.this);
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

    public boolean ElbowHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new ElbowSim(motor.getSimState(), RobotSim.leftView);

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

    class ElbowSim extends ArmSim {
        public ElbowSim(TalonFXSimState elbowMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                            config.elbowX,
                            config.elbowY,
                            config.simRatio,
                            config.length,
                            config.getMinRotations(),
                            80, // config.getMaxRotation() * config.getRatio(),
                            config.getMinRotations()),
                    mech,
                    elbowMotorSim,
                    config.getName());
        }
    }
}