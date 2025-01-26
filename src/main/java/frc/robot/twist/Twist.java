package frc.robot.twist;

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
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import lombok.*;

public class Twist extends Mechanism {

    public static class TwistConfig extends Config {

        // Positions set as percentage of Twist
        @Getter private final int initializedPosition = 20;

        /* twist positions in percentage of max rotation || 0 is horizontal */
        // TODO: Find twist positions
        @Getter private final double score = 100 - 65;
        @Getter private final double home = 100 - 1;
        @Getter private final double intake = 100;
        @Getter private final double ninetyDegrees = 100 - 39;
        @Getter private final double l1 = 63;
        @Getter private final double l2Algae = 90;
        @Getter private final double l3Algae = 90;
        @Getter private final double l2Coral = 90;
        @Getter private final double l3Coral = 90;
        @Getter private final double l4 = 57;
        @Getter private final double barge = 57;
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
        @Getter private double twistX = 0.6 + 0.41; // 0.8;
        @Getter private double twistY = 0.4 + 0.59;
        @Getter @Setter private double simRatio = 172.8; // TODO: Set this to actual twist ratio
        @Getter private double length = 0.1;

        public TwistConfig() {
            super("Twist", 42, Rio.RIO_CANBUS); // Rio.CANIVORE); // TODO: update ID
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 73500, 80500, 0); // 147000, 161000, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(0, 132.0126953); // 66.0063476563); // .96
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
            builder.addDoubleProperty(
                    "Position Percent",
                    () -> (getPositionRotations() / config.getMaxRotations()) * 100,
                    null);
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

    class TwistSim extends ArmSim {
        public TwistSim(TalonFXSimState twistMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.twistX,
                                    config.twistY,
                                    config.simRatio,
                                    config.length,
                                    // 225 - Units.rotationsToDegrees(config.getMaxRotations()) -
                                    90,
                                    // 225 - Units.rotationsToDegrees(config.getMinRotations()) -
                                    90,
                                    -45 - 90)
                            .setMount(Robot.getElevator().getSim(), false),
                    mech,
                    twistMotorSim,
                    "3" + config.getName()); // added 3 to the name to create it third
        }
    }
}
