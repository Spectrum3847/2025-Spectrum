package frc.robot.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import lombok.*;

public class Climber extends Mechanism {

    public static class ClimberConfig extends Config {
        /* Climber positions in percentage of max rotation || 0 is horizontal */
        @Getter private final double home = 0;
        @Getter private final double prepClimber = 100;
        @Getter private final double finishClimb = 50;
        @Getter @Setter private double tuneClimber = 0;

        /* Climber config settings */
        // TODO: None of these values are even remotely close
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double velocityKp = .4; // 186; // 200 w/ 0.013 good
        @Getter private final double velocityKv = 0.018;
        @Getter private final double velocityKs = 0;

        /* Sim properties */
        @Getter private double climberX = 1.3; // 1.0;
        @Getter private double climberY = 0.35;

        @Getter @Setter
        private double simRatio = 1; // TODO: Set to number of rotations per mech revolution

        @Getter private double length = 0.4;

        public ClimberConfig() {
            super("Climber", 53, Rio.RIO_CANBUS); // Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configMotionMagic(54.6, 60, 0); // 147000, 161000, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-7.714285714, 7.714285714); // TODO: find minmax rotations
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            setSimRatio(15.429);
        }

        public ClimberConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private ClimberConfig config;
    private CANcoder m_CANcoder;
    @Getter private ClimberSim sim;
    CANcoderSimState canCoderSim;

    public Climber(ClimberConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ClimberStates.setStates();
    }

    public void setupDefaultCommand() {
        ClimberStates.setupDefaultCommand();
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
                    "#Tune Position Percent", config::getTuneClimber, config::setTuneClimber);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    // TODO: Pivot hold position run commands needed

    public Command runHoldClimber() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Climber.holdPosition");
                addRequirements(Climber.this);
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

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new ClimberSim(motor.getSimState(), RobotSim.leftView);

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

    class ClimberSim extends ArmSim {
        public ClimberSim(TalonFXSimState climberMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.climberX,
                                    config.climberY,
                                    config.simRatio,
                                    config.length,
                                    0,
                                    // 180 - 45 +
                                    // Units.rotationsToDegrees(config.getMinRotations()),
                                    180,
                                    // 180 - 45 +
                                    // Units.rotationsToDegrees(config.getMaxRotations()),
                                    180)
                            .setColor(new Color8Bit(Color.kBlueViolet)),
                    mech,
                    climberMotorSim,
                    config.getName());
        }
    }
}
