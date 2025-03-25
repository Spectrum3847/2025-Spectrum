package frc.robot.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumServo;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Climb extends Mechanism {

    public static class ClimbConfig extends Config {

        @Getter private final double maxRotations = 0.335; // 0.36;
        @Getter private final double minRotations = -0.06;
        /* Climb positions in degrees || 0 is horizontal */
        @Getter private final double home = 90;
        @Getter private final double intake = 0;
        @Getter private final double algaeFloorIntake = 30;
        @Getter private final double prepClimber = 0;
        @Getter private final double finishClimb = 100;
        @Getter private final double coralFloorIntake = -10;
        @Getter private final double processorScore = 60;
        @Getter private final double latchOpen = 1;
        @Getter private final double latchClosed = 0;

        @Getter private final double offsetConstant = -90;

        /* Climb config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 60; // 60
        @Getter private final double torqueCurrentLimit = 180; // 180
        @Getter private final double positionKp = 190;
        @Getter private final double positionKd = 40;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.3;
        @Getter private final double positionKa = 0.001;
        @Getter private final double positionKg = 2.9;
        @Getter private final double mmCruiseVelocity = 1;
        @Getter private final double mmAcceleration = 10;
        @Getter private final double mmJerk = 0;

        /* Sim properties */
        @Getter private double climbX = 0.95; // 1.0;
        @Getter private double climbY = 0.55;

        @Getter @Setter private double simRatio = 74.6666666667;

        @Getter private double length = 0.4;

        public ClimbConfig() {
            super("ClimbTop", 55, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(99.5555555555); // 9t = 99.5555555555); 12t - 74.6666666667;
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(getMinRotations(), getMaxRotations());
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configGravityType(true);
            setSimRatio(simRatio);
            setFollowerConfigs(new FollowerConfig("ClimbBottom", 56, Rio.CANIVORE, false));
        }

        public ClimbConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private ClimbConfig config;
    private SpectrumServo latchServo = new SpectrumServo(9);
    @Getter private SpectrumState latched = new SpectrumState("ClimbLatched");
    @Getter private ClimbSim sim;

    public Climb(ClimbConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();
        setLatchOpen();

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ClimbStates.setStates();
    }

    public void setupDefaultCommand() {
        ClimbStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty(
                    "Position Rotations", () -> (this.getPositionRotations()), null);
            builder.addDoubleProperty("Position Degrees", () -> (this.getPositionDegrees()), null);
            // builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
        }
    }

    private void setInitialPosition() {
        if (config.isAttached()) {
            motor.setPosition(0.25);
            followerMotors[0].setPosition(0.25);
        }
    }

    private void setLatchOpen() {
        latchServo.set(config.getLatchOpen());
    }

    private void setLatchClosed() {
        latchServo.set(config.getLatchClosed());
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command resetToInitialPos() {
        return runOnce(this::setInitialPosition)
                .ignoringDisable(true)
                .withName("Climb.ResetToInitialPos");
    }

    public Command runHoldClimb() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Climb.holdPosition");
                addRequirements(Climb.this);
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

    public Command zeroClimbRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Climb.zeroRoutine");
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(degrees).withName(getName() + ".runPoseDegrees");
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffsetConstant());
    }

    public Command openLatch() {
        return new RunCommand(this::setLatchOpen, latchServo)
                .alongWith(latched.setFalse())
                .withName("ClimbLatch.openLatch");
    }

    public Command closeLatch() {
        return new RunCommand(this::setLatchClosed, latchServo)
                .alongWith(latched.setTrue())
                .withName("ClimbLatch.closeLatch");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new ClimbSim(motor.getSimState(), RobotSim.frontView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class ClimbSim extends ArmSim {
        public ClimbSim(TalonFXSimState climbMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.climbX,
                                    config.climbY,
                                    config.simRatio,
                                    config.length,
                                    -30,
                                    180,
                                    90)
                            .setColor(new Color8Bit(Color.kBrown)),
                    mech,
                    climbMotorSim,
                    config.getName());
        }
    }
}
