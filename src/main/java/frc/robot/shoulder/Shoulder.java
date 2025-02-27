package frc.robot.shoulder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Shoulder extends Mechanism {

    public static class ShoulderConfig extends Config {
        @Getter @Setter private boolean isPhoton = false;
        @Getter @Setter private boolean reversed = false;

        // Positions set as degrees of rotation || 0 is vertical down
        @Getter private final int initializedPosition = 0;

        /* Shoulder positions in degrees || 0 is vertical down || positions should be towards front of robot */
        // TODO: Find shoulder positions

        @Getter @Setter private double climbHome = 90; // TODO: Find this value
        @Getter @Setter private double home = 0;

        @Getter @Setter private double stationIntake = 0;
        @Getter @Setter private double stationExtendedIntake = 172;
        @Getter @Setter private double clawGroundAlgaeIntake = -164;
        @Getter @Setter private double clawGroundCoralIntake = -164;
        @Getter @Setter private double handOff = 180;

        @Getter @Setter private double processorAlgae = 0.0;
        @Getter @Setter private double l2Algae = 43.07;
        @Getter @Setter private double l3Algae = 43.07;
        @Getter @Setter private double netAlgae = 180;

        @Getter @Setter private double l1Coral = 14;
        @Getter @Setter private double l2Coral = 34;
        @Getter @Setter private double l2CoralScore = 34.0 - 15;
        @Getter @Setter private double l3Coral = 34;
        @Getter @Setter private double l3CoralScore = 34.0 - 15;
        @Getter @Setter private double l4Coral = -210;
        @Getter @Setter private double l4CoralScore = -210.0 + 50;

        @Getter @Setter private double tolerance = 0.95;

        @Getter @Setter private double offset = -90;
        @Getter @Setter private double initPosition = 0;

        /* Shoulder config settings */
        @Getter @Setter private double zeroSpeed = -0.1;
        @Getter @Setter private double holdMaxSpeedRPM = 18;

        @Getter @Setter private double currentLimit = 20;
        @Getter @Setter private double torqueCurrentLimit = 100;
        @Getter @Setter private double positionKp = 1500;
        @Getter @Setter private double positionKd = 140;
        @Getter @Setter private double positionKv = 0;
        @Getter @Setter private double positionKs = 0.06;
        @Getter @Setter private double positionKa = 0.001;
        @Getter @Setter private double positionKg = 12.5;
        @Getter @Setter private double mmCruiseVelocity = 10;
        @Getter @Setter private double mmAcceleration = 50;
        @Getter @Setter private double mmJerk = 0;

        /* Cancoder config settings */
        @Getter private final double CANcoderGearRatio = 30.0 / 36.0;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        /* Sim properties */
        @Getter private double shoulderX = 0.8;
        @Getter private double shoulderY = 1.1;

        @Getter @Setter private double simRatio = 1;

        @Getter private double length = 0.3;

        public ShoulderConfig() {
            super("Shoulder", 42, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(102.857);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(-1, 0.5);
            configReverseSoftLimit(-1, true);
            configForwardSoftLimit(0.5, true);
            configNeutralBrakeMode(true);
            if (Robot.isSimulation()) {
                configCounterClockwise_Positive();
            } else {
                configClockwise_Positive();
            }
            configGravityType(true);
            setSimRatio(102.857);
        }

        public ShoulderConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    protected ShoulderConfig config;
    protected SpectrumCANcoder canCoder;
    @Getter private ShoulderSim sim;
    CANcoderSimState canCoderSim;

    public Shoulder(ShoulderConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            // canCoder =
            //         new SpectrumCANcoder(42, motor, config)
            //                 .setGearRatio(config.getCANcoderGearRatio())
            //                 .setOffset(config.getCANcoderOffset())
            //                 .setAttached(false);
            setInitialPosition();
        }

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ShoulderStates.setStates();
    }

    public void setupDefaultCommand() {
        ShoulderStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty(
                    "Position Degrees", () -> (this.getPositionDegrees() - config.offset), null);
            builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty(
                    "Motor Voltage", this.motor.getSimState()::getMotorVoltage, null);
        }
    }

    void setInitialPosition() {
        // if (canCoder != null) {
        //     if (canCoder.isAttached()) {
        //         motor.setPosition(
        //                 canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
        //                         * config.getGearRatio());
        //     }
        // } else {
        motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
        // }
    }

    public Command resetToIntialPos() {
        return runOnce(this::setInitialPosition)
                .ignoringDisable(true)
                .withName("Reset to Initial position");
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroShoulderRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            canCoder.getCanCoder().setPosition(0);
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Shoulder.zeroShoulderRoutine");
    }

    /** Holds the position of the Shoulder. */
    public Command runHoldShoulder() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Shoulder.holdPosition");
                addRequirements(Robot.getShoulder());
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

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public boolean ShoulderHasError() {
        if (isAttached()) {
            return getPositionRotations() > config.getMaxRotations();
        }
        return false;
    }

    public double checkReversed(DoubleSupplier position) {
        if (!config.isReversed()) {
            return position.getAsDouble();
        }

        return position.getAsDouble() * -1;
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(offsetPosition(degrees)).withName(getName() + ".runPoseDegrees");
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
    }

    public Command moveToDegreesAndCheckReversed(DoubleSupplier degrees) {
        return moveToDegrees(() -> checkReversed(degrees));
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    void simulationInit() {
        if (isAttached()) {
            sim = new ShoulderSim(motor.getSimState(), RobotSim.leftView);
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

    class ShoulderSim extends ArmSim {
        public ShoulderSim(TalonFXSimState shoulderMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.shoulderX,
                                    config.shoulderY,
                                    config.simRatio,
                                    config.length,
                                    -270,
                                    360 - 90,
                                    -90)
                            .setMount(Robot.getElevator().getSim(), true),
                    mech,
                    shoulderMotorSim,
                    "2" + config.getName()); // added 2 to the name to create it second
        }
    }
}
