package frc.robot.elbow;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Elbow extends Mechanism {

    public static class ElbowConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        @Getter private final double handAlgae = 0;
        @Getter private final double home = 180;
        @Getter private final double clearElevator = -125;
        @Getter private final double climbPrep = 67;

        @Getter private final double scoreDelay = 0.2;

        @Getter private final double stationIntake = 158.7;
        @Getter private final double stationExtendedIntake = 154.4;
        @Getter private final double groundAlgaeIntake = -80;
        @Getter private final double groundCoralIntake = -75;

        @Getter private final double stage = -160;
        @Getter private final double l1Coral = -121.4;
        @Getter private final double l2Coral = -118;
        @Getter private final double l2Score = -108;
        @Getter private final double l3Coral = -124; // -144.5;
        @Getter private final double l3Score = -109; // -130;
        @Getter private final double l4Coral = -133; // -128;
        @Getter private final double l4Score = -104; // -107.6;

        @Getter private final double exL1Coral = -130.6;
        @Getter private final double exL2Coral = -118;
        @Getter private final double exL2Score = -109.6; // -127;
        @Getter private final double exL3Coral = -115.6; // -143;
        @Getter private final double exL3Score = -106.4; // -127;
        @Getter private final double exL4Coral = -132; // -126;
        @Getter private final double exL4Score = -106.9; // -104;

        @Getter private final double processorAlgae = -86;
        @Getter private final double l2Algae = -86;
        @Getter private final double l3Algae = -86;
        @Getter private final double net = -170;

        @Getter private final double tolerance = 0.95;

        @Getter private final double offset = -90;
        @Getter private final double initPosition = 180;

        /* Elbow config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 20;
        @Getter private final double torqueCurrentLimit = 60;
        @Getter private final double positionKp = 1400;
        @Getter private final double positionKd = 160;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.4;
        @Getter private final double positionKa = 0.002;
        @Getter private final double positionKg = 7;
        @Getter private final double mmCruiseVelocity = 10;
        @Getter private final double mmAcceleration = 50;
        @Getter private final double mmJerk = 0;

        /* Cancoder config settings */
        @Getter private final double CANcoderGearRatio = 30.0 / 36.0;
        @Getter private double CANcoderOffset = 0;
        @Getter private boolean isCANcoderAttached = false;

        /* Sim properties */
        @Getter private double elbowX = 0.8;
        @Getter private double elbowY = 0.8;
        @Getter @Setter private double simRatio = 1;
        @Getter private double length = 0.4;
        @Getter private double startingAngle = 90.0;

        public ElbowConfig() {
            super("Elbow", 43, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(102.857);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(0, 0.6);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            if (Robot.isSimulation()) {
                configCounterClockwise_Positive();
            } else {
                configClockwise_Positive();
            }
            configGravityType(true);
            setSimRatio(102.857);
        }

        public ElbowConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    @Getter private ElbowConfig config;
    private SpectrumCANcoder canCoder;
    @Getter private ElbowSim sim;
    CANcoderSimState canCoderSim;

    public Elbow(ElbowConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            canCoder =
                    new SpectrumCANcoder(43, motor, config)
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
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty("Position Degrees", () -> (getPositionWithNegative()), null);
            builder.addDoubleProperty("MotorVoltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
        }
    }

    private void setInitialPosition() {
        if (canCoder.isAttached()) {
            motor.setPosition(
                    canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                            * config.getGearRatio());
        } else {
            motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
        }
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    public double getPositionWithNegative() {
        double deg = getPositionDegrees() - config.getOffset();
        if (deg > 180) {
            return deg - 360;
        } else {
            return deg;
        }
    }

    @Override
    public Trigger belowDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        (getPositionDegrees() + config.getOffset())
                                < (degrees.getAsDouble() - tolerance.getAsDouble()));
    }

    @Override
    public Trigger aboveDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        (getPositionDegrees() + config.getOffset())
                                > (degrees.getAsDouble() + tolerance.getAsDouble()));
    }

    @Override
    public Trigger atDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() + config.getOffset() - degrees.getAsDouble())
                                < tolerance.getAsDouble());
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command zeroElbowRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            canCoder.getCanCoder().setPosition(0);
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
                stop();
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
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
        return super.moveToDegrees(offsetPosition(() -> checkNegative(degrees)))
                .withName(getName() + ".runPoseDegrees");
    }

    public double checkNegative(DoubleSupplier degrees) {
        double newDeg = degrees.getAsDouble();
        if (newDeg < 0) {
            newDeg += 360;
        }
        return newDeg;
    }

    public Command move(DoubleSupplier degrees, DoubleSupplier exDegrees) {
        return run(
                () -> {
                    // TODO: add a check for reversed and negate values when we do double sided
                    // scoring.
                    if (RobotStates.extended.getAsBoolean()) {
                        setMMPositionFoc(
                                () ->
                                        degreesToRotations(
                                                offsetPosition(() -> checkNegative(exDegrees))));
                    } else {
                        setMMPositionFoc(
                                () ->
                                        degreesToRotations(
                                                offsetPosition(() -> checkNegative(degrees))));
                    }
                });
    }

    public Command moveToMotorPosition(DoubleSupplier position) {
        return run(() -> setMMPositionFoc(position));
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
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
                                    90.0 - 360.0,
                                    360.0 - 90.0,
                                    180 - config.getStartingAngle())
                            .setColor(new Color8Bit(Color.kAqua))
                            .setMount(Robot.getShoulder().getSim(), true),
                    mech,
                    elbowMotorSim,
                    "3" + config.getName()); // added 3 to the name to create it third
        }
    }
}
