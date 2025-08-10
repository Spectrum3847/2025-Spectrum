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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.SpectrumCANcoderConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Shoulder extends Mechanism {

    public static class ShoulderConfig extends Config {
        @Getter @Setter private boolean isPhoton = false;

        // Positions set as degrees of rotation || 0 is vertical down
        @Getter private final int initializedPosition = 0;

        @Getter private final double scoreDelay = 0.3;
        @Getter private final double prescoreDelay = 0.3;

        /* Shoulder positions in degrees || 0 is vertical down || positions should be towards front of robot */

        @Getter @Setter private double climbPrep = 45; // 110;
        @Getter @Setter private double home = 0;

        @Getter @Setter private double stationIntake = -9.2;
        @Getter @Setter private double stationExtendedIntake = -23.6;
        @Getter @Setter private double groundAlgaeIntake = 0;
        @Getter @Setter private double groundCoralIntake = 4;

        @Getter @Setter private double processorAlgae = -143.877;
        @Getter @Setter private double l2Algae = 160; // -32;
        @Getter @Setter private double l3Algae = 160; // -32;
        @Getter @Setter private double netAlgae = 180;
        @Getter @Setter private double autonShoulderNetChecker = 60;

        @Getter @Setter private double l1Coral = 51.5;
        @Getter @Setter private double l2Coral = 15.3;
        @Getter @Setter private double l2Score = 70; // 26
        @Getter @Setter private double l3Coral = 11.1; // -8
        @Getter @Setter private double l3Score = 70; // 26;
        @Getter @Setter private double l4Coral = 170; // 158.5;
        @Getter @Setter private double l4CoralScore = 128; // 117;

        @Getter @Setter private double exL1Coral = 16.9;
        @Getter @Setter private double exL2Coral = -19.8; // -19.33; // -13.4; // -27;
        @Getter @Setter private double exL2Score = 25; // 30
        @Getter @Setter private double exL3Coral = -19.8; // -27;
        @Getter @Setter private double exL3Score = 30;
        @Getter @Setter private double exL4Coral = 193.5; // 190.3;
        @Getter @Setter private double exL4Score = 145.8; // 133;

        @Getter @Setter private double tolerance = 3.5;

        @Getter @Setter private double offset = -90;
        @Getter @Setter private double initPosition = 0;

        /* Shoulder config settings */
        @Getter @Setter private double zeroSpeed = -0.1;
        @Getter @Setter private double holdMaxSpeedRPM = 18;

        @Getter @Setter private double currentLimit = 60;
        @Getter @Setter private double torqueCurrentLimit = 80;
        @Getter @Setter private double positionKp = 1500;
        @Getter @Setter private double positionKd = 170;
        @Getter @Setter private double positionKv = 0;
        @Getter @Setter private double positionKs = 0.06;
        @Getter @Setter private double positionKa = 0.001;
        @Getter @Setter private double positionKg = 20.83333; // 12.5 * 1.666666
        @Getter @Setter private double mmCruiseVelocity = 10;
        @Getter @Setter private double mmAcceleration = 50;
        @Getter @Setter private double mmJerk = 0;
        @Getter @Setter private double slowMmAcceleration = 2;
        @Getter @Setter private double slowMmJerk = 50;

        @Getter @Setter private double sensorToMechanismRatio = 61.71428571; // 102.857;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter
        private double CANcoderRotorToSensorRatio = 61.71428571 * 1.2; // 102.857 * 1.2;
        // CANcoderSensorToMechanismRatio / sensorToMechanismRatio;

        @Getter @Setter
        private double CANcoderSensorToMechanismRatio =
                0.833333333333333333333333; // 1.2; // 30.0 / 36.0;

        @Getter @Setter private double CANcoderOffset = 0;
        @Getter @Setter private boolean CANcoderAttached = false;

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
            configGearRatio(sensorToMechanismRatio);
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
            setSimRatio(sensorToMechanismRatio);
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
    protected SpectrumCANcoderConfig canCoderConfig;
    @Getter private ShoulderSim sim;
    CANcoderSimState canCoderSim;

    public Shoulder(ShoulderConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) { // && RobotStates.pm.and(RobotStates.photon,
            // RobotStates.sim).not().getAsBoolean()) {
            if (config.isCANcoderAttached() && !Robot.isSimulation()) {
                canCoderConfig =
                        new SpectrumCANcoderConfig(
                                config.getCANcoderRotorToSensorRatio(),
                                config.getCANcoderSensorToMechanismRatio(),
                                config.getCANcoderOffset(),
                                config.isCANcoderAttached());
                canCoder =
                        new SpectrumCANcoder(
                                42,
                                canCoderConfig,
                                motor,
                                config,
                                SpectrumCANcoder.CANCoderFeedbackType.FusedCANcoder);
            }

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
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty(
                    "Position Degrees", () -> (getPositionDegrees() - config.offset), null);
            // builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("MotorVoltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
        }
    }

    void setInitialPosition() {
        if (canCoder != null) {
            if (canCoder.isAttached()
                    && canCoder.canCoderResponseOK(
                            canCoder.getCanCoder().getAbsolutePosition().getStatus())) {
                motor.setPosition(
                        canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                                / config.getCANcoderSensorToMechanismRatio());
            } else {
                motor.setPosition(
                        degreesToRotations(offsetPosition(() -> config.getInitPosition())));
            }
        } else {
            motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
        }
    }

    public Command resetToIntialPos() {
        return runOnce(this::setInitialPosition)
                .ignoringDisable(true)
                .withName("Reset to Initial position");
    }

    @Override
    public Trigger belowDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        (getPositionDegrees() - config.getOffset())
                                < (degrees.getAsDouble() - tolerance.getAsDouble()));
    }

    @Override
    public Trigger aboveDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - config.getOffset())
                                > (degrees.getAsDouble() + tolerance.getAsDouble()));
    }

    @Override
    public Trigger atDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - config.getOffset() - degrees.getAsDouble())
                                < tolerance.getAsDouble());
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

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(offsetPosition(degrees)).withName(getName() + ".runPoseDegrees");
    }

    public double checkMoveOverTop(DoubleSupplier degrees) {
        double newDeg = degrees.getAsDouble();
        if (newDeg < -90 && (getPositionDegrees() - config.offset) > 90) {
            newDeg += 360;
        } else if (newDeg > 90 && (getPositionDegrees() - config.offset) < -90) {
            newDeg -= 360;
        }
        return newDeg;
    }

    public Command move(DoubleSupplier shrinkDegrees, DoubleSupplier exDegrees) {
        return run(() -> {
                    if (!RobotStates.shrink.getAsBoolean()) {
                        setMMPositionFoc(getIfReversedOffsetInRotations(exDegrees));
                    } else {
                        setMMPositionFoc(getIfReversedOffsetInRotations(shrinkDegrees));
                    }
                })
                .withName("Shoulder.move");
    }

    public Command slowMove(DoubleSupplier degrees) {
        return run(
                () -> {
                    setDynMMPositionFoc(
                            getIfReversedOffsetInRotations(degrees),
                            () -> config.getMmCruiseVelocity(),
                            () -> config.getSlowMmAcceleration(),
                            () -> config.getSlowMmJerk());
                });
    }

    public Command move(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(getIfReversedOffsetInRotations(degrees)))
                .withName("Shoulder.move");
    }

    public DoubleSupplier getIfReversedOffsetInRotations(DoubleSupplier degrees) {
        return getOffsetRotations(getIfReversedDegrees(degrees));
    }

    public DoubleSupplier getIfReversedDegrees(DoubleSupplier degrees) {
        return () ->
                (RobotStates.reverse.getAsBoolean()
                        ? -1 * degrees.getAsDouble()
                        : degrees.getAsDouble());
    }

    public DoubleSupplier getOffsetRotations(DoubleSupplier degrees) {
        return () -> degreesToRotations(offsetPosition(() -> checkMoveOverTop(degrees)));
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
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
                                    -360,
                                    360.0 - 90.0,
                                    -90)
                            .setMount(Robot.getElevator().getSim(), true),
                    mech,
                    shoulderMotorSim,
                    "2" + config.getName()); // added 2 to the name to create it second
        }
    }
}
