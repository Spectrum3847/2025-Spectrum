package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.CachedDouble;
import frc.spectrumLib.SpectrumRobot;
import frc.spectrumLib.SpectrumSubsystem;
import frc.spectrumLib.talonFX.TalonFXFactory;
import frc.spectrumLib.util.CanDeviceId;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import lombok.*;

/**
 * Control Modes Docs:
 * https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html
 * Closed-loop & Motion Magic Docs:
 * https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
 */
public abstract class Mechanism implements NTSendable, SpectrumSubsystem {
    @Getter protected TalonFX motor;
    @Getter protected TalonFX[] followerMotors;
    public Config config;

    Alert currentAlert = new Alert("", AlertType.kWarning);
    private double target = 0;

    private final CachedDouble cachedRotations;
    private final CachedDouble cachedPercentage;
    private final CachedDouble cachedVoltage;
    private final CachedDouble cachedDegrees;
    private final CachedDouble cachedVelocity;
    private final CachedDouble cachedCurrent;

    protected Mechanism(Config config) {
        this.config = config;

        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);

            followerMotors = new TalonFX[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                followerMotors[i] =
                        TalonFXFactory.createPermanentFollowerTalon(
                                config.followerConfigs[i].id,
                                motor,
                                config.followerConfigs[i].opposeLeader);
            }
        }

        cachedCurrent = new CachedDouble(this::updateCurrent);
        cachedVoltage = new CachedDouble(this::updateVoltage);
        cachedRotations = new CachedDouble(this::updatePositionRotations);
        cachedPercentage = new CachedDouble(this::updatePositionPercentage);
        cachedDegrees = new CachedDouble(this::updatePositionDegrees);
        cachedVelocity = new CachedDouble(this::updateVelocityRPM);

        SpectrumRobot.add(this);
        this.register();
    }

    protected Mechanism(Config config, boolean attached) {
        this(config);
        config.attached = attached;
    }

    // Setup the telemetry values, has to be called at the end of the implemented mechanism
    // constructor
    public void telemetryInit() {
        SendableRegistry.add(this, getName());
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public String getName() {
        return config.getName();
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType(getName());
    }

    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }

        return "none";
    }

    public Trigger runningDefaultCommand() {
        return new Trigger(this::isRunningDefaultCommand);
    }

    private boolean isRunningDefaultCommand() {
        return this.getCurrentCommand() == this.getDefaultCommand();
    }

    // Return the closed loop target we have sent to the motor.
    public double getTarget() {
        return target;
    }

    public Trigger atTargetPosition(DoubleSupplier tolerance) {
        return new Trigger(() -> isAtTargetPosition(tolerance));
    }

    private boolean isAtTargetPosition(DoubleSupplier tolerance) {
        return Math.abs(cachedRotations.getAsDouble() - target) < tolerance.getAsDouble();
    }

    public Trigger atRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionRotations() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    public Trigger belowRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    public Trigger atPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionPercentage() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    public Trigger belowPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger abovePercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    public Trigger belowDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    public Trigger atVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> Math.abs(getVelocityRPM() - target.getAsDouble()) < tolerance.getAsDouble());
    }

    public Trigger belowVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    public Trigger atCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getStatorCurrent() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    public Trigger belowCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Update the value of the stator current for the motor
     *
     * @return
     */
    public double updateCurrent() {
        if (config.attached) {
            return motor.getStatorCurrent().getValueAsDouble();
        }
        return 0;
    }

    public double getStatorCurrent() {
        return cachedCurrent.getAsDouble();
    }

    public double updateVoltage() {
        if (config.attached) {
            return motor.getMotorVoltage().getValueAsDouble();
        }
        return 0;
    }

    public double getVoltage() {
        return cachedVoltage.getAsDouble();
    }

    /**
     * Percentage to Rotations
     *
     * @return
     */
    public double percentToRotations(DoubleSupplier percent) {
        return (percent.getAsDouble() / 100) * config.maxRotations;
    }

    /**
     * Rotations to Percentage
     *
     * @param rotations
     * @return
     */
    public double rotationsToPercent(DoubleSupplier rotations) {
        return (rotations.getAsDouble() / config.maxRotations) * 100;
    }

    /**
     * Degrees to Rotations
     *
     * @return rotations
     */
    public double degreesToRotations(DoubleSupplier degrees) {
        return (degrees.getAsDouble() / 360);
    }

    /**
     * Rotations to Degrees
     *
     * @param rotations
     * @return degrees
     */
    public double rotationsToDegrees(DoubleSupplier rotations) {
        return 360 * rotations.getAsDouble();
    }

    public double getPositionRotations() {
        return cachedRotations.getAsDouble();
    }

    /**
     * Updates the position of the motor
     *
     * @return motor position in rotations
     */
    private double updatePositionRotations() {
        if (config.attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    public double getPositionPercentage() {
        return cachedPercentage.getAsDouble();
    }

    private double updatePositionPercentage() {
        return rotationsToPercent(this::getPositionRotations);
    }

    public double getPositionDegrees() {
        return cachedDegrees.getAsDouble();
    }

    private double updatePositionDegrees() {
        return rotationsToDegrees(this::getPositionRotations);
    }

    /**
     * Updates the velocity of the motor
     *
     * @return motor velocity in rotations/sec which are the CTRE native units
     */
    private double updateVelocityRPS() {
        if (config.attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    public double getVelocityRPM() {
        return cachedVelocity.getAsDouble();
    }

    // Get Velocity in RPM
    private double updateVelocityRPM() {
        return Conversions.RPStoRPM(updateVelocityRPS());
    }

    /* Commands: see method in lambda for more information */
    /**
     * Runs the Mechanism at a given velocity
     *
     * @param velocityRPM in revolutions per minute
     */
    public Command runVelocity(DoubleSupplier velocityRPM) {
        return run(() -> setVelocity(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocity");
    }

    /**
     * Run the mechanism at given velocity rpm in TorqueCurrentFOC mode
     *
     * @param velocityRPM
     * @return
     */
    public Command runVelocityTcFocRpm(DoubleSupplier velocityRPM) {
        return run(() -> setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocityFOCrpm");
    }

    public Command runPercentage(DoubleSupplier percent) {
        return run(() -> setPercentOutput(percent)).withName(getName() + ".runPercentage");
    }

    public Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltageOutput(voltage)).withName(getName() + ".runVoltage");
    }

    public Command runTorqueCurrentFoc(DoubleSupplier current) {
        return run(() -> setTorqueCurrentFoc(current)).withName(getName() + ".runTorqueCurrentFoc");
    }

    /**
     * Run to the specified position.
     *
     * @param rotations position in revolutions
     */
    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runPoseRevolutions");
    }

    /**
     * Move to the specified position.
     *
     * @param percent position in percentage of max revolutions
     */
    public Command moveToPercentage(DoubleSupplier percent) {
        return run(() -> setMMPositionFoc(() -> percentToRotations(percent)))
                .withName(getName() + ".runPosePercentage");
    }

    /**
     * Move to the specified position.
     *
     * @param degrees position in degrees
     */
    public Command moveToDegrees(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(() -> degreesToRotations(degrees)))
                .withName(getName() + ".runPoseDegrees");
    }

    /**
     * Runs to the specified Motion Magic position using FOC control. Will require different PID and
     * feedforward configs
     *
     * @param rotations position in revolutions
     */
    public Command runFocRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runFOCPosition");
    }

    public Command runStop() {
        return run(this::stop).withName(getName() + ".runStop");
    }

    /**
     * Temporarily sets the mechanism to coast mode. The configuration is applied when the command
     * is started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName(getName() + ".coastMode");
    }

    /** Sets the motor to brake mode if it is in coast mode */
    public Command ensureBrakeMode() {
        return runOnce(() -> setBrakeMode(true))
                .onlyIf(
                        () ->
                                config.attached
                                        && config.talonConfig.MotorOutput.NeutralMode
                                                == NeutralModeValue.Coast)
                .ignoringDisable(true)
                .withName(getName() + ".ensureBrakeMode");
    }

    protected Command runCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        return new InstantCommand(() -> setCurrentLimits(supplyLimit, statorLimit));
    }

    protected void setCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        // toggleSupplyCurrentLimit(supplyLimit, true);
        // toggleTorqueCurrentLimit(statorLimit, true);
        applyCurrentLimit(supplyLimit, statorLimit);
    }

    protected void stop() {
        if (isAttached()) {
            motor.stopMotor();
        }
    }

    /** Sets the mechanism position of the motor to 0 */
    protected void tareMotor() {
        if (isAttached()) {
            setMotorPosition(() -> 0);
        }
    }

    /**
     * Sets the mechanism position of the motor
     *
     * @param rotations rotations
     */
    protected void setMotorPosition(DoubleSupplier rotations) {
        if (isAttached()) {
            motor.setPosition(rotations.getAsDouble());
        }
    }

    /**
     * Closed-loop Velocity Motion Magic with torque control (requires Pro)
     *
     * @param velocityRPS rotations per second
     */
    protected void setMMVelocityFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = velocityRPS.getAsDouble();
            MotionMagicVelocityTorqueCurrentFOC mm = config.mmVelocityFOC.withVelocity(target);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Velocity with torque control (requires Pro)
     *
     * @param velocityRPS rotations per second
     */
    protected void setVelocityTorqueCurrentFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = velocityRPS.getAsDouble();
            VelocityTorqueCurrentFOC output = config.velocityTorqueCurrentFOC.withVelocity(target);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop Velocity with torque control (requires Pro)
     *
     * @param velocity rotations per second
     */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = Conversions.RPMtoRPS(velocityRPS.getAsDouble());
            VelocityTorqueCurrentFOC output = config.velocityTorqueCurrentFOC.withVelocity(target);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop velocity control with voltage compensation
     *
     * @param velocityRPS rotations per second
     */
    protected void setVelocity(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = velocityRPS.getAsDouble();
            VelocityVoltage output = config.velocityControl.withVelocity(target);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop Position Motion Magic with torque control (requires Pro)
     *
     * @param rotations rotations
     */
    protected void setMMPositionFoc(DoubleSupplier rotations) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicTorqueCurrentFOC mm = config.mmPositionFOC.withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Position Motion Magic with torque control (requires Pro) Dynamic allows you to
     * set velocity, acceleration, and jerk during the command
     *
     * @param rotations
     * @param velocity
     * @param acceleration
     * @param jerk
     */
    protected void setDynMMPositionFoc(
            DoubleSupplier rotations,
            DoubleSupplier velocity,
            DoubleSupplier acceleration,
            DoubleSupplier jerk) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            DynamicMotionMagicTorqueCurrentFOC mm =
                    config.dynamicMMPositionFOC
                            .withPosition(target)
                            .withVelocity(velocity.getAsDouble())
                            .withAcceleration(acceleration.getAsDouble())
                            .withJerk(jerk.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Position Motion Magic
     *
     * @param rotations rotations
     */
    protected void setMMPosition(DoubleSupplier rotations) {
        setMMPosition(rotations, 0);
    }

    /**
     * Closed-loop Position Motion Magic using a slot other than 0
     *
     * @param rotations rotations
     * @param slot gains slot
     */
    public void setMMPosition(DoubleSupplier rotations, int slot) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot.withSlot(slot).withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Open-loop Percent output control with voltage compensation
     *
     * @param percent fractional units between -1 and +1
     */
    public void setPercentOutput(DoubleSupplier percent) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl.withOutput(
                            config.voltageCompSaturation * percent.getAsDouble());
            motor.setControl(output);
        }
    }

    public void setVoltageOutput(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output = config.voltageControl.withOutput(voltage.getAsDouble());
            motor.setControl(output);
        }
    }

    public void setTorqueCurrentFoc(DoubleSupplier current) {
        if (isAttached()) {
            TorqueCurrentFOC output = config.torqueCurrentFOC.withOutput(current.getAsDouble());
            motor.setControl(output);
        }
    }

    public void setBrakeMode(boolean isInBrake) {
        if (isAttached()) {
            config.configNeutralBrakeMode(isInBrake);
            config.applyTalonConfig(motor);
        }
    }

    public void toggleReverseSoftLimit(boolean enabled) {
        if (isAttached()) {
            double threshold = config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
            config.configReverseSoftLimit(threshold, enabled);
            config.applyTalonConfig(motor);
        }
    }

    public void toggleTorqueCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configForwardTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.configReverseTorqueCurrentLimit(-1 * enabledLimit.getAsDouble());
                config.configStatorCurrentLimit(enabledLimit.getAsDouble(), true);
                config.applyTalonConfig(motor);
            } else {
                config.configForwardTorqueCurrentLimit(300);
                config.configReverseTorqueCurrentLimit(-300);
                config.applyTalonConfig(motor);
            }
        }
    }

    public void toggleSupplyCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configSupplyCurrentLimit(enabledLimit.getAsDouble(), true);
                config.applyTalonConfig(motor);
            } else {
                config.configSupplyCurrentLimit(enabledLimit.getAsDouble(), false);
                config.applyTalonConfig(motor);
            }
        }
    }

    public void applyCurrentLimit(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        if (isAttached()) {
            if (config.talonConfig.CurrentLimits.StatorCurrentLimit != statorLimit.getAsDouble()
                    && config.talonConfig.CurrentLimits.SupplyCurrentLimit
                            != supplyLimit.getAsDouble()) {
                config.configSupplyCurrentLimit(Math.abs(supplyLimit.getAsDouble()), true);
                config.configStatorCurrentLimit(Math.abs(statorLimit.getAsDouble()), true);
                config.configForwardTorqueCurrentLimit(Math.abs(statorLimit.getAsDouble()));
                config.configReverseTorqueCurrentLimit(-1 * Math.abs(statorLimit.getAsDouble()));
                for (int i = 0; i < 10; i++) {
                    StatusCode result = motor.getConfigurator().apply(config.talonConfig);
                    if (!result.isOK()) {
                        System.out.println(
                                "Could not apply config changes to "
                                        + config.getName()
                                        + "\'s motor ");
                    } else {
                        break;
                    }
                }
            }
        }
    }

    public Command checkAvgCurrent(DoubleSupplier expectedCurrent, DoubleSupplier tolerance) {
        return new Command() {
            double totalCurrent = 0;
            int count = 0;
            String alertText = config.name + " AvgCurrent Error";

            @Override
            public void initialize() {
                totalCurrent = 0;
                count = 0;
            }

            @Override
            public void execute() {
                totalCurrent += getStatorCurrent();
                count++;
            }

            @Override
            public void end(boolean interrupted) {
                double avgCurrent = totalCurrent / count;
                if (Math.abs(avgCurrent - expectedCurrent.getAsDouble())
                        > tolerance.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + avgCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    public Command checkMaxCurrent(DoubleSupplier expectedCurrent) {
        return new Command() {
            double maxCurrent = 0;
            String alertText = config.name + " MaxCurrent Error";

            @Override
            public void initialize() {
                maxCurrent = 0;
            }

            @Override
            public void execute() {
                double current = getStatorCurrent();
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (maxCurrent > expectedCurrent.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + maxCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    public Command checkMinThresholdCurrent(DoubleSupplier expectedCurrent) {
        return new Command() {
            double maxCurrent = 0;
            String alertText = config.name + " Current Error";

            @Override
            public void initialize() {
                maxCurrent = 0;
            }

            @Override
            public void execute() {
                double current = getStatorCurrent();
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (maxCurrent < expectedCurrent.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected at least: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + maxCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    public static class FollowerConfig {
        @Getter private String name;
        @Getter private CanDeviceId id;
        @Getter private boolean attached = true;
        @Getter private boolean opposeLeader = false;

        public FollowerConfig(String name, int id, String canbus, boolean opposeLeader) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    public static class Config {
        @Getter private String name;
        @Getter @Setter private boolean attached = true;
        @Getter private CanDeviceId id;
        @Getter @Setter protected TalonFXConfiguration talonConfig;
        @Getter private int numMotors = 1;
        @Getter private double voltageCompSaturation = 12.0; // 12V by default
        @Getter private double minRotations = 0;
        @Getter private double maxRotations = 1;

        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        @Getter
        private MotionMagicVelocityTorqueCurrentFOC mmVelocityFOC =
                new MotionMagicVelocityTorqueCurrentFOC(0);

        @Getter
        private MotionMagicTorqueCurrentFOC mmPositionFOC = new MotionMagicTorqueCurrentFOC(0);

        @Getter
        private DynamicMotionMagicTorqueCurrentFOC dynamicMMPositionFOC =
                new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0);

        @Getter
        private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0);

        @Getter private MotionMagicVoltage mmPositionVoltage = new MotionMagicVoltage(0);

        @Getter
        private MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);

        @Getter private VoltageOut voltageControl = new VoltageOut(0);
        @Getter private VelocityVoltage velocityControl = new VelocityVoltage(0);

        @Getter
        private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        @Getter private TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);

        @Getter
        private DutyCycleOut percentOutput =
                new DutyCycleOut(
                        0); // Percent Output control using percentage of supply voltage //Should
        // normally use VoltageOut

        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();

            /* Put default config settings for all mechanisms here */
            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        public void applyTalonConfig(TalonFX talon) {
            StatusCode result = talon.getConfigurator().apply(talonConfig);
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to " + name + "\'s motor ", false);
            }
        }

        public void setFollowerConfigs(FollowerConfig... followers) {
            followerConfigs = followers;
        }

        public void configVoltageCompensation(double voltageCompSaturation) {
            this.voltageCompSaturation = voltageCompSaturation;
        }

        public void configCounterClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public void configClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        public void configSupplyCurrentLimit(double supplyLimit, boolean enabled) {
            if (supplyLimit < 0) {
                supplyLimit = -supplyLimit;
            }
            talonConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
        }

        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            if (statorLimit < 0) {
                statorLimit = -statorLimit;
            }
            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        public void configForwardTorqueCurrentLimit(double currentLimit) {
            if (currentLimit < 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        }

        public void configReverseTorqueCurrentLimit(double currentLimit) {
            if (currentLimit > 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        }

        public void configNeutralDeadband(double deadband) {
            talonConfig.MotorOutput.DutyCycleNeutralDeadband = deadband;
        }

        public void configPeakOutput(double forward, double reverse) {
            talonConfig.MotorOutput.PeakForwardDutyCycle = forward;
            talonConfig.MotorOutput.PeakReverseDutyCycle = reverse;
        }

        public void configForwardSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        }

        public void configReverseSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        }

        public void configContinuousWrap(boolean enabled) {
            talonConfig.ClosedLoopGeneral.ContinuousWrap = enabled;
        }

        // Configure optional motion magic velocity parameters
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC =
                    mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage =
                    mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        // Configure optional motion magic position parameters
        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
        }

        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

        // This is the ratio of rotor rotations to the mechanism's output.
        // If a remote sensor is used this a ratio of sensor rotations to the mechanism's output.
        public void configGearRatio(double gearRatio) {
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        public void configNeutralBrakeMode(boolean isInBrake) {
            if (isInBrake) {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            } else {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            }
        }

        /**
         * Defaults to slot 0
         *
         * @param kP
         * @param kI
         * @param kD
         */
        public void configPIDGains(double kP, double kI, double kD) {
            configPIDGains(0, kP, kI, kD);
        }

        public void configPIDGains(int slot, double kP, double kI, double kD) {
            talonConfigFeedbackPID(slot, kP, kI, kD);
        }

        /**
         * Defaults to slot 0
         *
         * @param kS
         * @param kV
         * @param kA
         * @param kG
         */
        public void configFeedForwardGains(double kS, double kV, double kA, double kG) {
            configFeedForwardGains(0, kS, kV, kA, kG);
        }

        public void configFeedForwardGains(int slot, double kS, double kV, double kA, double kG) {
            talonConfigFeedForward(slot, kV, kA, kS, kG);
        }

        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }

        /**
         * Defaults to slot 0
         *
         * @param isArm
         */
        public void configGravityType(boolean isArm) {
            configGravityType(0, isArm);
        }

        public void configGravityType(int slot, boolean isArm) {
            GravityTypeValue gravityType =
                    isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
            if (slot == 0) {
                talonConfig.Slot0.GravityType = gravityType;
            } else if (slot == 1) {
                talonConfig.Slot1.GravityType = gravityType;
            } else if (slot == 2) {
                talonConfig.Slot2.GravityType = gravityType;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid slot", false);
            }
        }

        // Configure the TalonFXConfiguration feed forward gains
        private void talonConfigFeedForward(int slot, double kV, double kA, double kS, double kG) {
            if (slot == 0) {
                talonConfig.Slot0.kV = kV;
                talonConfig.Slot0.kA = kA;
                talonConfig.Slot0.kS = kS;
                talonConfig.Slot0.kG = kG;
            } else if (slot == 1) {
                talonConfig.Slot1.kV = kV;
                talonConfig.Slot1.kA = kA;
                talonConfig.Slot1.kS = kS;
                talonConfig.Slot1.kG = kG;
            } else if (slot == 2) {
                talonConfig.Slot2.kV = kV;
                talonConfig.Slot2.kA = kA;
                talonConfig.Slot2.kS = kS;
                talonConfig.Slot2.kG = kG;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid FeedForward slot", false);
            }
        }

        private void talonConfigFeedbackPID(int slot, double kP, double kI, double kD) {
            if (slot == 0) {
                talonConfig.Slot0.kP = kP;
                talonConfig.Slot0.kI = kI;
                talonConfig.Slot0.kD = kD;
            } else if (slot == 1) {
                talonConfig.Slot1.kP = kP;
                talonConfig.Slot1.kI = kI;
                talonConfig.Slot1.kD = kD;
            } else if (slot == 2) {
                talonConfig.Slot2.kP = kP;
                talonConfig.Slot2.kI = kI;
                talonConfig.Slot2.kD = kD;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid Feedback slot", false);
            }
        }

        /**
         * Sets the minimum and maximum motor rotations
         *
         * @param minRotation
         * @param maxRotation
         */
        protected void configMinMaxRotations(double minRotation, double maxRotation) {
            this.minRotations = minRotation;
            this.maxRotations = maxRotation;
        }
    }
}
