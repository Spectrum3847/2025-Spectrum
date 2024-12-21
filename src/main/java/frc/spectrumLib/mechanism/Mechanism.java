package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.CachedDouble;
import frc.spectrumLib.SpectrumRobot;
import frc.spectrumLib.SpectrumSubsystem;
import frc.spectrumLib.talonFX.TalonFXFactory;
import frc.spectrumLib.util.CanDeviceId;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
    Alert disconnectedAlert =
            new Alert(getName() + " motor " + config.getId() + " disconnected", AlertType.kWarning);
    Alert unlicensedAlert = new Alert(getName() + " motor " + config.getId() + " unlicensed", AlertType.kWarning);

    private final CachedDouble cachedRotations;
    private final CachedDouble cachedPercentage;
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
        cachedRotations = new CachedDouble(this::updatePositionRotations);
        cachedPercentage = new CachedDouble(this::updatePositionPercentage);
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
                () -> Math.abs(getCurrent() - target.getAsDouble()) < tolerance.getAsDouble());
    }

    public Trigger belowCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(() -> getCurrent() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(() -> getCurrent() > (target.getAsDouble() - tolerance.getAsDouble()));
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

    public double getCurrent() {
        return cachedCurrent.getAsDouble();
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

    /**
     * Run to the specified position.
     *
     * @param rotations position in revolutions
     */
    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> setMMPosition(rotations)).withName(getName() + ".runPoseRevolutions");
    }

    /**
     * Move to the specified position.
     *
     * @param percent position in percentage of max revolutions
     */
    public Command moveToPercentage(DoubleSupplier percent) {
        return run(() -> setMMPosition(() -> percentToRotations(percent)))
                .withName(getName() + ".runPosePercentage");
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

    protected void checkMotorOK(Supplier<StatusCode> motorRequest) {
        if (motorRequest.get().isOK()) {
            disconnectedAlert.set(false);
            unlicensedAlert.set(false);
            return;
        }

        if (motorRequest.get().isOK()) {
            return;
        }

        disconnectedAlert.set(true);

        if (motorRequest.get().value == StatusCode.UnlicensedDevice.value) {
                unlicensedAlert.set(true);
        }
    }

    /**
     * Sets the mechanism position of the motor
     *
     * @param rotations rotations
     */
    protected void setMotorPosition(DoubleSupplier rotations) {
        if (!isAttached()) {
            disconnectedAlert.set(false);
            return;
        }
        checkMotorOK(() -> motor.setPosition(rotations.getAsDouble()));
    }

    /**
     * Closed-loop Velocity Motion Magic with torque control (requires Pro)
     *
     * @param velocityRPS rotations per second
     */
    protected void setMMVelocityFOC(DoubleSupplier velocityRPS) {
        if (!isAttached()) {
            return;
        }

        MotionMagicVelocityTorqueCurrentFOC mm =
                config.mmVelocityFOC.withVelocity(velocityRPS.getAsDouble());

        checkMotorOK(() -> motor.setControl(mm));
    }

    /**
     * Closed-loop Velocity with torque control (requires Pro)
     *
     * @param velocityRPS rotations per second
     */
    protected void setVelocityTorqueCurrentFOC(DoubleSupplier velocityRPS) {
        if (!isAttached()) {
            return;
        }

        VelocityTorqueCurrentFOC output =
                config.velocityTorqueCurrentFOC.withVelocity(velocityRPS.getAsDouble());

        checkMotorOK(() -> motor.setControl(output));
    }

    /**
     * Closed-loop Velocity with torque control (requires Pro)
     *
     * @param velocity rotations per second
     */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPS) {
        if (!isAttached()) {
            return;
        }

        VelocityTorqueCurrentFOC output =
                config.velocityTorqueCurrentFOC.withVelocity(
                        Conversions.RPMtoRPS(velocityRPS.getAsDouble()));

        checkMotorOK(() -> motor.setControl(output));
    }

    /**
     * Closed-loop velocity control with voltage compensation
     *
     * @param velocityRPS rotations per second
     */
    protected void setVelocity(DoubleSupplier velocityRPS) {
        if (!isAttached()) {
            return;
        }

        VelocityVoltage output = config.velocityControl.withVelocity(velocityRPS.getAsDouble());

        checkMotorOK(() -> motor.setControl(output));
    }

    /**
     * Closed-loop Position Motion Magic with torque control (requires Pro)
     *
     * @param rotations rotations
     */
    protected void setMMPositionFoc(DoubleSupplier rotations) {
        if (!isAttached()) {
            return;
        }

        MotionMagicTorqueCurrentFOC mm = config.mmPositionFOC.withPosition(rotations.getAsDouble());

        checkMotorOK(() -> motor.setControl(mm));
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
        if (!isAttached()) {
            return;
        }

        MotionMagicVoltage mm =
                config.mmPositionVoltageSlot.withSlot(slot).withPosition(rotations.getAsDouble());

        checkMotorOK(() -> motor.setControl(mm));
    }

    /**
     * Open-loop Percent output control with voltage compensation
     *
     * @param percent fractional units between -1 and +1
     */
    public void setPercentOutput(DoubleSupplier percent) {
        if (!isAttached()) {
            return;
        }

        VoltageOut output =
                config.voltageControl.withOutput(
                        config.voltageCompSaturation * percent.getAsDouble());

        checkMotorOK(() -> motor.setControl(output));
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
                config.configReverseTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.applyTalonConfig(motor);
            } else {
                config.configForwardTorqueCurrentLimit(400);
                config.configReverseTorqueCurrentLimit(400);
                config.applyTalonConfig(motor);
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
                totalCurrent += getCurrent();
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
                double current = getCurrent();
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
                double current = getCurrent();
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
        @Getter protected TalonFXConfiguration talonConfig;
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
        private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0);

        @Getter private MotionMagicVoltage mmPositionVoltage = new MotionMagicVoltage(0);

        @Getter
        private MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);

        @Getter private VoltageOut voltageControl = new VoltageOut(0);
        @Getter private VelocityVoltage velocityControl = new VelocityVoltage(0);

        @Getter
        private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

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
            talonConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
        }

        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        public void configForwardTorqueCurrentLimit(double currentLimit) {
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        }

        public void configReverseTorqueCurrentLimit(double currentLimit) {
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
