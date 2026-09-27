package frc.spectrumLib.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import frc.spectrumLib.util.CanDeviceId;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application.
 */
public class TalonFXFactory {

    private static NeutralModeValue neutralMode = NeutralModeValue.Brake;
    private static InvertedValue invertValue = InvertedValue.CounterClockwise_Positive;
    private static double neutralDeadband = 0.04;
    private static double supplyCurrentLimit = 40;

    /** Utility class — not instantiable. */
    private TalonFXFactory() {}

    /**
     * Creates a TalonFX configured with Spectrum's default parameter set.
     *
     * @param id CAN device identifier (device number + bus name)
     * @return the configured TalonFX
     */
    public static TalonFX createDefaultTalon(CanDeviceId id) {
        var talon = createTalon(id);
        CanConfigBudget.run(
                "Talon " + id.getDeviceNumber(),
                timeout -> talon.getConfigurator().apply(getDefaultConfig(), timeout));
        return talon;
    }

    /**
     * Creates a TalonFX and applies the supplied configuration.
     *
     * @param id CAN device identifier
     * @param config The {@link TalonFXConfiguration} to apply
     * @return the configured TalonFX
     */
    public static TalonFX createConfigTalon(CanDeviceId id, TalonFXConfiguration config) {
        var talon = createTalon(id);
        CanConfigBudget.run(
                "Talon " + id.getDeviceNumber(),
                timeout -> talon.getConfigurator().apply(config, timeout));
        return talon;
    }

    /**
     * Follow the motor output of another Talon.
     *
     * @param followerId Device ID of the follower.
     * @param leaderTalonFX The leader TalonFX to follow.
     * @param motorAlignment Set to Aligned for motor invert to match the leader's configured Invert
     *     - which is typical when leader and follower are mechanically linked and spin in the same
     *     direction. Set to Opposed for motor invert to oppose the leader's configured Invert -
     *     this is typical where the leader and follower mechanically spin in opposite directions.
     */
    public static TalonFX createPermanentFollowerTalon(
            CanDeviceId followerId, TalonFX leaderTalonFX, MotorAlignmentValue motorAlignment) {
        String leaderCanBus = leaderTalonFX.getNetwork().toString();
        int leaderId = leaderTalonFX.getDeviceID();
        if (!followerId.getBus().equals(leaderCanBus)) {
            throw new IllegalArgumentException(
                    "Leader and Follower Talons must be on the same CAN bus");
        }

        TalonFXConfiguration followerConfig = getDefaultConfig();
        leaderTalonFX.getConfigurator().refresh(followerConfig);
        final TalonFX talon = createConfigTalon(followerId, followerConfig);

        talon.setControl(new Follower(leaderId, motorAlignment));
        return talon;
    }

    /**
     * Builds a {@link TalonFXConfiguration} populated with Spectrum's standard defaults: brake
     * neutral mode, counter-clockwise positive invert, 4 % duty-cycle deadband, 40 A supply current
     * limit, software and hardware limits disabled, rotor sensor feedback, and audio cues enabled.
     *
     * @return a new configuration object with default values applied
     */
    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = invertValue;
        config.MotorOutput.DutyCycleNeutralDeadband = neutralDeadband;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.SensorToMechanismRatio = 1;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.Audio.BeepOnBoot = true;
        config.Audio.AllowMusicDurDisable = true;
        config.Audio.BeepOnConfig = true;

        return config;
    }
    /** Creates the talon. */
    private static TalonFX createTalon(CanDeviceId id) {
        TalonFX talon = new TalonFX(id.getDeviceNumber(), new CANBus(id.getBus()));
        // Blocking, and worthless on a bus with nothing on it.
        if (!CanConfigBudget.exhausted()) {
            talon.clearStickyFaults();
        }

        return talon;
    }
}
