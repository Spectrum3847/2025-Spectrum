package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.spectrumLib.mechanism.Mechanism.Config;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Wraps a CTRE CANcoder and applies Spectrum-specific configuration. On construction the encoder is
 * configured and the supplied TalonFX motor's feedback source is updated to use it.
 */
public class SpectrumCANcoder {

    /** The underlying CTRE CANcoder hardware object. */
    @Getter private CANcoder canCoder;

    private SpectrumCANcoderConfig config;

    /** Selects how the TalonFX reads position data from the remote CANcoder. */
    public enum CANCoderFeedbackType {
        /** Position is read remotely; motor encoder is used for velocity. */
        RemoteCANcoder,
        /** CANcoder position is fused with the motor encoder for high-bandwidth feedback. */
        FusedCANcoder,
        /** Motor encoder is synchronized to the CANcoder position on enable. */
        SyncCANcoder,
    }

    private CANCoderFeedbackType feedbackSource = CANCoderFeedbackType.FusedCANcoder;

    /**
     * Creates and configures a SpectrumCANcoder, then updates the motor's feedback configuration.
     *
     * @param CANcoderID CAN device ID of the CANcoder
     * @param config Configuration object containing offset, inversion, and ratio values
     * @param motor The TalonFX whose feedback configuration will be updated
     * @param mechConfig The mechanism configuration that holds the TalonFX config to modify
     * @param feedbackSource How the TalonFX should read data from this CANcoder
     */
    public SpectrumCANcoder(
            int CANcoderID,
            SpectrumCANcoderConfig config,
            TalonFX motor,
            Config mechConfig,
            CANCoderFeedbackType feedbackSource) {
        this.config = config;
        config.setCANcoderID(CANcoderID);
        this.feedbackSource = feedbackSource;

        if (config.isAttached()) {
            // Fused/Sync feedback requires the CANcoder to be on the same bus as the motor.
            canCoder = new CANcoder(CANcoderID, motor.getNetwork());
            CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
            canCoderConfigs.MagnetSensor.MagnetOffset = config.getOffset();
            canCoderConfigs.MagnetSensor.SensorDirection =
                    config.isInverted()
                            ? SensorDirectionValue.Clockwise_Positive
                            : SensorDirectionValue.CounterClockwise_Positive;
            canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            if (canCoderResponseOK(
                    CanConfigBudget.run(
                            "CANcoder " + CANcoderID,
                            timeout ->
                                    canCoder.getConfigurator().apply(canCoderConfigs, timeout)))) {
                // Modify configuration to use remote CANcoder fused
                modifyMotorConfig(motor, mechConfig);
            }
        }
    }

    /**
     * Returns whether this CANcoder is configured as physically present on the robot.
     *
     * @return {@code true} if the CANcoder is attached
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * Updates the TalonFX feedback configuration to reference this CANcoder using the chosen
     * feedback source type and the ratios defined in the config.
     *
     * @param motor The TalonFX motor to reconfigure
     * @param mechConfig The mechanism configuration whose stored TalonFX config is modified in
     *     place
     * @return this instance, for chaining
     */
    public SpectrumCANcoder modifyMotorConfig(TalonFX motor, Config mechConfig) {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration talonConfigMod = mechConfig.getTalonConfig();
        talonConfigMod.Feedback.FeedbackRemoteSensorID = config.getCANcoderID();
        switch (feedbackSource) {
            case RemoteCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        talonConfigMod.Feedback.RotorToSensorRatio = config.getRotorToSensorRatio();
        talonConfigMod.Feedback.SensorToMechanismRatio = config.getSensorToMechanismRatio();
        configurator.apply(talonConfigMod);
        mechConfig.setTalonConfig(talonConfigMod);
        return this;
    }

    /**
     * Checks whether a CANcoder configuration response indicates success. Prints a warning via
     * {@link Telemetry} if the response is not OK.
     *
     * @param response The {@link StatusCode} returned by the CANcoder configurator
     * @return {@code true} if the response is OK, {@code false} otherwise
     */
    public boolean canCoderResponseOK(StatusCode response) {
        if (!response.isOK()) {
            Telemetry.print(
                    "CANcoder ID "
                            + config.getCANcoderID()
                            + " failed config with error "
                            + response.toString());
            return false;
        }
        return true;
    }
}
