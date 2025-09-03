package frc.spectrumLib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.spectrumLib.mechanism.Mechanism.Config;
import lombok.Getter;

public class SpectrumCANcoder {

    @Getter private CANcoder canCoder;
    private SpectrumCANcoderConfig config;

    public enum CANCoderFeedbackType {
        RemoteCANcoder,
        FusedCANcoder,
        SyncCANcoder,
    }

    private CANCoderFeedbackType feedbackSource = CANCoderFeedbackType.FusedCANcoder;

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
            canCoder = new CANcoder(CANcoderID, Rio.CANIVORE);
            CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
            canCoderConfigs.MagnetSensor.MagnetOffset = config.getOffset();
            canCoderConfigs.MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive;
            canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            if (canCoderResponseOK(canCoder.getConfigurator().apply(canCoderConfigs))) {
                // Modify configuration to use remote CANcoder fused
                modifyMotorConfig(motor, mechConfig);
            }
        }
    }

    public boolean isAttached() {
        return config.isAttached();
    }

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
