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
    @Getter private int CANcoderID;
    @Getter private double gearRatio = 1;
    @Getter private double offset = 0;
    @Getter private boolean attached = false;

    @Getter private CANcoder canCoder;

    private enum CANCoderFeedbackType {
        RemoteCANcoder,
        FusedCANcoder,
        SyncCANcoder,
    }

    private CANCoderFeedbackType feedbackSource = CANCoderFeedbackType.FusedCANcoder;

    public SpectrumCANcoder(int CANcoderID, TalonFX motor, Config config) {
        this.CANcoderID = CANcoderID;

        if (isAttached()) {
            canCoder = new CANcoder(CANcoderID, Rio.CANIVORE);
            CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
            canCoderConfigs.MagnetSensor.MagnetOffset = offset;
            canCoderConfigs.MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive;
            canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            if (canCoderResponseOK(canCoder.getConfigurator().apply(canCoderConfigs))) {
                // Modify configuration to use remote CANcoder fused
                modifyMotorConfig(motor, config);
            }
        }
    }

    public SpectrumCANcoder setGearRatio(double ratio) {
        gearRatio = ratio;
        return this;
    }

    public SpectrumCANcoder setOffset(double offset) {
        this.offset = offset;
        return this;
    }

    public SpectrumCANcoder setAttached(boolean attached) {
        this.attached = attached;
        return this;
    }

    public SpectrumCANcoder modifyMotorConfig(TalonFX motor, Config config) {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration talonConfigMod = config.getTalonConfig();
        talonConfigMod.Feedback.FeedbackRemoteSensorID = CANcoderID;
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
        talonConfigMod.Feedback.RotorToSensorRatio = gearRatio;
        configurator.apply(talonConfigMod);
        config.setTalonConfig(talonConfigMod);
        return this;
    }

    public boolean canCoderResponseOK(StatusCode response) {
        if (!response.isOK()) {
            Telemetry.print(
                    "CANcoder ID "
                            + CANcoderID
                            + " failed config with error "
                            + response.toString());
            return false;
        }
        return true;
    }
}
