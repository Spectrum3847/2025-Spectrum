package frc.spectrumLib;

import lombok.Getter;
import lombok.Setter;

public class SpectrumCANcoderConfig {
    @Getter @Setter private int CANcoderID;
    @Getter private double rotorToSensorRatio = 1;
    @Getter private double sensorToMechanismRatio = 1;
    @Getter private double offset = 0;
    @Getter private boolean attached = false;

    public SpectrumCANcoderConfig(
            double rotorToSensorRatio,
            double sensorToMechanismRatio,
            double offset,
            boolean attached) {
        this.rotorToSensorRatio = rotorToSensorRatio;
        this.sensorToMechanismRatio = sensorToMechanismRatio;
        this.offset = offset;
        this.attached = attached;
    }
}
