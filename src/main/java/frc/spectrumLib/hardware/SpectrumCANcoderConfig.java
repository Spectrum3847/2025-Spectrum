package frc.spectrumLib.hardware;

import lombok.Getter;
import lombok.Setter;

/** Configuration parameters for a {@link SpectrumCANcoder}. */
public class SpectrumCANcoderConfig {
    /** CAN device ID of the CANcoder; may be set after construction. */
    @Getter @Setter private int CANcoderID;
    /** Gear ratio between the motor rotor and the CANcoder shaft (rotor turns / sensor turn). */
    @Getter private double rotorToSensorRatio = 1;
    /**
     * Gear ratio between the CANcoder shaft and the mechanism output (sensor turns / mechanism
     * turn).
     */
    @Getter private double sensorToMechanismRatio = 1;
    /** Magnetic offset applied to the CANcoder reading, in rotations. */
    @Getter private double offset = 0;
    /** Whether the CANcoder hardware is physically present on the robot. */
    @Getter private boolean attached = false;
    /** Whether the CANcoder sensor direction is inverted (clockwise positive). */
    @Getter private boolean inverted = false;

    /**
     * Creates a fully-specified CANcoder configuration.
     *
     * @param rotorToSensorRatio Gear ratio from motor rotor to CANcoder shaft
     * @param sensorToMechanismRatio Gear ratio from CANcoder shaft to mechanism output
     * @param offset Magnetic offset in rotations
     * @param attached {@code true} if the CANcoder is physically installed
     * @param inverted {@code true} to make clockwise rotation positive
     */
    public SpectrumCANcoderConfig(
            double rotorToSensorRatio,
            double sensorToMechanismRatio,
            double offset,
            boolean attached,
            boolean inverted) {
        this.rotorToSensorRatio = rotorToSensorRatio;
        this.sensorToMechanismRatio = sensorToMechanismRatio;
        this.offset = offset;
        this.attached = attached;
        this.inverted = inverted;
    }
}
