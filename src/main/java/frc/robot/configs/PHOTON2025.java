package frc.robot.configs;

import frc.robot.Robot.Config;
import frc.robot.elevator.PhotonElevatorConfig;
import frc.robot.intake.PhotonIntakeConfig;
import frc.robot.leds.PhotonLEDsConfig;
import frc.robot.shoulder.PhotonShoulderConfig;

public class PHOTON2025 extends Config {
    public PHOTON2025() {
        super();
        // to flip direction of swerve add 0.5 to negative values and subtract 0.5 from positive
        // values
        // swerve.configEncoderOffsets(0.204590, -0.414062, -0.163330, 0.026855);
        swerve.configEncoderOffsets(-0.29541, 0.085938, 0.33667, 0.387451);

        elevator = new PhotonElevatorConfig();
        shoulder = new PhotonShoulderConfig();
        intake = new PhotonIntakeConfig();
        leds = new PhotonLEDsConfig();

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
        shoulder.setAttached(true);
        intake.setAttached(true);
        climb.setAttached(true);
        leds.setAttached(true);

        // Always false for Photon
        elbow.setAttached(false);
        twist.setAttached(false);
    }
}
