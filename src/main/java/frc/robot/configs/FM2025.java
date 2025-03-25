package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2025 extends Config {

    public FM2025() {
        super();
        swerve.configEncoderOffsets(-0.393799, 0.115234, 0.054932, 0.388184);
        // swerve.configEncoderOffsets(-0.395264, 0.11499, 0.061279, 0.385742);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
        shoulder.setAttached(true);
        elbow.setAttached(true);
        twist.setAttached(true);
        intake.setAttached(true);
        climb.setAttached(true);
    }
}
