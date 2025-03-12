package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM20235 extends Config {

    // 2023.5 Off-season Robot

    public FM20235() {
        super();
        swerve.configEncoderOffsets(-0.399170, -0.189209, 0.121826, -0.021973);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        shoulder.setAttached(true);
        elbow.setAttached(true);

        elevator.setAttached(false);
    }
}
