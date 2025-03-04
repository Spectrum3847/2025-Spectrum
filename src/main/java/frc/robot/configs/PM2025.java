package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2025 extends Config {

    public PM2025() {
        super();
        swerve.configEncoderOffsets(0.490967, 0.410400, -0.208252, 0.422607);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
        shoulder.setAttached(true);
        elbow.setAttached(true);
        twist.setAttached(true);
        intake.setAttached(false);
        climb.setAttached(false);
        groundIntake.setAttached(false);
    }
}
