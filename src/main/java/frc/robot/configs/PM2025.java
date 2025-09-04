package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2025 extends Config {

    public PM2025() {
        super();
        swerve.configEncoderOffsets(0.490967, 0.410400, -0.208252, 0.422607);
        climb.configGearRatio(99.5555555555);
        // climb.setMinRotations(-0.055);
        // climb.setMaxRotations(0.45);
        climb.configReverseSoftLimit(-0.045, true);
        climb.configForwardSoftLimit(0.45, true);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
        shoulder.setAttached(true);
        elbow.setAttached(true);
        twist.setAttached(true);
        intake.setAttached(true);
        climb.setAttached(true);

        shoulder.setCANcoderAttached(false);
        shoulder.setCANcoderOffset(0.035318666);
        elbow.setCANcoderAttached(false);
        twist.setCANcoderAttached(true);
        twist.setCANcoderOffset(-0.385254);
    }
}
