package frc.robot.configs;

import frc.robot.Robot.Config;

/** The 2025 competition robot (Gamma Ray). CAN IDs are in each subsystem's config. */
public class FM2025 extends Config {

    public FM2025() {
        super();
        swerve.configEncoderOffsets(-0.395508, 0.117920, 0.055176, 0.386963);

        shoulder.setCANcoderAttached(true);
        // Add -0.208333 to the inverse of the position read with the arm hanging straight down.
        shoulder.setCANcoderOffset(-0.237305 - 0.208333333);
        twist.setCANcoderAttached(true);
        twist.setCANcoderOffset(0.266846);
    }
}
