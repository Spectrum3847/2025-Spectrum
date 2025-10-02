package frc.robot.configs;

import frc.robot.Robot.Config;

public class OM2025 extends Config {

    public OM2025() {
        super();
        swerve.configEncoderOffsets(-0.395508, 0.117920, 0.055176, 0.386963);
        // swerve.configEncoderOffsets(-0.395264, 0.11499, 0.061279, 0.385742);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
        claw.setAttached(true);
        shoulder.setAttached(true);
        groundIntake.setAttached(true);
        climbPivot.setAttached(true);

        // shoulder.setCANcoderAttached(true);
        // shoulder.setCANcoderOffset(
        //         -0.237305 - 0.208333333); // add -0.208333333 to the inverse of the position
    }
}
