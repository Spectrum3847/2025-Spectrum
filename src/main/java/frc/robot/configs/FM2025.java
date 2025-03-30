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

        shoulder.setCANcoderAttached(true);
        shoulder.setCANcoderOffset(-0.169026333); // add -0.208333333 to the inverse of the position

        elbow.setCANcoderAttached(true);
        elbow.setCANcoderOffset(0.45125333333);
        // 0.460042333); // add 0.208333333 to the inverse of the position
        twist.setCANcoderAttached(true);
        twist.setCANcoderOffset(0.271484); // 0.112793 // -0.358154 // 0.462402); // -0.485596
    }
}
