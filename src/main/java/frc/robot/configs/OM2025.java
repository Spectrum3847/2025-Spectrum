package frc.robot.configs;

import frc.robot.Robot.Config;

public class OM2025 extends Config{
    
    public OM2025() {
        super();
        //attached things
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
        shoulder.setAttached(false);
        elbow.setAttached(true);
        twist.setAttached(false);
        intake.setAttached(true);
        climb.setAttached(true);
    }

}
