package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class TagDistanceAlignController {
    Swerve swerve;
    SwerveConfig config;
    PIDController controller;
    Constraints constraints;

    double calculatedValue = 0;

    public TagDistanceAlignController(SwerveConfig config) {
        this.config = config;
        controller =
                new PIDController(
                        config.getKPTagDistanceController(),
                        config.getKITagDistanceController(),
                        config.getKDTagDistanceController());

        controller.setTolerance(config.getTagDistanceTolerance());
    }

    public double calculate(double goalArea, double currentArea) {
        calculatedValue = controller.calculate(currentArea, goalArea);

        if (atSetpoint()) {
            calculatedValue = 0;
            return calculatedValue;
        } else {
            return calculatedValue;
        }
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public void reset(double current) {
        // controller.reset(current);
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
