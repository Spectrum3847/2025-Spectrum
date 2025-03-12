package frc.robot.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class TagCenterAlignController {
    Swerve swerve;
    SwerveConfig config;
    ProfiledPIDController controller;
    Constraints constraints;

    double calculatedValue = 0;

    public TagCenterAlignController(SwerveConfig config) {
        this.config = config;
        constraints = new Constraints(4.7 / 4, 4.7 / 40);
        controller =
                new ProfiledPIDController(
                        config.getKPTagCenterController(),
                        config.getKITagCenterController(),
                        config.getKDTagController(),
                        constraints);

        controller.setTolerance(config.getTagCenterTolerance());
    }

    public double calculate(double goalMeters, double currentMeters) {
        calculatedValue = controller.calculate(currentMeters, goalMeters);

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

    public void reset(double currentMeters) {
        controller.reset(currentMeters);
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
