package frc.robot.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class TranslationXController {
    Swerve swerve;
    SwerveConfig config;
    ProfiledPIDController controller;

    double calculatedValue = 0;

    public TranslationXController(SwerveConfig config) {
        this.config = config;
        controller =
                new ProfiledPIDController(
                        config.getKPTranslationController(),
                        config.getKITranslationController(),
                        config.getKDTranslationController(),
                        config.getTranslationConstraints());

        controller.setTolerance(config.getTranslationTolerance());
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
