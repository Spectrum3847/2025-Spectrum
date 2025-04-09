package frc.robot.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        ;
        SmartDashboard.putData("X controller", controller);
    }

    public double calculate(double goalMeters, double currentMeters) {
        calculatedValue = controller.calculate(currentMeters, goalMeters);

        if (atGoal(currentMeters)) {
            calculatedValue = 0;
            return calculatedValue;
        } else {
            return calculatedValue + (config.getKSdrive() * Math.signum(calculatedValue));
        }
    }

    public boolean atGoal(double current) {
        double goal = controller.getGoal().position;
        boolean atGoal = Math.abs(current - goal) < config.getTranslationTolerance();
        System.out.println("X At Goal: " + atGoal + " Goal: " + goal + " Current: " + current);
        return atGoal;
    }

    public void reset(double currentMeters) {
        controller.reset(currentMeters);
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
