package frc.robot.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class TranslationYController {
    Swerve swerve;
    SwerveConfig config;
    ProfiledPIDController controller;

    double calculatedValue = 0;

    public TranslationYController(SwerveConfig config) {
        this.config = config;
        controller =
                new ProfiledPIDController(
                        config.getKPTranslationController(),
                        config.getKITranslationController(),
                        config.getKDTranslationController(),
                        config.getTranslationConstraints());

        SmartDashboard.putData("Y controller", controller);
    }

    public double calculate(DoubleSupplier goalMeters, DoubleSupplier currentMeters) {
        calculatedValue =
                controller.calculate(currentMeters.getAsDouble(), goalMeters.getAsDouble());

        SmartDashboard.putNumber("Y Controller Output", calculatedValue);
        if (atGoal(currentMeters.getAsDouble())) {
            calculatedValue = 0;
            return calculatedValue;
        } else {
            return calculatedValue + (config.getKSdrive() * Math.signum(calculatedValue));
        }
    }

    public boolean atGoal(double current) {
        double goal = controller.getGoal().position;
        boolean atGoal = Math.abs(current - goal) < config.getTranslationTolerance();
        System.out.println("Y At Goal: " + atGoal + " Goal: " + goal + " Current: " + current);
        return atGoal;
    }

    public void reset(double currentMeters) {
        controller.reset(currentMeters);
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
