package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class TranslationXController {
    private final SwerveConfig config;
    private final ProfiledPIDController controller;
    private final double deadband = 1e-3;

    public TranslationXController(SwerveConfig config) {
        this.config = config;
        this.controller =
                new ProfiledPIDController(
                        config.getKPTranslationController(),
                        config.getKITranslationController(),
                        config.getKDTranslationController(),
                        config.getTranslationConstraints());

        controller.setTolerance(config.getTranslationTolerance());
        SmartDashboard.putData("X Controller", controller);
    }

    public double calculate(double goalMeters, double currentMeters) {
        if (controller.atGoal()) {
            return 0.0;
        }

        double output = controller.calculate(currentMeters, goalMeters);

        if (Math.abs(output) > deadband) {
            output += config.getKSdrive() * Math.signum(output);
        }

        output =
                MathUtil.clamp(
                        output,
                        -config.getTranslationConstraints().maxVelocity,
                        config.getTranslationConstraints().maxVelocity);

        // SmartDashboard.putNumber("X Controller Output", output);
        // SmartDashboard.putBoolean("X At Goal", controller.atGoal());
        // SmartDashboard.putNumber("X Position Error", controller.getPositionError());
        // SmartDashboard.putNumber("X Tolerance", controller.getPositionTolerance());
        return output;
    }

    public boolean atGoal() {
        return controller.atGoal();
    }

    public void reset(double currentMeters) {
        controller.reset(currentMeters);
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
