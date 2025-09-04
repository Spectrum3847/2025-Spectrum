package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotationController {
    private final ProfiledPIDController motionController;
    private final PIDController holdController;
    private final SwerveConfig config;

    private double lastOutput = 0.0;
    private final double deadband = 1e-3;

    public RotationController(SwerveConfig config) {
        this.config = config;

        motionController =
                new ProfiledPIDController(
                        config.getKPRotationController(),
                        config.getKIRotationController(),
                        config.getKDRotationController(),
                        new Constraints(
                                config.getMaxAngularVelocity(),
                                config.getMaxAngularAcceleration()));

        motionController.enableContinuousInput(-Math.PI, Math.PI);
        motionController.setTolerance(config.getRotationTolerance());
        SmartDashboard.putData("Rotation Controller", motionController);

        holdController =
                new PIDController(
                        config.getKPHoldController(),
                        config.getKIHoldController(),
                        config.getKDHoldController());

        holdController.enableContinuousInput(-Math.PI, Math.PI);
        holdController.setTolerance(config.getRotationTolerance() / 2.0);
        SmartDashboard.putData("Hold Controller", holdController);
    }

    public double calculate(double goalRadians, double currentRadians, boolean useHold) {
        double output;

        if (useHold && atGoal()) {
            output = holdController.calculate(currentRadians, goalRadians);
        } else {
            output = motionController.calculate(currentRadians, goalRadians);
        }

        // SmartDashboard.putNumber("Rotation Output (raw)", output);

        if (Math.abs(output) > deadband) {
            output += config.getKSsteer() * Math.signum(output);
        }

        lastOutput = output;
        // SmartDashboard.putNumber("Rotation Output (final)", output);
        // SmartDashboard.putBoolean("Rotation At Goal", motionController.atGoal());

        return output;
    }

    public double calculate(double goalRadians, double currentRadians) {
        return calculate(goalRadians, currentRadians, true);
    }

    public boolean atGoal() {
        return motionController.atGoal();
    }

    public boolean atSetpoint() {
        return motionController.atSetpoint();
    }

    public void reset(double currentRadians) {
        motionController.reset(currentRadians);
        holdController.reset();
        lastOutput = 0.0;
    }

    public void updatePID(double kP, double kI, double kD) {
        motionController.setPID(kP, kI, kD);
    }
}
