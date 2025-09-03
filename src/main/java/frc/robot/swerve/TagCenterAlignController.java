package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class TagCenterAlignController {
    Swerve swerve;
    SwerveConfig config;
    PIDController controller;
    Constraints constraints;

    double calculatedValue = 0;
    double maxVelocity;

    public TagCenterAlignController(SwerveConfig config) {
        this.config = config;
        maxVelocity = Robot.getConfig().swerve.getSpeedAt12Volts().baseUnitMagnitude() * 0.5;
        controller =
                new PIDController(
                        config.getKPTagCenterController(),
                        config.getKITagCenterController(),
                        config.getKDTagCenterController());
        controller.setTolerance(0.0);

        SmartDashboard.putData("tagCenterController", controller);
    }

    public double calculate(double goalMeters, double currentMeters) {
        calculatedValue = controller.calculate(currentMeters, goalMeters);

        if (atGoal(currentMeters)) {
            calculatedValue = 0;
            return calculatedValue;
        } else {
            if (Math.abs(calculatedValue) > maxVelocity) {
                calculatedValue = maxVelocity * Math.signum(calculatedValue);
            }
            return calculatedValue + (config.getKSdrive() * Math.signum(calculatedValue));
        }
    }

    public boolean atGoal(double current) {
        double goal = controller.getSetpoint();
        boolean atGoal = Math.abs(current - goal) < config.getTagCenterTolerance();
        System.out.println("At Goal: " + atGoal + " Goal: " + goal + " Current: " + current);
        return atGoal;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
