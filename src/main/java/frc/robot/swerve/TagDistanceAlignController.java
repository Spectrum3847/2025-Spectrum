package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        controller.setTolerance(0.0);
        SmartDashboard.putData("tagDistanceController", controller);
    }

    public double calculate(double goalArea, double currentArea) {
        calculatedValue = controller.calculate(currentArea, goalArea);

        if (atGoal(currentArea)) {
            calculatedValue = 0;
            return calculatedValue;
        } else {
            return calculatedValue + (config.getKSdrive() * Math.signum(calculatedValue));
        }
    }

    public boolean atGoal(double current) {
        double goal = controller.getSetpoint();
        boolean atGoal = Math.abs(current - goal) < config.getTagDistanceTolerance();
        System.out.println("At Goal: " + atGoal + " Goal: " + goal + " Current: " + current);
        return atGoal;
    }

    public void reset(double current) {
        // controller.reset(current);
    }

    public void updatePID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
