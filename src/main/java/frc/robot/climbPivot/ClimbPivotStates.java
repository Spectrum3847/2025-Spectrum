package frc.robot.climbPivot;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.climbPivot.ClimbPivot.ClimbPivotConfig;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ClimbPivotStates {
    private static ClimbPivot climb = Robot.getClimbPivot();
    private static ClimbPivotConfig config = Robot.getConfig().climbPivot;

    public static final SpectrumState isLatched = climb.getLatched();

    public static void setupDefaultCommand() {
        climb.setDefaultCommand(log(climb.runHoldClimb().withName("Climb.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        Robot.getOperator().latchOpen_startUp.onTrue(openLatch());
        Robot.getOperator().latchCloser_startDown.onTrue(closeLatch());

        Robot.getOperator()
                .climbPrep_start
                .and(Robot.getOperator().noTriggers.not())
                .whileTrue(runClimb(() -> Robot.getOperator().getClimberTriggerAxis()));
        Robot.getOperator()
                .climbPrep_start
                .and(Robot.getOperator().leftTriggerOnly)
                .onTrue(openLatch());
        Robot.getOperator()
                .climbPrep_start
                .and(Robot.getOperator().rightTriggerOnly)
                .onTrue(closeLatch());
        // Robot.getOperator()
        //         .climbPrep_start
        //         .onTrue(
        //                 climb.moveToRotations(config::getMinRotations)
        //                         .withTimeout(1.5)
        //                         .andThen(closeLatch())
        //                         .withName("Climb.prepClimber"));

        // homeAll.and(climb.getLatched().not(), climbPrep.not()).whileTrue(log(home()));
        homeAll.whileTrue(log(home().alongWith(openLatch())));
    }

    public static Command runClimb(DoubleSupplier speed) {
        return climb.runPercentage(speed).withName("Climb.runClimb");
    }

    public static Command home() {
        return climb.moveToDegrees(config::getHome).withName("Climb.home");
    }

    public static DoubleSupplier getPosition() {
        return (() -> climb.getPositionPercentage());
    }

    public static Command coastMode() {
        return climb.coastMode().withName("Climb.CoastMode");
    }

    public static Command stopMotor() {
        return climb.runStop().withName("Climb.stop");
    }

    public static Command ensureBrakeMode() {
        return climb.ensureBrakeMode().withName("Climb.BrakeMode");
    }

    public static Command openLatch() {
        return climb.openLatch().withName("Climb.openLatch");
    }

    public static Command closeLatch() {
        return climb.closeLatch().withName("Climb.closeLatch");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
