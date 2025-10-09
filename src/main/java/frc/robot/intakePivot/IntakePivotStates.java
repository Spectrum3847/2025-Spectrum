package frc.robot.intakePivot;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.intakePivot.IntakePivot.IntakePivotConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class IntakePivotStates {
    private static IntakePivot intakePivot = Robot.getIntakePivot();
    private static IntakePivotConfig config = Robot.getConfig().intakePivot;
    public static final Trigger isHome =
            intakePivot.atDegrees(config::getHome, config::getTolerance);

    public static final Trigger isHandOff =
            intakePivot.atDegrees(config::getHandOff, config::getTolerance);

    public static void setupDefaultCommand() {
        intakePivot.setDefaultCommand(
                log(intakePivot.runHoldIntakePivot().withName("IntakePivot.HoldDefault")));
        // IntakePivot.runStop());
    }

    public static void setStates() {
        homeAll.debounce(0.3).whileTrue(home());
        coastMode.onTrue(log(coastMode()).ignoringDisable(true));
        coastMode.onFalse(log(ensureBrakeMode()));

        groundCoral.whileTrue(move(config::getGroundCoralIntake, "IntakePivot.groundCoral"));

        L1Coral.whileTrue(move(config::getL1, "IntakePivot.L1"));

        handOff.whileTrue(move(config::getHandOff, "IntakePivot.Stage"));

        actionPrepState.and(l1.not()).debounce(0.2).onTrue(home());

        Robot.getPilot().reZero_start.onTrue(intakePivot.resetToIntialPos());
    }

    public static Command runIntakePivot(DoubleSupplier speed) {
        return intakePivot.runPercentage(speed).withName("IntakePivot.runIntakePivot");
    }

    public static Command home() {
        return intakePivot.moveToDegrees(config::getHome).withName("IntakePivot.home");
    }

    public static DoubleSupplier getPosition() {
        return () -> (intakePivot.getPositionDegrees() + 90);
    }

    public static Command move(DoubleSupplier degrees, String name) {
        return intakePivot.move(degrees).withName(name);
    }

    public static Command coastMode() {
        return intakePivot.coastMode().withName("IntakePivot.CoastMode");
    }

    public static Command stopMotor() {
        return intakePivot.runStop().withName("IntakePivot.stop");
    }

    public static Command ensureBrakeMode() {
        return intakePivot.ensureBrakeMode().withName("IntakePivot.BrakeMode");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
