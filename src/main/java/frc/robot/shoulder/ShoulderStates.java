package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elevator.ElevatorStates;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;

public class ShoulderStates {
    private static Shoulder shoulder = Robot.getShoulder();
    private static ShoulderConfig config = Robot.getConfig().shoulder;
    public static final Trigger isHome = shoulder.atDegrees(config::getHome, config::getTolerance);
    public static final Trigger isNetPosition = shoulder.atDegrees(config::getNetAlgae, () -> 90);
    public static final Trigger isAutonNetPosition =
            shoulder.aboveDegrees(config::getAutonShoulderNetChecker, config::getTolerance);

    public static final Trigger isLow = shoulder.atDegrees(config::getIsLow, () -> 90);
    //     .or(
    //             shoulder.atDegrees(() -> -config.getIsLow(), () -> 90)
    //                     .and(reverse));

    public static final Trigger isHandOff =
            shoulder.atDegrees(config::getHandOff, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getHandOff(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL2Coral =
            shoulder.atDegrees(config::getL2Coral, () -> (config.getTolerance() + 3))
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getL2Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL3Coral =
            shoulder.atDegrees(config::getL3Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getL3Coral(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL4Coral =
            shoulder.atDegrees(config::getL4Coral, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getL4Coral(), config::getTolerance)
                                    .and(reverse));

    public static final Trigger isL2Algae =
            shoulder.atDegrees(config::getL2Algae, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getL2Algae(), config::getTolerance)
                                    .and(reverse));
    public static final Trigger isL3Algae =
            shoulder.atDegrees(config::getL3Algae, config::getTolerance)
                    .and(reverse.not())
                    .or(
                            shoulder.atDegrees(() -> -config.getL3Algae(), config::getTolerance)
                                    .and(reverse));

    public static void setupDefaultCommand() {
        shoulder.setDefaultCommand(
                log(shoulder.runHoldShoulder().withName("Shoulder.HoldDefault")));
        // shoulder.runStop()););
    }

    public static void setStates() {
        Robot.getOperator()
                .climbPrep_start
                .onTrue(moveWithoutReverse(config::getClimbPrep, "Shoulder.climbPrep"));

        isHandOff.onTrue(shoulderHandOff.setTrue());
        isHandOff.onFalse(shoulderHandOff.setFalse());

        homeAll.whileTrue(home());
        homeAll.and(Util.autoMode).whileTrue(slowHome());
        coastMode.onTrue(log(coastMode()).ignoringDisable(true));
        coastMode.onFalse(log(ensureBrakeMode()));

        lollipopCoral.whileTrue(
                moveWithoutReverse(config::getLollipopCoral, "Shoulder.lollipopCoral"));

        // stationIntaking.whileTrue(move(config::getStationIntake, "Shoulder.stationIntake"));
        // stationIntaking.or(groundAlgae).onFalse(home());

        groundAlgae.whileTrue(
                moveWithoutReverse(config::getGroundAlgaeIntake, "Shoulder.groundAlgae"));
        groundAlgae.onFalse(moveWithoutReverse(config::getHome, "Shoulder.groundAlgae"));

        handOff.and(ElevatorStates.isHandOff)
                .whileTrue(moveWithoutReverse(config::getHandOff, "Shoulder.handOff"));

        stagedCoral
                .and(
                        actionState.not(),
                        actionPrepState.not().debounce(getActionPrepToActionTime()),
                        Util.autoMode.not())
                .whileTrue(move(config::getHome, "Shoulder.Stage"));

        L2Coral.and(actionPrepState)
                .debounce(0.3)
                .whileTrue(
                        moveWithoutReverse(config::getL2Coral, "Shoulder.L2Coral.prescoreRepeat"));
        L2Coral.and(actionState)
                .whileTrue(moveWithoutReverse(config::getL2Score, "Shoulder.L2Coral.score"));
        L3Coral.and(actionPrepState)
                .debounce(0.3)
                .whileTrue(
                        moveWithoutReverse(config::getL3Coral, "Shoulder.L3Coral.prescoreRepeat"));
        L3Coral.and(actionState)
                .whileTrue(moveWithoutReverse(config::getL3Score, "Shoulder.L3Coral.score"));
        L4Coral.and(actionPrepState)
                .debounce(0.3)
                .whileTrue(
                        moveWithoutReverse(config::getL4Coral, "Shoulder.L4Coral.prescoreRepeat"));
        L4Coral.and(actionState)
                .whileTrue(moveWithoutReverse(config::getL4CoralScore, "Shoulder.L4Coral.score"));

        L4Coral.and(actionPrepState, Util.autoMode, autonL4reverse)
                .debounce(0.3)
                .whileTrue(
                        moveWithoutReverse(
                                () -> -config.getL4Coral(), "Shoulder.L4Coral.prescoreRepeat"));
        L4Coral.and(actionState, Util.autoMode, autonL4reverse)
                .whileTrue(
                        moveWithoutReverse(
                                () -> -config.getL4CoralScore(), "Shoulder.L4Coral.score"));
        // L4Coral.and(actionPrepState, Util.autoMode)
        //         .whileTrue(slowMove(config::getL4Coral, "Shoulder.L4Coral.slowPrescore"));

        // algae
        processorAlgae
                .and(actionPrepState)
                .whileTrue(move(config::getProcessorAlgae, "Shoulder.processorAlgae"));
        processorAlgae
                .and(actionState)
                .whileTrue(move(config::getHome, "Shoulder.processorAlgaeHome"));

        L2Algae.and(actionPrepState)
                .whileTrue(moveWithoutReverse(config::getL2Algae, "Shoulder.L2Algae"));
        L2Algae.and(actionState)
                .whileTrue(moveWithoutReverse(config::getHome, "Shoulder.L2AlgaeHome"));
        L3Algae.and(actionPrepState)
                .whileTrue(moveWithoutReverse(config::getL3Algae, "Shoulder.L3Algae"));
        L3Algae.and(actionState)
                .whileTrue(moveWithoutReverse(config::getHome, "Shoulder.L3AlgaeHome"));

        netAlgae.and((actionPrepState.or(actionState).not()), (Util.autoMode.not()))
                .whileTrue(move(config::getHome, "Shoulder.netAlgaePrep"));
        netAlgae.and(actionPrepState.or(actionState))
                .whileTrue(move(config::getNetAlgae, "Shoulder.netAlgae"));

        Robot.getPilot().reZero_start.onTrue(shoulder.resetToIntialPos());
    }

    public static Command runShoulder(DoubleSupplier speed) {
        return shoulder.runPercentage(speed).withName("Shoulder.runShoulder");
    }

    public static Command home() {
        return shoulder.moveToDegrees(config::getHome).withName("Shoulder.home");
    }

    public static Command slowHome() {
        return shoulder.slowMove(config::getHome).withName("Shoulder.slowHome");
    }

    public static DoubleSupplier getPosition() {
        return () -> (shoulder.getPositionDegrees() + 90);
    }

    public static Command move(DoubleSupplier degrees, String name) {
        return shoulder.move(degrees, degrees).withName(name);
    }

    public static Command slowMove(DoubleSupplier degrees, String name) {
        return shoulder.slowMove(degrees).withName(name);
    }

    public static Command move(DoubleSupplier degrees, DoubleSupplier exDegrees, String name) {
        return shoulder.move(degrees, exDegrees).withName(name);
    }

    public static Command move(
            DoubleSupplier degrees, DoubleSupplier exDegrees, DoubleSupplier delay, String name) {
        return (new WaitCommand(delay.getAsDouble())
                .andThen(move(degrees, exDegrees, name))
                .withName(name));
    }

    public static Command slowMove(
            DoubleSupplier degrees, DoubleSupplier exDegrees, DoubleSupplier delay, String name) {
        return new WaitCommand(delay.getAsDouble()).andThen(slowMove(degrees, name).withName(name));
    }

    public static Command moveWithoutReverse(DoubleSupplier degrees, String name) {
        return shoulder.moveWithoutReverse(degrees, degrees).withName(name);
    }

    public static Command coastMode() {
        return shoulder.coastMode().withName("Shoulder.CoastMode");
    }

    public static Command stopMotor() {
        return shoulder.runStop().withName("Shoulder.stop");
    }

    public static Command ensureBrakeMode() {
        return shoulder.ensureBrakeMode().withName("Shoulder.BrakeMode");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
