package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.claw.ClawStates;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.shoulder.ShoulderStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();
    private static ElevatorConfig config = Robot.getConfig().elevator;

    /* Check Elevator States */
    public static final Trigger isUp =
            elevator.atPercentage(config::getElevatorIsUpHeight, config::getTriggerTolerance);
    public static final Trigger isHigh =
            elevator.atPercentage(config::getElevatorIsHighHeight, config::getTriggerTolerance);
    public static final Trigger isHome =
            elevator.atRotations(config::getHome, config::getTriggerTolerance);

    public static final Trigger isHandOff =
            elevator.atRotations(config::getHandOff, config::getTriggerTolerance);

    public static final Trigger isL2Coral =
            elevator.atRotations(config::getL2Coral, config::getTriggerTolerance);
    public static final Trigger isL3Coral =
            elevator.atRotations(config::getL3Coral, config::getTriggerTolerance);
    public static final Trigger isL4Coral =
            elevator.atRotations(config::getL4Coral, config::getTriggerTolerance);

    public static final Trigger handOffAvoid =
            elevator.atRotations(config::getHandOffAvoid, config::getTriggerTolerance);

    public static final Trigger isL2Algae =
            elevator.atRotations(config::getL2Algae, config::getTriggerTolerance);
    public static final Trigger isL3Algae =
            elevator.atRotations(config::getL3Algae, config::getTriggerTolerance);

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(holdPosition().withName("Elevator.default"));
        // Removed run when disabled, so that the elevator doesn't jump up on enable
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        homeAll.and(ClawStates.hasCoral).debounce(0.2).whileTrue(home());

        homeAll.and(ShoulderStates.isLow.not()).whileTrue(home());
        // homeAll.and(Util.autoMode, ShoulderStates.isHome).whileTrue(slowHome());
        Robot.getOperator()
                .antiSecretClimb_LTRSup
                .whileTrue(move(config::getFullExtend, "Elevator.fullExtend"));

        stationIntaking
                .and(ShoulderStates.isLow.not())
                .whileTrue(
                        move(
                                config::getStationIntake,
                                // config::getStationExtendedIntake,
                                "Elevator.stationIntake"));
        stationIntaking.and(ShoulderStates.isLow.not()).onFalse(home());

        lollipopCoral.whileTrue(home());

        groundAlgae.whileTrue(move(config::getClawGroundAlgaeIntake, "Ground Algae"));
        // (groundAlgae.onFalse().and(ShoulderStates.isLow.not())).onTrue(home());
        groundCoral.whileTrue(home());

        (stagedCoral.or(stagedAlgae))
                .and(
                        actionState.not(),
                        actionPrepState.not().debounce(getActionPrepToActionTime()),
                        (Util.autoMode.not()))
                .whileTrue(move(config::getHome, "Elevator.Stage"));

        actionPrepState
                .and(L2Coral, ShoulderStates.isLow)
                .whileTrue(move(config::getHandOffAvoid, "Elevator.avoidHit").until(handOffAvoid));
        actionPrepState
                .and(L2Coral, ShoulderStates.isLow.not())
                .whileTrue(move(config::getL2Coral, "Elevator.L2Coral"));

        actionPrepState
                .and(L3Coral, ShoulderStates.isLow)
                .whileTrue(move(config::getHandOffAvoid, "Elevator.avoidHit").until(handOffAvoid));
        actionPrepState
                .and(L3Coral, ShoulderStates.isLow.not())
                .whileTrue(move(config::getL3Coral, "Elevator.L2Coral"));

        L2Coral.and(actionState).whileTrue(move(config::getL2Score, "Elevator.L2CoralScore"));
        L3Coral.and(actionState).whileTrue(move(config::getL3Score, "Elevator.L3CoralScore"));
        L4Coral.and(actionPrepState).whileTrue(move(config::getL4Coral, "Elevator.L4Coral"));
        L4Coral.and(actionState).whileTrue(move(config::getL4Score, "Elevator.L4CoralScore"));

        handOff.whileTrue(move(config::getHandOff, "Elevator.HandOff"));

        // L4Coral.and(actionPrepState, Util.autoMode)
        //         .whileTrue(
        //                 slowMove(config::getL4Coral, config::getExl4Coral,
        // "Elevator.slowL4Coral"));
        // L4Coral.and(actionState, Util.autoMode)
        //         .whileTrue(
        //                 slowMove(
        //                         config::getL4Score, config::getExl4Score,
        // "Elevator.L4CoralScore"));

        processorAlgae
                .and(actionPrepState)
                .whileTrue(move(config::getProcessorAlgae, "Elevator.processorAlgae"));
        processorAlgae
                .and(actionState)
                .whileTrue(move(config::getHome, "Elevator.processorAlgaeHome"));
        L2Algae.and(actionPrepState).whileTrue(move(config::getL2Algae, "Elevator.L2Algae"));
        L2Algae.and(actionState).whileTrue(move(config::getHome, "Elevator.L2AlgaeHome"));
        L3Algae.and(actionPrepState).whileTrue(move(config::getL3Algae, "Elevator.L3Algae"));
        L3Algae.and(actionState).whileTrue(move(config::getHome, "Elevator.L3AlgaeHome"));
        netAlgae.and(actionPrepState).whileTrue(move(config::getNetAlgae, "Elevator.NetAlgae"));
        // netAlgae.and(actionPrepState, Util.autoMode)
        //         .whileTrue(slowMove(config::getNetAlgae, "Elevator.NetAlgae"));

        Robot.getPilot().reZero_start.onTrue(elevator.resetToInitialPos());
    }

    public static DoubleSupplier getPosition() {
        return () -> elevator.getPositionRotations();
    }

    public static Command move(DoubleSupplier rotations, String name) {
        return elevator.move(rotations).withName(name);
    }

    public static Command slowMove(DoubleSupplier rotations, String name) {
        return elevator.slowMove(rotations, rotations).withName(name);
    }

    public static Command slowMove(
            DoubleSupplier rotations, DoubleSupplier exRotaitons, String name) {
        return elevator.slowMove(rotations, exRotaitons).withName(name);
    }

    private static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
    }

    private static Command home() {
        return move(config::getHome, "Elevator.home");
    }

    private static Command slowHome() {
        return slowMove(config::getHome, "Elevator.slowHome");
    }

    private static Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    private static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
