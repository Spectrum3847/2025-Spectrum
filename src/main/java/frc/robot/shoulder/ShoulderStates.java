package frc.robot.shoulder;

import static frc.robot.RobotStates.*;
import static frc.robot.auton.Auton.autonScore;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ShoulderStates {
    private static Shoulder shoulder = Robot.getShoulder();
    private static ShoulderConfig config = Robot.getConfig().shoulder;
    public static final Trigger isHome = shoulder.atDegrees(config::getHome, config::getTolerance);

    public static void setupDefaultCommand() {
        shoulder.setDefaultCommand(
                log(shoulder.runHoldShoulder().withName("Shoulder.HoldDefault")));
        // shoulder.runStop());
    }

    public static void setStates() {
        homeAll.whileTrue(home());
        coastMode.onTrue(log(coastMode()).ignoringDisable(true));
        coastMode.onFalse(log(ensureBrakeMode()));

        Robot.getOperator().antiSecretClimb_LTRSup.whileTrue(shoulder.move(config::getNetAlgae));

        stationIntaking.whileTrue(
                move(
                        config::getStationIntake,
                        // config::getStationExtendedIntake,
                        "Shoulder.stationIntake"));
        stationIntaking.or(groundCoral, groundAlgae).onFalse(home());

        groundCoral.whileTrue(move(config::getGroundCoralIntake, "Shoulder.groundCoral"));
        groundAlgae.whileTrue(move(config::getGroundAlgaeIntake, "Shoulder.groundAlgae"));

        Robot.getPilot()
                .photonRemoveL2Algae
                .whileTrue(move(config::getL2Algae, "Shoulder.L2Algae"));
        Robot.getPilot()
                .photonRemoveL3Algae
                .whileTrue(move(config::getL3Algae, "Shoulder.L3Algae"));
        Robot.getPilot()
                .photonRemoveL2Algae
                .or(Robot.getPilot().photonRemoveL3Algae)
                .onFalse(home());

        stagedCoral.and(actionState.not()).whileTrue(move(config::getHome, "Shoulder.Stage"));

        L1Coral.and(actionState.or(actionPrepState))
                .whileTrue(move(config::getL1Coral, config::getExl1Coral, "Shoulder.L1Coral"));
        L2Coral.and(actionPrepState)
                .whileTrue(
                        move(
                                config::getL2Coral,
                                config::getExl2Coral,
                                "Shoulder.L2Coral.prescore"));
        L2Coral.and(actionState)
                .whileTrue(
                        move(
                                config::getL2Score,
                                config::getExl2Score,
                                config::getScoreDelay,
                                "Shoulder.L2Coral.score"));
        L3Coral.and(actionPrepState)
                .whileTrue(
                        move(
                                config::getL3Coral,
                                config::getExl3Coral,
                                "Shoulder.L3Coral.prescore"));
        L3Coral.and(actionState)
                .whileTrue(
                        move(
                                config::getL3Score,
                                config::getExl3Score,
                                config::getScoreDelay,
                                "Shoulder.L3Coral.score"));
        L4Coral.and(actionPrepState)
                .whileTrue(
                        move(
                                config::getL4Coral,
                                config::getExl4Coral,
                                "Shoulder.L4Coral.prescore"));
        L4Coral.and(actionState)
                .whileTrue(
                        move(
                                config::getL4CoralScore,
                                config::getExl4Score,
                                config::getScoreDelay,
                                "Shoulder.L4Coral.score"));

        shoulderL4.onTrue(move(config::getExl4Coral, "Shoulder.L4Coral.prescore"));

        // algae
        processorAlgae
                .and(actionPrepState)
                .whileTrue(move(config::getProcessorAlgae, "Shoulder.processorAlgae"));
        processorAlgae
                .and(actionState)
                .whileTrue(move(config::getHome, "Shoulder.processorAlgaeHome"));
        L2Algae.and(actionPrepState).whileTrue(move(config::getL2Algae, "Shoulder.L2Algae"));
        L2Algae.and(actionState).whileTrue(move(config::getHome, "Shoulder.L2AlgaeHome"));
        L3Algae.and(actionPrepState).whileTrue(move(config::getL3Algae, "Shoulder.L3Algae"));
        L3Algae.and(actionState).whileTrue(move(config::getHome, "Shoulder.L3AlgaeHome"));
        netAlgae.and(actionPrepState.or(actionState).not())
                .whileTrue(move(config::getHome, "Shoulder.netAlgaePrep"));
        netAlgae.and(actionPrepState.or(actionState))
                .whileTrue(move(config::getNetAlgae, "Shoulder.netAlgae"));

        Robot.getPilot().reZero_start.onTrue(shoulder.resetToIntialPos());
        Robot.getOperator()
                .climbPrep_start
                .whileTrue(move(config::getClimbPrep, "Shoulder.startClimb"));

        autonScore.onTrue(new WaitCommand(4.0).andThen(move(() -> 180, "Shoulder.homeAuto")));
    }

    public static Command runShoulder(DoubleSupplier speed) {
        return shoulder.runPercentage(speed).withName("Shoulder.runShoulder");
    }

    public static Command home() {
        return shoulder.moveToDegrees(config::getHome).withName("Shoulder.home");
    }

    public static DoubleSupplier getPosition() {
        return () -> (shoulder.getPositionDegrees() + 90);
    }

    public static Command move(DoubleSupplier degrees, String name) {
        return shoulder.move(degrees, degrees).withName(name);
    }

    public static Command move(DoubleSupplier degrees, DoubleSupplier exDegrees, String name) {
        return shoulder.move(degrees, exDegrees).withName(name);
    }

    public static Command move(
            DoubleSupplier degrees, DoubleSupplier exDegrees, DoubleSupplier delay, String name) {
        return new WaitCommand(delay.getAsDouble())
                .andThen(move(degrees, exDegrees, name).withName(name));
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
