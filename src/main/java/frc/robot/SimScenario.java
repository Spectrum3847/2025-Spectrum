package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.reefscape.Field;
import frc.reefscape.Field.ReefHeight;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Branch;
import frc.robot.subsystems.SuperStructure.Piece;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * A scripted regression run for the simulator: set {@code SIM_SCENARIO=1} and run {@code ./gradlew
 * simulateJava}. It places the robot near the reef, runs teleop auto-scores from awkward starts, a
 * ground intake and the 3-coral auto, prints PASS/FAIL for each, and exits with the failure count.
 * Watch it in AdvantageScope while it runs.
 *
 * <p>Deliberately not a command: mode changes cancel every scheduled command, which would cancel
 * the script. It is stepped from {@link Robot#simulationPeriodic()} instead.
 */
final class SimScenario {

    /** One step: start it, then wait until done or out of time, then finish it. */
    private record Step(Runnable start, BooleanSupplier done, double timeout, Runnable end) {}

    private final List<Step> steps = new ArrayList<>();
    private int index = -1;
    private double stepStart;
    private int failures = 0;
    private int scoresBefore;
    private double scoreStart;
    private double tArm, tNear, tGate, tStage;
    private double worstLoopMs, loopMsSum;
    private int loops;

    SimScenario() {
        wait(2.0);
        teleopScore(18, 1.8, 0.0, 0, ReefHeight.L4, Branch.LEFT);
        teleopScore(18, 1.5, 0.6, 25, ReefHeight.L2, Branch.RIGHT);
        teleopScore(20, 1.2, -0.4, -30, ReefHeight.L3, Branch.LEFT);
        groundIntake();
        auto("Left | 3 L4 Coral", 3);
        step(this::finish, () -> false, 1e9, () -> {});
    }

    void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now > 3) {
            worstLoopMs = Math.max(worstLoopMs, Robot.getLoopMs());
            loopMsSum += Robot.getLoopMs();
            loops++;
        }
        if (Double.isNaN(tStage) && ss().getElevator().getTargetRotations() > 1) {
            tStage = now - scoreStart;
        }
        if (Double.isNaN(tArm)
                && ss().isArmAtTarget()
                && ss().getElevator().getTargetRotations() > 1) {
            tArm = now - scoreStart;
        }
        if (Double.isNaN(tNear)
                && Robot.getVision().getReefTagTracker().getPose().isPresent()
                && ss().getCurrentSuperState().toString().equals("ALIGN_CORAL")
                && Math.abs(Robot.getSwerve().getCurrentRobotChassisSpeeds().vxMetersPerSecond)
                        < 0.05
                && now - scoreStart > 0.3) {
            tNear = now - scoreStart;
        }
        if (Double.isNaN(tGate) && ss().isScoreGateOpen()) {
            tGate = now - scoreStart;
        }
        if (index >= 0) {
            Step s = steps.get(index);
            if (!s.done().getAsBoolean() && now - stepStart < s.timeout()) {
                return;
            }
            s.end().run();
        }
        index++;
        stepStart = now;
        steps.get(index).start().run();
    }

    private void step(Runnable start, BooleanSupplier done, double timeout, Runnable end) {
        steps.add(new Step(start, done, timeout, end));
    }

    private void wait(double seconds) {
        step(() -> {}, () -> false, seconds, () -> {});
    }

    private static SuperStructure ss() {
        return Robot.getSuperStructure();
    }

    private static Pose2d truePose() {
        return Robot.getSwerve().getMapleSim().mapleSimDrive.getSimulatedDriveTrainPose();
    }

    private void teleopScore(
            int tag, double out, double side, double yawDeg, ReefHeight level, Branch branch) {
        String name = String.format("Auto-score %s %s at tag %d", level, branch, tag);
        Pose2d start =
                Field.tagPose(tag)
                        .transformBy(
                                new Transform2d(out, side, Rotation2d.fromDegrees(180 + yawDeg)));
        // Placed twice: the first reset after boot can land before the sim gyro has caught up.
        step(
                () -> {
                    setMode(false, false);
                    Robot.getSwerve().resetPose(start);
                },
                () -> false,
                0.5,
                () -> Robot.getSwerve().resetPose(start));
        // Enable a step ahead: teleopInit resets the wanted state to idle.
        step(() -> setMode(true, false), () -> false, 0.1, () -> {});
        step(
                () -> {
                    scoresBefore = ss().getCoralScoreCount();
                    ss().setHeldPiece(Piece.CORAL);
                    ss().setSelectedLevel(level);
                    ss().setSelectedBranch(branch);
                    scoreStart = Timer.getFPGATimestamp();
                    tArm = tNear = tGate = tStage = Double.NaN;
                    ss().setWantedSuperState(WantedSuperState.AUTO_SCORE_CORAL);
                },
                () -> ss().getCoralScoreCount() > scoresBefore,
                8.0,
                () -> {
                    Pose2d goal =
                            Field.tagPose(tag)
                                    .transformBy(
                                            new Transform2d(
                                                    Units.inchesToMeters(29.5 / 2 + 10),
                                                    0,
                                                    Rotation2d.k180deg));
                    Transform2d err = new Transform2d(goal, truePose());
                    boolean scored = ss().getCoralScoreCount() > scoresBefore;
                    boolean onTarget =
                            Math.abs(err.getX()) < Units.inchesToMeters(1.5)
                                    && Math.abs(err.getY()) < Units.inchesToMeters(1.5)
                                    && Math.abs(err.getRotation().getDegrees()) < 3;
                    report(
                            name,
                            scored && onTarget,
                            String.format(
                                    "stage %.2f, drive settled %.2f, arm %.2f, gate %.2f;"
                                            + " scored=%b after %.2f s; true error along %.2f in,"
                                            + " lateral %.2f in, heading %.2f deg",
                                    tStage,
                                    tNear,
                                    tArm,
                                    tGate,
                                    scored,
                                    Timer.getFPGATimestamp() - scoreStart,
                                    Units.metersToInches(err.getX()),
                                    Units.metersToInches(err.getY()),
                                    err.getRotation().getDegrees()));
                });
        wait(1.0);
    }

    private void groundIntake() {
        step(
                () -> {
                    ss().setHeldPiece(Piece.NONE);
                    ss().setWantedSuperState(WantedSuperState.GROUND_CORAL);
                },
                () -> false,
                1.0,
                () -> ss().setWantedSuperState(WantedSuperState.IDLE));
        step(
                () -> {},
                () -> false,
                0.2,
                () ->
                        report(
                                "Ground coral intake",
                                ss().getHeldPiece() == Piece.CORAL,
                                "held " + ss().getHeldPiece()));
        wait(1.5);
    }

    private void auto(String name, int expectedScores) {
        step(
                () -> {
                    setMode(false, false);
                    NetworkTableInstance.getDefault()
                            .getTable("SmartDashboard/Auto Chooser")
                            .getEntry("selected")
                            .setString(name);
                },
                () -> false,
                1.5,
                () -> {});
        step(
                () -> {
                    scoresBefore = ss().getCoralScoreCount();
                    setMode(true, true);
                },
                () -> false,
                15.3,
                () -> {
                    int scored = ss().getCoralScoreCount() - scoresBefore;
                    setMode(false, false);
                    report("Auto " + name, scored >= expectedScores, scored + " coral scored");
                });
    }

    private static void setMode(boolean enabled, boolean auto) {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(auto);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
    }

    private void report(String name, boolean pass, String detail) {
        if (!pass) {
            failures++;
        }
        Telemetry.print(
                (pass ? "SCENARIO PASS: " : "SCENARIO FAIL: ") + name + " -- " + detail,
                PrintPriority.HIGH);
    }

    private void finish() {
        Telemetry.print(
                String.format(
                        "SCENARIO LOOP: mean %.2f ms, worst %.2f ms",
                        loopMsSum / loops, worstLoopMs),
                PrintPriority.HIGH);
        Telemetry.print(
                "SCENARIO DONE: " + (failures == 0 ? "all passed" : failures + " failed"),
                PrintPriority.HIGH);
        System.exit(failures);
    }
}
