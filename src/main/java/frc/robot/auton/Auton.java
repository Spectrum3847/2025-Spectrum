package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.Field.ReefHeight;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Branch;
import frc.robot.subsystems.SuperStructure.Piece;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.Set;

/**
 * Autonomous routines. Paths come from PathPlanner; what the robot does along them comes from the
 * event markers in the path files, which {@link #bindEventMarkers} maps onto superstructure
 * requests. The marker names are the ones the 2025 paths already use.
 */
public class Auton {

    // Zoned: true while the path is inside the zone. Vision fuses in auto only there.
    public static final EventTrigger autonPoseUpdate = new EventTrigger("poseUpdate");

    private static final EventTrigger coral = new EventTrigger("coral");
    private static final EventTrigger algae = new EventTrigger("algae");
    private static final EventTrigger l1 = new EventTrigger("L1");
    private static final EventTrigger l2 = new EventTrigger("L2");
    private static final EventTrigger l3 = new EventTrigger("L3");
    private static final EventTrigger l4 = new EventTrigger("L4");
    private static final EventTrigger left = new EventTrigger("left");
    private static final EventTrigger right = new EventTrigger("right");
    private static final EventTrigger actionOn = new EventTrigger("actionOn");
    private static final EventTrigger actionOff = new EventTrigger("actionOff");
    private static final EventTrigger autoScoreMarker = new EventTrigger("autoScore");
    private static final EventTrigger sourceIntakeOn = new EventTrigger("sourceIntakeOn");
    private static final EventTrigger sourceIntakeOff = new EventTrigger("sourceIntakeOff");
    private static final EventTrigger groundIntake = new EventTrigger("groundIntake");
    private static final EventTrigger home = new EventTrigger("home");
    private static final EventTrigger clearStates = new EventTrigger("clearStates");
    private static final EventTrigger net = new EventTrigger("net");
    private static final EventTrigger reverse = new EventTrigger("reverse");

    /** Seconds auto waits for the score gate before scoring anyway. */
    private static final double AUTO_SCORE_TIMEOUT = 3.0;

    private final SuperStructure superStructure;
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    /** Set by the "algae" marker, cleared by "coral": what "actionOn"/"actionOff" act on. */
    private boolean algaeMode = false;

    public Auton(SuperStructure superStructure) {
        this.superStructure = superStructure;
        chooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
        addAuto("Left | 3 L4 Coral", worlds3Coral(false));
        addAuto("Right | 3 L4 Coral", worlds3Coral(true));
        addAuto("Center | 3 Net Algae", worlds3Algae());
        addAuto("Drive Forward", path("Drive Forward", false));
        SmartDashboard.putData("Auto Chooser", chooser);
        bindEventMarkers();
        Telemetry.print("Auton Initialized");
    }

    private void addAuto(String name, Command command) {
        chooser.addOption(name, command.withName(name));
    }

    public Command getAutonomousCommand() {
        Command selected = chooser.getSelected();
        return selected != null ? selected : Commands.none().withName("Do Nothing");
    }

    // ── Routines ───────────────────────────────────────────────────────────────

    private Command worlds3Coral(boolean mirrored) {
        return Commands.sequence(
                preload(),
                path("W3C-Start", mirrored),
                autoScore(),
                path("W3C-Leg1", mirrored),
                autoScore(),
                path("W3C-Leg2", mirrored),
                autoScore(),
                superStructure.setStateCommand(WantedSuperState.IDLE));
    }

    private Command worlds3Algae() {
        return Commands.sequence(
                preload(),
                path("W3A-Start", false),
                autoScore(),
                superStructure.setStateCommand(WantedSuperState.REEF_ALGAE),
                Commands.waitSeconds(1.0),
                superStructure.setStateCommand(WantedSuperState.IDLE),
                path("W3A-End", false));
    }

    /** The robot starts holding a coral, staged for L4. */
    private Command preload() {
        return Commands.runOnce(
                () -> {
                    superStructure.setHeldPiece(Piece.CORAL);
                    superStructure.setSelectedLevel(ReefHeight.L4);
                    algaeMode = false;
                });
    }

    /**
     * Aligns on the nearest reef face and scores when the gate opens; if it has not opened by the
     * timeout, scores where it is (the 2025 fallback) rather than leaving the coral in the claw.
     */
    public Command autoScore() {
        return Commands.defer(
                        () -> {
                            int before = superStructure.getCoralScoreCount();
                            return Commands.sequence(
                                    superStructure.setStateCommand(
                                            WantedSuperState.AUTO_SCORE_CORAL),
                                    Commands.waitUntil(
                                                    () ->
                                                            superStructure.getCoralScoreCount()
                                                                    > before)
                                            .withTimeout(AUTO_SCORE_TIMEOUT),
                                    Commands.either(
                                            Commands.none(),
                                            superStructure
                                                    .setStateCommand(WantedSuperState.SCORE_CORAL)
                                                    .andThen(
                                                            Commands.waitUntil(
                                                                    () ->
                                                                            superStructure
                                                                                            .getCoralScoreCount()
                                                                                    > before)),
                                            () -> superStructure.getCoralScoreCount() > before));
                        },
                        Set.of())
                .withName("Auton.autoScore");
    }

    private Command path(String autoName, boolean mirrored) {
        return new PathPlannerAuto(autoName, mirrored).withName(autoName);
    }

    // ── Event markers ──────────────────────────────────────────────────────────

    private void bindEventMarkers() {
        coral.onTrue(Commands.runOnce(() -> algaeMode = false));
        algae.onTrue(Commands.runOnce(() -> algaeMode = true));
        l1.onTrue(Commands.runOnce(() -> superStructure.setSelectedLevel(ReefHeight.L1)));
        l2.onTrue(Commands.runOnce(() -> superStructure.setSelectedLevel(ReefHeight.L2)));
        l3.onTrue(Commands.runOnce(() -> superStructure.setSelectedLevel(ReefHeight.L3)));
        l4.onTrue(Commands.runOnce(() -> superStructure.setSelectedLevel(ReefHeight.L4)));
        left.onTrue(Commands.runOnce(() -> superStructure.setSelectedBranch(Branch.LEFT)));
        right.onTrue(Commands.runOnce(() -> superStructure.setSelectedBranch(Branch.RIGHT)));

        actionOn.onTrue(
                Commands.runOnce(() -> superStructure.setWantedSuperState(actionOnState())));
        actionOff.onTrue(
                Commands.runOnce(() -> superStructure.setWantedSuperState(actionOffState())));
        autoScoreMarker.onTrue(superStructure.setStateCommand(WantedSuperState.AUTO_SCORE_CORAL));
        sourceIntakeOn.onTrue(superStructure.setStateCommand(WantedSuperState.STATION_CORAL));
        sourceIntakeOff.onTrue(superStructure.setStateCommand(WantedSuperState.IDLE));
        groundIntake.onTrue(superStructure.setStateCommand(WantedSuperState.GROUND_CORAL));
        home.onTrue(superStructure.setStateCommand(WantedSuperState.IDLE));
        clearStates.onTrue(superStructure.setStateCommand(WantedSuperState.IDLE));
        net.onTrue(superStructure.setStateCommand(WantedSuperState.STAGE_NET));
        reverse.onTrue(Commands.runOnce(() -> superStructure.setForceMirroredIntake(true)));
        reverse.onFalse(Commands.runOnce(() -> superStructure.setForceMirroredIntake(false)));
    }

    /** In the 2025 paths, algae at "L4" meant the net. */
    private WantedSuperState actionOnState() {
        if (!algaeMode) {
            return WantedSuperState.STAGE_CORAL;
        }
        return superStructure.getSelectedLevel() == ReefHeight.L4
                ? WantedSuperState.STAGE_NET
                : WantedSuperState.REEF_ALGAE;
    }

    private WantedSuperState actionOffState() {
        if (!algaeMode) {
            return WantedSuperState.SCORE_CORAL;
        }
        return superStructure.getSelectedLevel() == ReefHeight.L4
                ? WantedSuperState.SCORE_NET
                : WantedSuperState.IDLE;
    }

    // ── Checks ─────────────────────────────────────────────────────────────────

    private static final Alert missingAutoFiles =
            new Alert("Auto files missing on the rio; see the console", AlertType.kError);

    /**
     * Every PathPlanner auto the routines load must exist, with the exact case: the rio's
     * filesystem is case-sensitive and the Windows sim is not.
     */
    public static void verifyAutoFiles() {
        var names = AutoBuilder.getAllAutoNames();
        boolean missing = false;
        for (String n :
                new String[] {
                    "W3C-Start", "W3C-Leg1", "W3C-Leg2", "W3A-Start", "W3A-End", "Drive Forward"
                }) {
            if (!names.contains(n)) {
                Telemetry.print("Missing auto file: " + n + ".auto");
                missing = true;
            }
        }
        missingAutoFiles.set(missing);
    }
}
