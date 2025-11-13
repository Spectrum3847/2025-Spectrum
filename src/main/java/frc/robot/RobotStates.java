package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.FieldHelpers;
import frc.reefscape.Zones;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.ElevatorStates;
import frc.robot.intake.IntakeStates;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.shoulder.ShoulderStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumState;
import lombok.Getter;

public class RobotStates {
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Coordinator coordinator = Robot.getCoordinator();

    @Getter private static State appliedState = State.CORAL_L4_READY;

    @Getter private static double scoreTime = 2.0;
    @Getter private static double autonScoreTime = 0.75;
    @Getter private static double twistAtReefDelay = 0.2;
    @Getter private static double scoreAfterAlignTime = 0.03;
    @Getter private static double autonScoreAfterAlignTime = 0.05;
    @Getter private static double actionPrepToActionTime = 0.05;

    // Robot States
    // These are states that aren't directly tied to hardware or buttons, etc.
    // If they should be set by multiple Triggers do that in SetupStates()
    public static final SpectrumState reverse = new SpectrumState("reverse");
    public static final SpectrumState rightScore = new SpectrumState("rightScore");
    //     public static final SpectrumState aligned = new SpectrumState("aligned");
    //     public static final SpectrumState coastMode = new SpectrumState("coast");
    //     public static final SpectrumState coral = new SpectrumState("coral");
    //     public static final SpectrumState algae = new SpectrumState("algae");
    //     public static final SpectrumState l1 = new SpectrumState("l1");
    //     public static final SpectrumState l2 = new SpectrumState("l2");
    //     public static final SpectrumState l3 = new SpectrumState("l3");
    //     public static final SpectrumState l4 = new SpectrumState("l4");
    //     public static final SpectrumState shrinkState = new SpectrumState("extendedStates");
    //     public static final SpectrumState actionPrepState = new SpectrumState("actionPrepState");
    //     public static final SpectrumState actionState = new SpectrumState("actionState");
    //     public static final SpectrumState homeAll = new SpectrumState("homeAll");
    //     public static final SpectrumState autonStationIntake = new
    // SpectrumState("autonStationIntake");
    //     public static final SpectrumState twistAtReef = new SpectrumState("twistCoralReef");
    //     public static final SpectrumState autoScoreMode = new SpectrumState("autoScoreMode");
    //     public static final SpectrumState autonAutoScoreMode = new
    // SpectrumState("autonAutoScoreMode");
    //     public static final SpectrumState coralScoring = new SpectrumState("coralScoring");

    /**
     * Define Robot States here and how they can be triggered States should be triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving the state
     * RobotType Triggers
     */
    public static final Trigger pm = new Trigger(() -> Rio.id == Rio.PM_2025);

    public static final Trigger photon = new Trigger(() -> Rio.id == Rio.PHOTON_2025);
    public static final Trigger sim = new Trigger(RobotBase::isSimulation);

    // Intake Triggers
    public static final Trigger stationIntaking = pilot.stationIntake_LT;
    // public static final Trigger stationExtendedIntaking = pilot.stationIntakeExtended_LT_RB;
    public static final Trigger groundAlgae = pilot.groundAlgae_RT;
    public static final Trigger groundCoral = pilot.groundCoral_LB_LT;
    public static final Trigger intaking = stationIntaking.or(groundAlgae, groundCoral);

    // climb Triggers
    public static final Trigger climbPrep = operator.climbPrep_start;
    public static final Trigger climbFinish = pilot.climbRoutine_start;

    // mechanism preset Triggers (Wrist, Elevator, etc.)
    public static final Trigger shrink = pilot.fn;
    public static final Trigger processorAlgae = operator.L1.and(operator.algaeStage);
    public static final Trigger L2Algae = operator.L2.and(operator.algaeStage).or(autonLowAlgae);
    public static final Trigger L3Algae = operator.L3.and(operator.algaeStage).or(autonHighAlgae);
    public static final Trigger netAlgae = operator.L4.and(operator.algaeStage).or(autonNet);
    public static final Trigger stagedAlgae = processorAlgae.or(L2Algae, L3Algae, netAlgae);

    public static final Trigger L1Coral = operator.L1.and(operator.coralStage).or(autonL1);
    public static final Trigger L2Coral = operator.L2.and(operator.coralStage);
    public static final Trigger L3Coral = operator.L3.and(operator.coralStage);
    public static final Trigger L4Coral = operator.L4.and(operator.coralStage);
    public static final Trigger branch = L2Coral.or(L3Coral, L4Coral);
    public static final Trigger stagedCoral = L1Coral.or(L2Coral, L3Coral, L4Coral);

    public static final Trigger staged = stagedAlgae.or(stagedCoral);

    public static final Trigger atL1Coral =
            ElbowStates.isL1Coral.and(ShoulderStates.isL1Coral, ElevatorStates.isL1Coral);
    public static final Trigger atL2Coral =
            ElbowStates.isL2Coral.and(ShoulderStates.isL2Coral, ElevatorStates.isL2Coral);
    public static final Trigger atL3Coral =
            ElbowStates.isL3Coral.and(ShoulderStates.isL3Coral, ElevatorStates.isL3Coral);
    public static final Trigger atL4Coral =
            ElbowStates.isL4Coral
                    .and(ShoulderStates.isL4Coral, ElevatorStates.isL4Coral)
                    .or(autonAtL4Coral);

    public static final Trigger atL2Algae =
            ElbowStates.isL2Algae.and(ShoulderStates.isL2Algae, ElevatorStates.isL2Algae);
    public static final Trigger atL3Algae =
            ElbowStates.isL3Algae.and(ShoulderStates.isL3Algae, ElevatorStates.isL3Algae);

    public static final Trigger completeStagedCoral = atL1Coral.or(atL2Coral, atL3Coral, atL4Coral);
    public static final Trigger completeStagedAlgae = atL2Algae.or(atL3Algae);

    public static final Trigger toggleReverse = pilot.toggleReverse.or(operator.toggleReverse);

    // pose Triggers
    public static final Trigger poseReversal =
            new Trigger(
                    () -> FieldHelpers.reverseRotationBlue() == Zones.blueFieldSide.getAsBoolean());

    public static final Trigger isAtHome =
            ElevatorStates.isHome.and(ElbowStates.isHome, ShoulderStates.isHome);

    // reset triggers
    public static final Trigger homeElevator = operator.homeElevator_A;

    public static final Trigger hasCoral = new Trigger(IntakeStates.hasCoral);
    public static final Trigger hasAlgae = new Trigger(IntakeStates.hasAlgae);
    public static final Trigger hasGamePiece = hasCoral.or(hasAlgae);

    // Setup any binding to set states
    public static void setupStates() {

        // HOME STATES
        pilot.home_select.or(operator.home_select).onTrue(applyState(State.REHOME));
        pilot.home_select.or(operator.home_select).onFalse(applyState(State.IDLE_EMPTY));

        // IDLE STATES
        isAtHome.and(hasGamePiece.not()).onTrue(applyState(State.IDLE_EMPTY));
        isAtHome.and(hasCoral).onTrue(applyState(State.IDLE_CORAL));
        isAtHome.and(hasAlgae).onTrue(applyState(State.IDLE_ALGAE));

        // INTAKE STATES
        stationIntaking.onTrue(applyState(State.CORAL_INTAKE_HUMAN));
        stationIntaking.onFalse(applyState(State.IDLE_CORAL));
        groundCoral.onTrue(applyState(State.CORAL_INTAKE_FLOOR));
        groundCoral.onFalse(applyState(State.IDLE_CORAL));
        groundAlgae.onTrue(applyState(State.ALGAE_INTAKE_FLOOR));
        groundAlgae.onFalse(applyState(State.IDLE_ALGAE));
        L2Algae.onTrue(applyState(State.ALGAE_INTAKE_L2));
        L2Algae.onFalse(applyState(State.IDLE_ALGAE));
        L3Algae.onTrue(applyState(State.ALGAE_INTAKE_L3));
        L3Algae.onFalse(applyState(State.IDLE_ALGAE));

        // CORAL READY STATES
        L1Coral.onTrue(applyState(State.CORAL_L1_READY));

        L2Coral.and(operator.leftScore).onTrue(applyState(State.CORAL_L2_READY.left()));
        L2Coral.and(operator.rightScore).onTrue(applyState(State.CORAL_L2_READY.right()));

        L3Coral.and(operator.leftScore).onTrue(applyState(State.CORAL_L3_READY.left()));
        L3Coral.and(operator.rightScore).onTrue(applyState(State.CORAL_L3_READY.right()));

        L4Coral.and(operator.leftScore).onTrue(applyState(State.CORAL_L4_READY.left()));
        L4Coral.and(operator.rightScore).onTrue(applyState(State.CORAL_L4_READY.right()));

        // CORAL SCORE STATES
        pilot.actionReady_RB.onTrue(applyState(appliedState.getNextScoreState()));
        pilot.actionReady_RB.onFalse(applyState(appliedState.getNextScoreState()));

        // ALGAE READY STATES
        netAlgae.onTrue(applyState(State.IDLE_ALGAE));
        netAlgae.and(pilot.actionReady_RB).onTrue(applyState(State.ALGAE_NET_READY));
        netAlgae.and(pilot.actionReady_RB).onFalse(applyState(State.ALGAE_NET_RELEASE));

        // AUTO SCORE
        // aligned.debounce(scoreAfterAlignTime)
        //         .and(
        //                 autoScoreMode,
        //                 actionPrepState,
        //                 completeStagedCoral,
        //                 pilot.actionReady_RB.not())
        //         .onTrue(
        //                 actionPrepState.setFalse(),
        //                 actionState
        //                         .setTrueForTimeWithCancel(
        //                                 RobotStates::getScoreTime, actionPrepState)
        //
        // .andThen(autoScoreMode.setFalse().onlyIf(actionPrepState.not())));

        // aligned.debounce(autonScoreAfterAlignTime)
        //         .and(autonAutoScoreMode, actionPrepState, completeStagedCoral)
        //         .onTrue(
        //                 actionPrepState.setFalse(),
        //                 actionState
        //                         .setTrueForTime(RobotStates::getAutonScoreTime)
        //                         .andThen(autonAutoScoreMode.setFalse()));
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    private static Command applyState(State state) {
        return new InstantCommand(
                        () -> {
                            appliedState = state;
                            SmartDashboard.putString("Applied State", state.toString());
                            SmartDashboard.putString(
                                    "AppliedState Variable", appliedState.toString());
                            coordinator.applyRobotState(state);
                        })
                .withName(state.toString());
    }

    //     public static Command clearStaged() {
    //         return l1.setFalse()
    //                 .alongWith(
    //                         l2.setFalse(),
    //                         l3.setFalse(),
    //                         l4.setFalse(),
    //                         rightScore.setFalse(),
    //                         coral.setFalse(),
    //                         algae.setFalse(),
    //                         shrinkState.setFalse(),
    //                         autonStationIntake.setFalse())
    //                 .withName("Clear Staged");
    //     }

    //     public static Command clearStates() {
    //         return clearStaged()
    //                 .alongWith(
    //                         reverse.setFalse(),
    //                         actionPrepState.setFalse(),
    //                         actionState.setFalse(),
    //                         homeAll.setFalse(),
    //                         coastMode.setFalse(),
    //                         twistAtReef.setFalse(),
    //                         aligned.setFalse(),
    //                         autoScoreMode.setFalse(),
    //                         autonAutoScoreMode.setFalse(),
    //                         coralScoring.setFalse())
    //                 .withName("Clear States");
    //     }

    //     // clears states without stopping homing sequence
    //     public static Command autonClearStates() {
    //         return clearStaged()
    //                 .alongWith(
    //                         reverse.setFalse(),
    //                         actionPrepState.setFalse(),
    //                         actionState.setFalse(),
    //                         coastMode.setFalse(),
    //                         twistAtReef.setFalse(),
    //                         aligned.setFalse(),
    //                         autoScoreMode.setFalse(),
    //                         autonAutoScoreMode.setFalse(),
    //                         coralScoring.setFalse())
    //                 .withName("Auton Clear States");
    //     }
}
