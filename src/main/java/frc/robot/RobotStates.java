package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.FieldHelpers;
import frc.reefscape.Zones;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.ElevatorStates;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.shoulder.ShoulderStates;
import frc.robot.swerve.SwerveStates;
import frc.robot.twist.TwistStates;
import frc.robot.vision.VisionStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.util.Util;
import lombok.Getter;

public class RobotStates {
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();

    @Getter private static double scoreTime = 2.0;
    @Getter private static double autonScoreTime = 0.75;
    @Getter private static double twistAtReefDelay = 0.2;
    @Getter private static double scoreAfterAlignTime = 0.03;
    @Getter private static double autonScoreAfterAlignTime = 0.05;
    @Getter private static double actionPrepToActionTime = 0.05;

    // Robot States
    // These are states that aren't directly tied to hardware or buttons, etc.
    // If they should be set by multiple Triggers do that in SetupStates()
    public static final SpectrumState coastMode = new SpectrumState("coast");
    public static final SpectrumState coral = new SpectrumState("coral");
    public static final SpectrumState algae = new SpectrumState("algae");
    public static final SpectrumState l1 = new SpectrumState("l1");
    public static final SpectrumState l2 = new SpectrumState("l2");
    public static final SpectrumState l3 = new SpectrumState("l3");
    public static final SpectrumState l4 = new SpectrumState("l4");
    public static final SpectrumState shrinkState = new SpectrumState("extendedStates");
    public static final SpectrumState rightScore = new SpectrumState("rightScore");
    public static final SpectrumState reverse = new SpectrumState("reverse");
    public static final SpectrumState actionPrepState = new SpectrumState("actionPrepState");
    public static final SpectrumState actionState = new SpectrumState("actionState");
    public static final SpectrumState homeAll = new SpectrumState("homeAll");
    public static final SpectrumState autonStationIntake = new SpectrumState("autonStationIntake");
    public static final SpectrumState twistAtReef = new SpectrumState("twistCoralReef");
    public static final SpectrumState aligned = new SpectrumState("aligned");
    public static final SpectrumState autoScoreMode = new SpectrumState("autoScoreMode");
    public static final SpectrumState autonAutoScoreMode = new SpectrumState("autonAutoScoreMode");
    public static final SpectrumState coralScoring = new SpectrumState("coralScoring");

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
    public static final Trigger intakeRunning = coral.or(algae);
    public static final Trigger stationIntaking = pilot.stationIntake_LT.or(autonStationIntake);
    // public static final Trigger stationExtendedIntaking = pilot.stationIntakeExtended_LT_RB;
    public static final Trigger groundAlgae = pilot.groundAlgae_RT;
    public static final Trigger groundCoral = pilot.groundCoral_LB_LT;
    public static final Trigger intaking = stationIntaking.or(groundAlgae, groundCoral);

    // climb Triggers
    public static final Trigger climbPrep = operator.climbPrep_start;
    public static final Trigger climbFinish = pilot.climbRoutine_start;

    // mechanism preset Triggers (Wrist, Elevator, etc.)
    public static final Trigger shrink = pilot.fn.or(shrinkState);
    public static final Trigger processorAlgae = l1.and(algae);
    public static final Trigger L2Algae = (l2.and(algae)).or(autonLowAlgae);
    public static final Trigger L3Algae = (l3.and(algae)).or(autonHighAlgae);
    public static final Trigger netAlgae = (l4.and(algae)).or(autonNet);
    public static final Trigger stagedAlgae = processorAlgae.or(L2Algae, L3Algae, netAlgae);

    public static final Trigger L1Coral = (l1.and(coral)).or(autonL1);
    public static final Trigger L2Coral = l2.and(coral);
    public static final Trigger L3Coral = l3.and(coral);
    public static final Trigger L4Coral = (l4.and(coral));
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
            (ElbowStates.isL4Coral.and(ShoulderStates.isL4Coral, ElevatorStates.isL4Coral))
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

    // auton Triggers
    public static final Trigger poseUpdate = autonPoseUpdate.or(autonAutoScoreMode);

    public static final Trigger isAtHome =
            ElevatorStates.isHome.and(ElbowStates.isHome, ShoulderStates.isHome);

    public static final Trigger twistStageComplete =
            branch.and(
                    TwistStates.isLeft
                            .and(rightScore.not())
                            .or(TwistStates.isRight.and(rightScore)));

    // reset triggers
    public static final Trigger homeElevator = operator.homeElevator_A;

    public static final Trigger hasGamePiece = new Trigger(Robot.getIntake()::hasIntakeGamePiece);

    // Setup any binding to set states
    public static void setupStates() {
        Util.disabled.onTrue(clearStates().repeatedly().withTimeout(3));

        // *********************************

        // HOME Commands and States
        pilot.home_select.or(operator.home_select).whileTrue(homeAll.toggleToTrue());
        pilot.home_select.or(operator.home_select).onFalse(clearStates());
        autonClearStates.onTrue(autonClearStates());

        actionState
                .or(operator.staged)
                .onChangeToFalse(coral.setFalse().alongWith(algae.setFalse()));
        isAtHome.onTrue(homeAll.setFalse());

        pilot.coastOn_dB.or(operator.coastOn_dB).onTrue(coastMode.setTrue().ignoringDisable(true));
        pilot.coastOff_dA
                .or(operator.coastOff_dA)
                .onTrue(coastMode.setFalse().ignoringDisable(true));

        // *********************************
        // ActionPrep and ActionState
        pilot.actionReady_RB.onFalse(actionPrepState.setFalse());
        autonActionOff.onTrue(actionPrepState.setFalse());

        (pilot.actionReady_RB.and(coral.or(algae)))
                .or(autonActionOn)
                .onTrue(actionPrepState.setTrue(), actionState.setFalse());

        actionPrepState.or(autonActionOn).onTrue(actionState.setFalse());
        actionPrepState.onChangeToFalse(
                actionState
                        .setTrueForTimeWithCancel(RobotStates::getScoreTime, actionPrepState)
                        .onlyIf(autoScoreMode.not()));

        autonActionOff.onChangeToFalse(actionState.setTrueForTime(RobotStates::getScoreTime));

        operator.algaeStage.or(operator.coralStage).onTrue(actionState.setFalse());

        (L2Algae.or(L3Algae, processorAlgae)).and(actionState).onTrue(actionState.setFalse());

        // *********************************
        // Intaking States
        stationIntaking.whileTrue(coral.toggleToTrue(), algae.setFalse());
        stationIntaking.onFalse(homeAll.toggleToTrue());

        groundCoral.whileTrue(coral.toggleToTrue(), algae.setFalse());
        groundCoral.onChangeToFalse(homeAll.toggleToTrue());

        groundAlgae.whileTrue(algae.toggleToTrue(), coral.setFalse());
        groundAlgae.onChangeToFalse(homeAll.toggleToTrue());

        pilot.l2AlgaeRemoval.onTrue(
                algae.setTrue(), coral.setFalse(), l2.setTrue(), actionPrepState.setTrue());
        pilot.l2AlgaeRemoval.onFalse(l2.setFalse(), actionPrepState.setFalse());

        pilot.l3AlgaeRemoval.onTrue(
                algae.setTrue(), coral.setFalse(), l3.setTrue(), actionPrepState.setTrue());
        pilot.l3AlgaeRemoval.onFalse(l3.setFalse(), actionPrepState.setFalse());

        // **********************************
        // Staging and Scoring

        // Clear staged if we aren't scoring, holding, or staged
        coral.not()
                .and(algae.not(), actionState.not(), actionPrepState.not())
                .onTrue(clearStaged());

        // When we change to not scoring and not coral stage we turn off coral
        actionState.not().and(operator.coralStage.not()).onTrue(coral.setFalse());

        // When we change to not scoring and not algae stage we turn off algae
        // actionState.not().and(operator.algaeStage.not()).onTrue(algae.setFalse());

        // Set coral if we are staging coral
        operator.coralStage.or(autonCoral).onTrue(coral.setTrue(), algae.setFalse());

        // Set algae if we are staging algae
        operator.algaeStage.or(autonAlgae).onTrue(algae.setTrue(), coral.setFalse());

        // Set Levels
        (operator.L1.and(operator.staged))
                .or(autonL1)
                .onTrue(l1.setTrue(), l2.setFalse(), l3.setFalse(), l4.setFalse());
        (operator.L2.and(operator.staged))
                .or(autonL2)
                .onTrue(l2.setTrue(), l1.setFalse(), l3.setFalse(), l4.setFalse());
        (operator.L3.and(operator.staged))
                .or(autonL3)
                .onTrue(l3.setTrue(), l1.setFalse(), l2.setFalse(), l4.setFalse());
        (operator.L4.and(operator.staged))
                .or(autonL4)
                .onTrue(l4.setTrue(), l1.setFalse(), l2.setFalse(), l3.setFalse());

        // Set left or right score
        operator.leftScore.and(operator.staged).onTrue(rightScore.setFalse());
        operator.rightScore.and(operator.staged).onTrue(rightScore.setTrue());

        // Set twist at reef if the arm is staged and at left or right
        actionPrepState
                .and(twistStageComplete.debounce(getTwistAtReefDelay()))
                .onTrue(twistAtReef.setTrue());
        actionState.onTrue(twistAtReef.setFalse());
        reverse.onChange(twistAtReef.setFalse());

        // Set coralScoring when we try to score, turn off when homed
        actionPrepState.and(L1Coral, atL1Coral).onTrue(coralScoring.setTrue());
        actionPrepState.and(L2Coral, atL2Coral).onTrue(coralScoring.setTrue());
        actionPrepState.and(L3Coral, atL3Coral).onTrue(coralScoring.setTrue());
        actionPrepState.and(L4Coral, atL4Coral).onTrue(coralScoring.setTrue());
        algae.onTrue(coralScoring.setFalse());
        homeAll.onTrue(coralScoring.setFalse());

        // *********************************
        // Auton States
        autonSourceIntakeOn.onTrue(autonStationIntake.setTrue());
        autonSourceIntakeOff.onTrue(autonStationIntake.setFalse());
        autonLeft.onTrue(rightScore.setFalse());
        autonRight.onTrue(rightScore.setTrue());
        autonHome.onTrue(homeAll.toggleToTrue());
        autonReverse.whileTrue(reverse.setTrue());
        autonAutoScore.onTrue(autonAutoScoreMode.setTrue());

        // *********************************
        // Reversal States
        toggleReverse.onTrue(reverse.toggle());

        poseReversal.and(stagedCoral.or(L2Algae, L3Algae)).onTrue(reverse.setTrue());
        poseReversal.not().and(stagedCoral.or(L2Algae, L3Algae)).onTrue(reverse.setFalse());
        stagedCoral
                .or(L2Algae, L3Algae)
                .and(
                        VisionStates.usingRearTag,
                        actionPrepState.not(),
                        actionState.not(),
                        poseReversal.not())
                .onTrue(reverse.setTrue());
        stagedCoral
                .or(L2Algae, L3Algae)
                .and(
                        VisionStates.usingRearTag.not(),
                        actionPrepState.not(),
                        actionState.not(),
                        poseReversal.not())
                .onTrue(reverse.setFalse());
        groundAlgae.or(groundCoral, processorAlgae).and(toggleReverse).onTrue(reverse.setTrue());
        groundAlgae
                .or(groundCoral, processorAlgae)
                .and(toggleReverse.not())
                .onTrue(reverse.setFalse());

        stationIntaking
                .and(Zones.bottomLeftZone, SwerveStates.isFrontClosestToLeftStation.not())
                .onTrue(reverse.setTrue());
        stationIntaking
                .and(Zones.bottomLeftZone, SwerveStates.isFrontClosestToLeftStation)
                .onTrue(reverse.setFalse());
        stationIntaking
                .and(Zones.bottomRightZone, SwerveStates.isFrontClosestToRightStation.not())
                .onTrue(reverse.setTrue());
        stationIntaking
                .and(Zones.bottomRightZone, SwerveStates.isFrontClosestToRightStation)
                .onTrue(reverse.setFalse());

        // netAlgae.and(SwerveStates.isFrontClosestToNet.not()).onTrue(reverse.setTrue());
        // netAlgae.and(SwerveStates.isFrontClosestToNet).onTrue(reverse.setFalse());
        netAlgae.onTrue(reverse.setFalse());

        climbPrep.onTrue(reverse.setFalse());

        // *********************************
        // Align States
        SwerveStates.isAlignedToReef
                .and(pilot.reefAlignScore_B.or(pilot.reefVision_A, Util.autoMode))
                .onTrue(aligned.setTrue());
        SwerveStates.isAlignedToReef
                .and(pilot.reefAlignScore_B.or(pilot.reefVision_A, Util.autoMode))
                .onFalse(aligned.setFalse());

        // *********************************
        // Autoscore States
        pilot.reefAlignScore_B.and(stagedCoral).onTrue(autoScoreMode.setTrue());
        pilot.reefAlignScore_B
                .not()
                .and(autoScoreMode, Util.autoMode.not())
                .debounce(actionPrepToActionTime)
                .onTrue(Commands.waitUntil(actionState.not()), autoScoreMode.setFalse());

        pilot.actionReady_RB.onTrue(autoScoreMode.setFalse());

        // prep before autoscoring
        Zones.isCloseToReef
                .and(pilot.reefAlignScore_B, stagedCoral)
                .onTrue(actionPrepState.setTrue());
        Zones.isCloseToReef
                .and(pilot.reefAlignScore_B, stagedCoral)
                .onFalse(actionPrepState.setFalse().onlyIf(pilot.actionReady_RB.not()));

        // scoring action
        aligned.debounce(scoreAfterAlignTime)
                .and(
                        autoScoreMode,
                        actionPrepState,
                        completeStagedCoral,
                        pilot.actionReady_RB.not())
                .onTrue(
                        actionPrepState.setFalse(),
                        actionState
                                .setTrueForTimeWithCancel(
                                        RobotStates::getScoreTime, actionPrepState)
                                .andThen(autoScoreMode.setFalse().onlyIf(actionPrepState.not())));

        aligned.debounce(autonScoreAfterAlignTime)
                .and(autonAutoScoreMode, actionPrepState, completeStagedCoral)
                .onTrue(
                        actionPrepState.setFalse(),
                        actionState
                                .setTrueForTime(RobotStates::getAutonScoreTime)
                                .andThen(autonAutoScoreMode.setFalse()));
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    public static Command clearStaged() {
        return l1.setFalse()
                .alongWith(
                        l2.setFalse(),
                        l3.setFalse(),
                        l4.setFalse(),
                        rightScore.setFalse(),
                        coral.setFalse(),
                        algae.setFalse(),
                        shrinkState.setFalse(),
                        autonStationIntake.setFalse())
                .withName("Clear Staged");
    }

    public static Command clearStates() {
        return clearStaged()
                .alongWith(
                        reverse.setFalse(),
                        actionPrepState.setFalse(),
                        actionState.setFalse(),
                        homeAll.setFalse(),
                        coastMode.setFalse(),
                        twistAtReef.setFalse(),
                        aligned.setFalse(),
                        autoScoreMode.setFalse(),
                        autonAutoScoreMode.setFalse(),
                        coralScoring.setFalse())
                .withName("Clear States");
    }

    // clears states without stopping homing sequence
    public static Command autonClearStates() {
        return clearStaged()
                .alongWith(
                        reverse.setFalse(),
                        actionPrepState.setFalse(),
                        actionState.setFalse(),
                        coastMode.setFalse(),
                        twistAtReef.setFalse(),
                        aligned.setFalse(),
                        autoScoreMode.setFalse(),
                        autonAutoScoreMode.setFalse(),
                        coralScoring.setFalse())
                .withName("Auton Clear States");
    }
}
