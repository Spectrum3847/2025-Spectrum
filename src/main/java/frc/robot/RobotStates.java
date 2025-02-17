package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.swerve.Swerve;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumState;

public class RobotStates {
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Swerve swerve = Robot.getSwerve();

    /**
     * Define Robot States here and how they can be triggered States should be triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving the state
     * RobotType Triggers
     */
    public static final Trigger am = new Trigger(() -> Rio.id == Rio.AM_2025);

    public static final Trigger fm = new Trigger(() -> Rio.id == Rio.FM_2024);
    public static final Trigger sim = new Trigger(RobotBase::isSimulation);

    // zones
    public static final Trigger topLeftZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger topRightZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));
    public static final Trigger bottomLeftZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger bottomRightZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));

    public static final Trigger bargeZone =
            swerve.inXzoneAlliance(
                            3 * Field.getHalfLength() / 4,
                            Field.getHalfLength()
                                    - Units.inchesToMeters(24)
                                    - swerve.getConfig().getRobotLength() / 2)
                    .and(topLeftZone);

    // intake Triggers
    public static final Trigger visionIntaking = Trigger.kFalse;
    public static final Trigger stationIntaking =
            pilot.stationIntake_LT
                    .or(visionIntaking, autonSourceIntake)
                    .and(bottomLeftZone.or(bottomRightZone));
    public static final Trigger stationExtendedIntake = pilot.stationExtendedIntake_LB_LT;
    public static final Trigger groundAlgae = pilot.groundAlgae_RT;
    public static final Trigger groundCoral = pilot.groundCoral_LB_RT;

    // score Triggers
    public static final Trigger actionPrepState = pilot.actionReady;

    // climb Triggers
    public static final Trigger climbPrep = operator.climbPrep_start;
    public static final Trigger climbFinish = pilot.climbRoutine_start;

    // mechanism preset Triggers (Wrist, Elevator, etc.)
    public static final Trigger processorLollipopScore = pilot.lollipopProcessor_A;
    public static final Trigger L2Algae =
            operator.L2Algae_B.or(autonLowAlgae); // TODO: Likely going to Pilot Intake commands
    public static final Trigger L3Algae = operator.L3Algae_X.or(autonHighAlgae);
    public static final Trigger algaeRetract =
            pilot.algaeRetract_B; // TODO: make this sole algae command once vision done

    public static final Trigger barge = operator.barge_Y.and(bargeZone);

    public static final Trigger L1Coral = operator.L1Coral_A;
    public static final Trigger L2Coral = operator.L2Coral_B;
    public static final Trigger L3Coral = operator.L3Coral_X;
    public static final Trigger L4Coral = operator.L4Coral_Y;

    public static final Trigger algaeHandoff = operator.algaeHandoff_X;
    public static final Trigger coralHandoff = operator.coralHandoff_Y;

    public static final Trigger homeAll = operator.homeState;

    // reset triggers
    public static final Trigger homeElevator = operator.homeElevator_A;
    public static final Trigger homeInClimb = operator.homeInClimb_B;

    // Robot States
    // These are states that aren't directly tied to hardware or buttons, etc.
    // If they should be set by multiple Triggers do that in SetupStates()
    public static final SpectrumState coastMode = new SpectrumState("coast");
    public static final SpectrumState leftScore = new SpectrumState("leftScore");
    public static final SpectrumState rightScore = new SpectrumState("rightScore");
    public static final SpectrumState scoreState = new SpectrumState("scoreState");

    public static final Trigger coastOn = pilot.coastOn_dB;

    public static final Trigger reefPosition =
            L2Algae.or(L3Algae, L1Coral, L2Coral, L3Coral, L4Coral);

    public static final Trigger coralReefPosition = L1Coral.or(L2Coral, L3Coral, L4Coral);

    public static final SpectrumState backwardMode = new SpectrumState("backward");

    // Setup any binding to set states
    public static void setupStates() {
        pilot.coastOn_dB.onTrue(coastMode.setTrue().ignoringDisable(true));
        pilot.coastOff_dA.onTrue(coastMode.setFalse().ignoringDisable(true));
        actionPrepState.onFalse(scoreState.setTrue());
        actionPrepState.onTrue(scoreState.setFalse());

        bottomLeftZone
                .and(stationIntaking)
                .onTrue(
                        backwardMode
                                .setTrue()
                                .onlyIf(
                                        () ->
                                                !swerve.frontClosestToAngle(
                                                        Field.CoralStation.leftCenterFace
                                                                .getRotation()
                                                                .getDegrees())));
        bottomLeftZone
                .and(stationIntaking)
                .onTrue(
                        backwardMode
                                .setFalse()
                                .onlyIf(
                                        () ->
                                                swerve.frontClosestToAngle(
                                                        Field.CoralStation.leftCenterFace
                                                                .getRotation()
                                                                .getDegrees())));

        bottomRightZone
                .and(stationIntaking)
                .onTrue(
                        backwardMode
                                .setTrue()
                                .onlyIf(
                                        () ->
                                                !swerve.frontClosestToAngle(
                                                        Field.CoralStation.rightCenterFace
                                                                .getRotation()
                                                                .getDegrees())));
        bottomRightZone
                .and(stationIntaking)
                .onTrue(
                        backwardMode
                                .setFalse()
                                .onlyIf(
                                        () ->
                                                swerve.frontClosestToAngle(
                                                        Field.CoralStation.rightCenterFace
                                                                .getRotation()
                                                                .getDegrees())));

        bargeZone
                .and(barge)
                .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(180)));
        bargeZone
                .and(barge)
                .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(180)));

        topLeftZone
                .and(reefPosition)
                .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(60)));
        topLeftZone
                .and(reefPosition)
                .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(60)));

        topRightZone
                .and(reefPosition)
                .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(-60)));
        topRightZone
                .and(reefPosition)
                .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(-60)));

        bottomLeftZone
                .and(reefPosition)
                .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(120)));
        bottomLeftZone
                .and(reefPosition)
                .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(120)));

        bottomRightZone
                .and(reefPosition)
                .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(-120)));
        bottomRightZone
                .and(reefPosition)
                .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(-120)));

        operator.leftScore_Dpad.onTrue(leftScore.setTrue(), rightScore.setFalse());
        operator.rightScore_Dpad.onTrue(rightScore.setTrue(), leftScore.setFalse());
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
