package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;

public class Operator extends Gamepad {
    // Triggers, these would be robot states such as intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();

    public final Trigger leftScore_Dpad = leftDpad.and(noFn, teleop);
    public final Trigger rightScore_Dpad = rightDpad.and(noFn, teleop);

    public final Trigger climbPrep_start = start.and(noFn, teleop);

    public final SpectrumState coralStageSelector = new SpectrumState("coral stage");
    public final SpectrumState algaeStageSelector = new SpectrumState("algae stage");

    public final Trigger operatorCoralStage = leftBumper.and(teleop).or(coralStageSelector);
    public final Trigger operatorAlgaeStage = rightBumper.and(teleop).or(algaeStageSelector);
    public final Trigger homeState = operatorCoralStage.not().and(operatorAlgaeStage.not());

    public final Trigger L1Coral_A = A.and(operatorCoralStage);
    public final Trigger L2Coral_B = B.and(operatorCoralStage);
    public final Trigger L3Coral_X = X.and(operatorCoralStage);
    public final Trigger L4Coral_Y = Y.and(operatorCoralStage);

    public final Trigger L2Algae_B = B.and(operatorAlgaeStage);
    public final Trigger L3Algae_X = X.and(operatorAlgaeStage);
    public final Trigger barge_Y = Y.and(operatorAlgaeStage);

    public final Trigger homeElevator_A = A.and(noFn, teleop);
    public final Trigger homeInClimb_B = B.and(noFn, teleop);

    public final Trigger algaeHandoff_X = X.and(noFn, teleop);
    public final Trigger coralHandoff_Y = Y.and(noFn, teleop);

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B);
    public final Trigger coastOff_dA = disabled.and(A);

    // TEST TRIGGERS

    public static class OperatorConfig extends Config {

        public OperatorConfig() {
            super("Operator", 1);
            setTriggersDeadzone(0.0);
        }
    }

    private OperatorConfig config;

    public Operator(OperatorConfig config) {
        super(config);
        this.config = config;
        Robot.add(this);
        Telemetry.print("Operator Subsystem Initialized: ");
    }

    public void setupStates() {
        // Left Blank so we can bind when the controller is connected
        OperatorStates.setStates();
    }

    public void setupDefaultCommand() {
        OperatorStates.setupDefaultCommand();
    }

    public double getElevatorOverride() {
        return getLeftY();
    }

    public double getClimberOverride() {
        return getRightY();
    }
}
