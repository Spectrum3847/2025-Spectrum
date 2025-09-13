package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;
import lombok.Getter;
import lombok.Setter;

public class Pilot extends Gamepad {

    // Triggers, these would be robot states such as ampReady, intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger enabled = teleop.or(testMode); // works for both teleop and testMode
    private final Trigger photon = new Trigger(() -> Rio.id == Rio.PHOTON_2025);
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();
    public final Trigger home_select = select;

    public final Trigger stationIntake_LT = leftTrigger.and(noFn, teleop);
    public final Trigger groundAlgae_RT = rightTrigger.and(noFn, teleop, photon.not());
    public final Trigger photonRemoveL2Algae = groundAlgae_RT.and(photon);
    public final Trigger groundCoral_LB_LT = leftTrigger.and(fn, teleop, photon.not());

    public final Trigger l2AlgaeRemoval = X.and(teleop);
    public final Trigger l3AlgaeRemoval = Y.and(teleop);
    public final Trigger photonRemoveL3Algae = rightTrigger.and(fn, teleop, photon);

    public final Trigger climbRoutine_start = start.and(noFn, teleop);

    public final Trigger actionReady_RB = rightBumper.and(teleop);

    // Vision Triggers
    public final Trigger tagsInView = new Trigger(() -> Robot.getVision().tagsInView());

    // vision Drive
    // public final Trigger reefAim_A = A.and(teleop, tagsInView.not()); // turn off
    public final Trigger reefVision_A = A.and(teleop); // , tagsInView); // remove tags in view
    public final Trigger reefAlignScore_B = B.and(teleop);
    // public final Trigger cageAim_B = B.and(teleop);

    // Drive Triggers
    public final Trigger upReorient = upDpad.and(fn, teleop);
    public final Trigger leftReorient = leftDpad.and(fn, teleop);
    public final Trigger downReorient = downDpad.and(fn, teleop);
    public final Trigger rightReorient = rightDpad.and(fn, teleop);

    /* Use the right stick to set a cardinal direction to aim at */
    public final Trigger driving = enabled.and(leftStickX.or(leftStickY));
    public final Trigger steer = enabled.and(rightStickX.or(rightStickY));

    public final Trigger fpv_LS = leftStickClick.and(enabled); // Remapped to Left back button
    public final Trigger toggleReverse =
            rightStickClick.and(enabled); // Remapped to Right back button

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B);
    public final Trigger coastOff_dA = disabled.and(A);
    public final Trigger reZero_start = disabled.and(leftBumper, rightBumper, start);
    public final Trigger visionPoseReset_LB_Select = disabled.and(leftBumper, select);

    // TEST TRIGGERS
    public final Trigger testTune_tB = testMode.and(B);
    public final Trigger testTune_tA = testMode.and(A);
    public final Trigger testTune_tX = testMode.and(X);
    public final Trigger testTune_tY = testMode.and(Y);
    public final Trigger testTune_RB = testMode.and(rightBumper);
    public final Trigger testTune_LB = testMode.and(leftBumper);
    public final Trigger testTriggersTrigger = testMode.and(leftTrigger.or(rightTrigger));

    public final Trigger testActionReady = rightBumper.and(testMode);

    public static class PilotConfig extends Config {

        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double defaultTurnScalor = 0.6;
        @Getter @Setter private double turboModeScalor = 1;
        private double deadzone = 0.05;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(deadzone);
            setLeftStickExp(3);
            // Set Scalar in Constructor from Swerve Config

            setRightStickDeadzone(deadzone);
            setRightStickExp(3.0);
            setRightStickScalar(3 * Math.PI);

            setTriggersDeadzone(deadzone);
            setTriggersExp(1);
            setTriggersScalar(1);
        }
    }

    private PilotConfig config;

    private @Getter @Setter SpectrumState slowMode = new SpectrumState("SlowMode");
    @Getter @Setter SpectrumState turboMode = new SpectrumState("TurboMode");

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;

        // Set Left stick Scalar from Swerve Config
        config.setLeftStickScalar(Robot.getConfig().swerve.getSpeedAt12Volts().magnitude());
        leftStickCurve.setScalar(config.getLeftStickScalar());

        Robot.add(this);
        Telemetry.print("Pilot Subsystem Initialized: ");
    }

    @Override
    public void setupStates() {
        // Used for setting rumble and control mode states only
        PilotStates.setStates();
    }

    @Override
    public void setupDefaultCommand() {
        PilotStates.setupDefaultCommand();
    }

    // DRIVE METHODS
    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (slowMode.getAsBoolean()) {
            fwdPositive *= Math.abs(config.getSlowModeScalor());
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (slowMode.getAsBoolean()) {
            leftPositive *= Math.abs(config.getSlowModeScalor());
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (slowMode.getAsBoolean()) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (turboMode.getAsBoolean()) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        } else {
            ccwPositive *= Math.abs(config.getDefaultTurnScalor());
        }
        return -1 * ccwPositive; // invert the value
    }

    public double getPilotStickAngle() {
        return getLeftStickDirection().getRadians();
    }
}
