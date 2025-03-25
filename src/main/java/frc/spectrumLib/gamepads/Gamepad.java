package frc.spectrumLib.gamepads;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.SpectrumSubsystem;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.ExpCurve;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

// Gamepad class
public abstract class Gamepad implements SpectrumSubsystem {
    private Alert disconnectedAlert;

    public static final Trigger kFalse = new Trigger(() -> false);

    private CommandXboxController xboxController;
    protected Trigger A = kFalse;
    protected Trigger B = kFalse;
    protected Trigger X = kFalse;
    protected Trigger Y = kFalse;
    protected Trigger leftBumper = kFalse;
    protected Trigger rightBumper = kFalse;
    protected Trigger leftTrigger = kFalse;
    protected Trigger rightTrigger = kFalse;
    protected Trigger leftStickClick = kFalse;
    protected Trigger rightStickClick = kFalse;
    protected Trigger start = kFalse;
    protected Trigger select = kFalse;
    protected Trigger upDpad = kFalse;
    protected Trigger downDpad = kFalse;
    protected Trigger leftDpad = kFalse;
    protected Trigger rightDpad = kFalse;
    protected Trigger leftStickY = kFalse;
    protected Trigger leftStickX = kFalse;
    protected Trigger rightStickY = kFalse;
    protected Trigger rightStickX = kFalse;

    // Function bumper and trigger buttons
    public Trigger noBumpers;
    public Trigger leftBumperOnly;
    public Trigger rightBumperOnly;
    public Trigger bothBumpers;
    public Trigger noTriggers;
    public Trigger leftTriggerOnly;
    public Trigger rightTriggerOnly;
    public Trigger bothTriggers;
    public Trigger noModifiers;

    private Rotation2d storedLeftStickDirection = new Rotation2d();
    private Rotation2d storedRightStickDirection = new Rotation2d();
    private boolean configured =
            false; // Used to determine if we detected the gamepad is plugged and we have configured
    // it
    private boolean printed = false; // Used to only print Gamepad Not Detected once

    @Getter protected final ExpCurve leftStickCurve;
    @Getter protected final ExpCurve rightStickCurve;
    @Getter protected final ExpCurve triggersCurve;

    protected Trigger teleop = Util.teleop;
    protected Trigger autoMode = Util.autoMode;
    protected Trigger testMode = Util.testMode;
    protected Trigger disabled = Util.disabled;

    public static class Config {
        @Getter private String name;
        @Getter private int port; // USB port on the DriverStation app

        // A configured value to say if we should use this controller on this robot
        @Getter @Setter private boolean attached;

        @Getter @Setter double leftStickDeadzone = 0.001;
        @Getter @Setter double leftStickExp = 1.0;
        @Getter @Setter double leftStickScalar = 1.0;

        @Getter @Setter double rightStickDeadzone = 0.001;
        @Getter @Setter double rightStickExp = 1.0;
        @Getter @Setter double rightStickScalar = 1.0;

        @Getter @Setter double triggersDeadzone = 0.002;
        @Getter @Setter double triggersExp = 1.0;
        @Getter @Setter double triggersScalar = 1.0;

        public Config(String name, int port) {
            this.name = name;
            this.port = port;
        }
    }

    private Config config;

    /**
     * Constructs a Gamepad object with the specified configuration.
     *
     * @param config the configuration object containing settings for the gamepad
     *     <p>The constructor initializes the following: - Superclass with port and attachment
     *     status from the configuration. - Curve objects for left stick, right stick, and triggers
     *     using exponential curves. - If the gamepad is attached, initializes the Xbox controller
     *     and its buttons, triggers, sticks, and D-pad.
     */
    protected Gamepad(Config config) {
        this.config = config;
        disconnectedAlert =
                new Alert(config.name + " Gamepad Disconnected", Alert.AlertType.kError);

        // Curve objects that we use to configure the controller axis objects
        leftStickCurve =
                new ExpCurve(
                        config.getLeftStickExp(),
                        0,
                        config.getLeftStickScalar(),
                        config.getLeftStickDeadzone());
        rightStickCurve =
                new ExpCurve(
                        config.getRightStickExp(),
                        0,
                        config.getRightStickScalar(),
                        config.getRightStickDeadzone());
        triggersCurve =
                new ExpCurve(
                        config.getTriggersExp(),
                        0,
                        config.getTriggersScalar(),
                        config.getTriggersDeadzone());

        if (config.attached) {
            xboxController = new CommandXboxController(config.port);
            A = xboxController.a();
            B = xboxController.b();
            X = xboxController.x();
            Y = xboxController.y();
            leftBumper = xboxController.leftBumper();
            rightBumper = xboxController.rightBumper();
            leftTrigger =
                    xboxController.leftTrigger(
                            config.triggersDeadzone); // Assuming a default threshold of 0.5
            rightTrigger =
                    xboxController.rightTrigger(
                            config.triggersDeadzone); // Assuming a default threshold of 0.5
            leftStickClick = xboxController.leftStick();
            rightStickClick = xboxController.rightStick();
            start = xboxController.start();
            select = xboxController.back();
            upDpad = xboxController.povUp();
            downDpad = xboxController.povDown();
            leftDpad =
                    xboxController
                            .povLeft()
                            .or(xboxController.povUpLeft(), xboxController.povDownLeft());
            rightDpad =
                    xboxController
                            .povRight()
                            .or(xboxController.povDownRight(), xboxController.povUpRight());
            leftStickY = leftYTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            leftStickX = leftXTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            rightStickY = rightYTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);
            rightStickX = rightXTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);
            leftStickY = leftYTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            leftStickX = leftXTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            rightStickY = rightYTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);
            rightStickX = rightXTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);

            // Setup function bumper and trigger buttons
            noBumpers = rightBumper.negate().and(leftBumper.negate());
            leftBumperOnly = leftBumper.and(rightBumper.negate());
            rightBumperOnly = rightBumper.and(leftBumper.negate());
            bothBumpers = rightBumper.and(leftBumper);
            noTriggers = leftTrigger.negate().and(rightTrigger.negate());
            leftTriggerOnly = leftTrigger.and(rightTrigger.negate());
            rightTriggerOnly = rightTrigger.and(leftTrigger.negate());
            bothTriggers = leftTrigger.and(rightTrigger);
            noModifiers = noBumpers.and(noTriggers);
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        configure();
    }

    // Configure the pilot controller
    public void configure() {
        if (config.isAttached()) {
            disconnectedAlert.set(!isConnected()); // Display if the controller is disconnected

            // Detect whether the Xbox controller has been plugged in after start-up
            if (!configured) {
                if (!isConnected()) {
                    if (!printed) {
                        Telemetry.print("##" + getName() + ": GAMEPAD NOT CONNECTED ##");
                        printed = true;
                    }
                    return;
                }

                configured = true;
                Telemetry.print("## " + getName() + ": gamepad is connected ##");
            }
        }
    }

    // Reset the controller configure, should be used with
    // CommandScheduler.getInstance.clearButtons()
    // to reset buttons
    public void resetConfig() {
        configured = false;
        configure();
    }

    /* Zero is stick up, 90 is stick to the left*/
    public Rotation2d getLeftStickDirection() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedLeftStickDirection = angle;
        }
        return storedLeftStickDirection;
    }

    public Rotation2d getRightStickDirection() {
        double x = getRightX();
        double y = getRightY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedRightStickDirection = angle;
        }
        return storedRightStickDirection;
    }

    public double getLeftStickCardinals() {
        double stickAngle = getLeftStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getRightStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getLeftStickMagnitude() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        return Math.sqrt(x * x + y * y);
    }

    public double getRightStickMagnitude() {
        double x = getRightX();
        double y = getRightY();
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Get proper stick angles for each alliance
     *
     * @return
     */
    public double chooseCardinalDirections() {
        // hotfix
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return getRedAllianceStickCardinals();
        }
        return getBlueAllianceStickCardinals();
    }

    public double getBlueAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return 0;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return -Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else {
            return Math.PI; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    /**
     * Flips the stick direction for the red alliance.
     *
     * @return
     */
    public double getRedAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();

        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return Math.PI;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return -Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return Math.PI / 4;
        } else {
            return 0; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    public Trigger leftYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftY);
    }

    public Trigger leftXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftX);
    }

    public Trigger rightYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightY);
    }

    public Trigger rightXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightX);
    }

    public Trigger rightStick(double threshold) {
        return new Trigger(
                () -> Math.abs(getRightX()) >= threshold || Math.abs(getRightY()) >= threshold);
    }

    public Trigger leftStick(double threshold) {
        return new Trigger(
                () -> Math.abs(getLeftX()) >= threshold || Math.abs(getLeftY()) >= threshold);
    }

    private Trigger axisTrigger(Threshold t, double threshold, DoubleSupplier v) {
        return new Trigger(
                () -> {
                    double value = v.getAsDouble();
                    switch (t) {
                        case GREATER:
                            return value > threshold;
                        case LESS:
                            return value < threshold;
                        case ABS_GREATER: // Also called Deadband
                            return Math.abs(value) > threshold;
                        default:
                            return false;
                    }
                });
    }

    public enum Threshold {
        GREATER,
        LESS,
        ABS_GREATER;
    }

    private void rumble(double leftIntensity, double rightIntensity) {
        rumbleController(leftIntensity, rightIntensity);
    }

    /** Command that can be used to rumble the pilot controller */
    public Command rumbleCommand(
            double leftIntensity, double rightIntensity, double durationSeconds) {
        return new RunCommand(() -> rumble(leftIntensity, rightIntensity), this)
                .withTimeout(durationSeconds)
                .ignoringDisable(true)
                .withName("Gamepad.Rumble");
    }

    public Command rumbleCommand(double intensity, double durationSeconds) {
        return rumbleCommand(intensity, intensity, durationSeconds);
    }

    /**
     * Returns a new Command object that combines the given command with a rumble command. The
     * rumble command has a rumble strength of 1 and a duration of 0.5 seconds. The name of the
     * returned command is set to the name of the given command.
     *
     * @param command the command to be combined with the rumble command
     * @return a new Command object with rumble command
     */
    public Command rumbleCommand(Command command) {
        return command.alongWith(rumbleCommand(1, 0.5)).withName(command.getName());
    }

    public boolean isConnected() {
        if (config.attached) {
            return this.getHID().isConnected();
        } else {
            return false;
        }
    }

    protected double getRightTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightTriggerAxis();
    }

    protected double getLeftTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftTriggerAxis();
    }

    protected double getTwist() {
        double right = getRightTriggerAxis();
        double left = getLeftTriggerAxis();
        double value = right - left;
        return value;
    }

    protected double getLeftX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftX();
    }

    protected double getLeftY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftY();
    }

    protected double getRightX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightX();
    }

    protected double getRightY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightY();
    }

    protected GenericHID getHID() {
        if (!config.attached) {
            return null;
        }
        return xboxController.getHID();
    }

    protected GenericHID getRumbleHID() {
        if (!isConnected()) {
            return null;
        }
        return xboxController.getHID();
    }

    public void rumbleController(double leftIntensity, double rightIntensity) {
        if (!isConnected()) {
            return;
        }
        getRumbleHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
        getRumbleHID().setRumble(RumbleType.kRightRumble, rightIntensity);
    }
}
