package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.CanConfigBudget;
import frc.spectrumLib.hardware.TalonFXFactory;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.CanDeviceId;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import lombok.*;

/**
 * Abstract base class representing a CTRE TalonFX-driven robot mechanism with common control modes,
 * telemetry, and convenience {@link edu.wpi.first.wpilibj2.command.Command} factories.
 *
 * <p>This class centralizes:
 *
 * <ul>
 *   <li>Motor creation/configuration (leader + optional followers) via the provided {@link Config}
 *   <li>Cached sensor readings (position, velocity, voltage, current) for efficient access
 *   <li>Common unit conversions (rotations/percent/degrees; RPS/RPM)
 *   <li>Standard closed-loop and open-loop control helpers (Motion Magic, velocity, voltage,
 *       percent, torque current)
 *   <li>Convenience {@link edu.wpi.first.wpilibj2.command.button.Trigger} factories (at/above/below
 *       thresholds)
 *   <li>Basic periodic current reporting to a battery/current logger
 * </ul>
 *
 * <h2>Attachment semantics</h2>
 *
 * If {@link Config#isAttached()} is {@code false}, the mechanism will not attempt to construct or
 * command hardware and will return safe default sensor values (typically {@code 0}).
 *
 * <h2>Target tracking</h2>
 *
 * The {@code target} field tracks the last commanded closed-loop setpoint sent by this class. It is
 * used by helper triggers such as {@code atTargetPosition(...)}.
 *
 * <h2>Extending</h2>
 *
 * Concrete mechanisms should provide a {@link Config} describing motor IDs, Talon configuration,
 * follower configuration, and mechanism-specific min/max rotation bounds as needed.
 *
 * <p><b>Note:</b> This class assumes CTRE Phoenix 6 units (rotations, rotations/sec, etc.) and uses
 * {@code Config.talonConfig.Feedback.SensorToMechanismRatio} as the mechanism gearing ratio.
 */
public abstract class Mechanism implements Subsystem {

    // ── Fields ─────────────────────────────────────────────────────────────────

    /** The primary (leader) TalonFX motor controller. */
    @Getter protected TalonFX motor;

    /** Optional follower TalonFX motor controllers that mirror the leader. */
    @Getter protected TalonFX[] followerMotors;

    /** Configuration object holding motor IDs, Talon settings, and mechanism parameters. */
    public Config config;

    /** Alert displayed when an unexpected current reading is detected during diagnostics. */
    Alert currentAlert = new Alert("", AlertType.kWarning);

    /** One alert per follower, raised when it stops pulling its share of the load. */
    private Alert[] followerAlerts = new Alert[0];

    /**
     * Seconds each follower has been silent while its leader was loaded. Any sign of life zeroes
     * it.
     */
    private double[] followerMismatchSeconds = new double[0];

    /** FPGA time of the previous follower check, for integrating the mismatch. */
    private double followerCheckLastSeconds = 0;

    /** The last closed-loop position setpoint (in rotations) sent to the motor. */
    private double target = 0;

    /** The last closed-loop velocity setpoint (in rotations per second) sent to the motor. */
    private double velocityTarget = 0;

    // Status signals read by the getters, refreshed together once per loop (see signalValue)
    private BaseStatusSignal positionSignal;

    /** Typed views of the position and velocity signals, for latency compensation. */
    private StatusSignal<Angle> positionStatusSignal;

    private StatusSignal<AngularVelocity> velocityStatusSignal;
    private BaseStatusSignal velocitySignal;
    private BaseStatusSignal voltageSignal;
    private BaseStatusSignal statorCurrentSignal;
    private BaseStatusSignal supplyCurrentSignal;
    private BaseStatusSignal tempSignal;
    private BaseStatusSignal[] followerSupplySignals = new BaseStatusSignal[0];

    /** Every signal above, so one refreshAll covers the loop. Empty when unattached. */
    private BaseStatusSignal[] loopSignals = new BaseStatusSignal[0];

    /** Loop on which {@link #loopSignals} were last refreshed, or -1 for never. */
    private long lastSignalRefreshLoop = -1;

    /** Per-follower supply current log keys, built once. */
    private String[] followerCurrentKeys = new String[0];

    // Diagnostic log keys, built on the first logDiagnostics() call for the prefix it was given
    private String diagnosticsPrefix;
    private String voltageKey;
    private String statorCurrentKey;
    private String supplyCurrentKey;
    private String tempKey;
    private String motorConnectedKey;

    /**
     * BatteryLogger channel name for this mechanism, built once to avoid per-loop concatenation.
     */
    private final String batteryKey;

    // ── Status signal rates ────────────────────────────────────────────────────
    //
    // Every signal published here costs CANivore bandwidth on every mechanism motor AND every
    // follower. Publishing all eight at 250 Hz put bus utilization at 66-81% in the 2026-09-04
    // logs, which is where the stale-frame warnings and the intermittently unresponsive hood came
    // from. Only the feedback used for control needs to be fast; everything else is read once per
    // 20 ms robot loop for logging, so a faster frame is bandwidth spent on samples nobody reads.

    /**
     * Rate for a leader's position and velocity. Nothing on the rio consumes them faster than the
     * 50 Hz loop (Motion Magic closes the loop on the motor itself), so 100 Hz leaves a 2x margin
     * for latency compensation at 2.5x less bus and Phoenix CPU than the 250 Hz this ran at through
     * 2026-09-05, when CANivore utilization sat at 63-77% and the rio CPU at 92-95%.
     */
    private static final double CONTROL_SIGNAL_HZ = 100;

    /**
     * Rate for the output signals (duty cycle, motor voltage, torque current) of a leader that has
     * followers. A follower mirrors its leader from the leader's status frames, so CTRE requires
     * these to stay enabled on such a leader; 50 Hz is the rate the followers have run on all
     * season, kept as is.
     */
    private static final double FOLLOWED_LEADER_OUTPUT_HZ = 50;

    /**
     * Rate for signals nothing controls on: currents and voltage, the output signals of a leader
     * with no followers, and everything on a follower. They are read once per loop and logged at 10
     * Hz, so 20 Hz is already double what is kept.
     */
    private static final double DIAGNOSTIC_SIGNAL_HZ = 20;

    /** Rate for device temperature, which changes over seconds. */
    private static final double TEMPERATURE_SIGNAL_HZ = 4;

    /** What a motor has to publish, by its job in the mechanism. */
    private enum SignalRole {
        /** Runs the mechanism alone. */
        LEADER,
        /** Runs the mechanism with followers mirroring its output frames. */
        FOLLOWED_LEADER,
        /** Mirrors a leader; nothing on the rio reads its feedback. */
        FOLLOWER
    }

    // ── Constructors ───────────────────────────────────────────────────────────

    /**
     * Creates a Mechanism and, if {@link Config#isAttached()} is {@code true}, initializes the
     * leader TalonFX and any configured follower motors. Sensor caches are always initialized so
     * safe defaults ({@code 0}) are returned even when unattached.
     *
     * @param config the mechanism configuration (motor IDs, Talon settings, follower config, etc.)
     */
    protected Mechanism(Config config) {
        this.config = config;
        batteryKey = "Mechanisms/" + config.getName();

        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
            boolean hasFollowers = config.followerConfigs.length > 0;
            configureStatusSignals(
                    motor,
                    hasFollowers ? SignalRole.FOLLOWED_LEADER : SignalRole.LEADER,
                    config.fastOutputLogging);

            followerMotors = new TalonFX[config.followerConfigs.length];
            followerSupplySignals = new BaseStatusSignal[config.followerConfigs.length];
            followerCurrentKeys = new String[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                followerMotors[i] =
                        TalonFXFactory.createPermanentFollowerTalon(
                                config.followerConfigs[i].id,
                                motor,
                                config.followerConfigs[i].opposeLeader);
                configureStatusSignals(followerMotors[i], SignalRole.FOLLOWER, false);
                followerSupplySignals[i] = followerMotors[i].getSupplyCurrent(false);
                followerCurrentKeys[i] =
                        "Followers/" + config.followerConfigs[i].getName() + "/SupplyCurrent";
            }

            // getX(false) returns the device's signal object without refreshing it; the getters
            // refresh all of them in one Phoenix call per loop.
            positionStatusSignal = motor.getPosition(false);
            velocityStatusSignal = motor.getVelocity(false);
            positionSignal = positionStatusSignal;
            velocitySignal = velocityStatusSignal;
            voltageSignal = motor.getMotorVoltage(false);
            statorCurrentSignal = motor.getStatorCurrent(false);
            supplyCurrentSignal = motor.getSupplyCurrent(false);
            tempSignal = motor.getDeviceTemp(false);
            loopSignals = new BaseStatusSignal[6 + followerSupplySignals.length];
            loopSignals[0] = positionSignal;
            loopSignals[1] = velocitySignal;
            loopSignals[2] = voltageSignal;
            loopSignals[3] = statorCurrentSignal;
            loopSignals[4] = supplyCurrentSignal;
            loopSignals[5] = tempSignal;
            System.arraycopy(
                    followerSupplySignals, 0, loopSignals, 6, followerSupplySignals.length);

            followerAlerts = new Alert[config.followerConfigs.length];
            followerMismatchSeconds = new double[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                FollowerConfig f = config.followerConfigs[i];
                followerAlerts[i] =
                        new Alert(
                                f.getName()
                                        + " (CAN "
                                        + f.getId().getDeviceNumber()
                                        + ") is not drawing current while "
                                        + config.getName()
                                        + " is loaded - check its power leads",
                                AlertType.kError);
            }
        }

        this.register();
    }

    /**
     * Creates a Mechanism and explicitly overrides the {@code attached} flag in the config.
     *
     * @param config the mechanism configuration
     * @param attached {@code true} to enable hardware; {@code false} to run in software-only mode
     */
    protected Mechanism(Config config, boolean attached) {
        // The override must be applied BEFORE the delegated constructor runs, since that
        // constructor decides whether to create the motor hardware based on the flag.
        this(applyAttachedOverride(config, attached));
    }
    /** Applies the attached override. */
    private static Config applyAttachedOverride(Config config, boolean attached) {
        config.attached = attached;
        return config;
    }

    /**
     * Sets the status frame rates this mechanism relies on and disables everything else. A
     * follower's frames cost the same bandwidth as the leader's, so it gets the diagnostic rate for
     * everything; a leader that has followers keeps the output frames they mirror; a leader whose
     * feedforward is fit from logs ({@link Config#isFastOutputLogging()}) publishes its output at
     * the control rate so the logged voltage lines up with the logged velocity.
     *
     * @param talon the motor to configure
     * @param role the motor's job, which decides which frames need to be fast
     * @param fastOutput {@code true} to publish the output frames at the control rate
     */
    private static void configureStatusSignals(TalonFX talon, SignalRole role, boolean fastOutput) {
        double controlHz = role == SignalRole.FOLLOWER ? DIAGNOSTIC_SIGNAL_HZ : CONTROL_SIGNAL_HZ;
        double outputHz =
                fastOutput
                        ? CONTROL_SIGNAL_HZ
                        : role == SignalRole.FOLLOWED_LEADER
                                ? FOLLOWED_LEADER_OUTPUT_HZ
                                : DIAGNOSTIC_SIGNAL_HZ;
        BaseStatusSignal.setUpdateFrequencyForAll(
                controlHz, talon.getPosition(), talon.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(
                outputHz, talon.getDutyCycle(), talon.getMotorVoltage(), talon.getTorqueCurrent());
        BaseStatusSignal.setUpdateFrequencyForAll(
                DIAGNOSTIC_SIGNAL_HZ, talon.getStatorCurrent(), talon.getSupplyCurrent());
        talon.getDeviceTemp().setUpdateFrequency(TEMPERATURE_SIGNAL_HZ);
        // A long run of per-signal config calls, and only ever an optimisation. On a dead
        // bus it is pure boot latency, so it is the first thing dropped once the budget is
        // spent.
        if (!CanConfigBudget.exhausted()) {
            talon.optimizeBusUtilization();
        }
    }

    // ── Subsystem Overrides ────────────────────────────────────────────────────

    /**
     * Called once per scheduler loop. Concrete subclasses should override to implement their
     * periodic state-machine logic, telemetry, and sensor updates.
     */
    @Override
    public void periodic() {}

    /**
     * Called once per simulation loop. Concrete subclasses should override to update simulation
     * state (e.g., physics model inputs).
     */
    @Override
    public void simulationPeriodic() {}

    /**
     * Returns the human-readable name of this mechanism, as defined in its {@link Config}.
     *
     * @return the mechanism name
     */
    @Override
    public String getName() {
        return config.getName();
    }

    // ── Utility ────────────────────────────────────────────────────────────────

    /**
     * Returns {@code true} if physical hardware is attached and motor commands should be sent.
     *
     * @return {@code true} when hardware is available
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * Returns {@code true} when the leader motor is attached and its status frames are arriving
     * over CAN. A mechanism that is commanded but reports 0 V with this false has dropped off the
     * bus, which is what the hood did intermittently on the 2026-09-04 bench.
     *
     * @return {@code true} if the leader TalonFX is currently reachable
     */
    public boolean isMotorConnected() {
        return isAttached() && motor.isConnected();
    }

    /**
     * Reports the combined supply current draw of the leader motor and all followers to the battery
     * logger. Does nothing if the mechanism is not attached.
     */
    public void logBatteryUsage() {
        if (isAttached()) {
            // getSupplyCurrent() refreshes every signal for this loop, followers included.
            double motorCurrent = getSupplyCurrent();
            double followersCurrent = 0;
            for (int i = 0; i < followerMotors.length; i++) {
                double amps = followerSupplySignals[i].getValueAsDouble();
                followersCurrent += amps;
                checkFollowerAlive(i, amps, motorCurrent);
            }
            Robot.getBatteryLogger()
                    .reportCurrentUsage(batteryKey, motorCurrent + followersCurrent);
        }
    }

    /** Leader supply current above which a healthy follower must be pulling its share. */
    private static final double FOLLOWER_CHECK_LEADER_AMPS = 8.0;

    /** Follower supply current at or below which it is doing no work at all. */
    private static final double FOLLOWER_DEAD_AMPS = 0.5;

    /**
     * Seconds of accumulated mismatch before it is called out.
     *
     * <p>Accumulated rather than continuous, because nothing here stays loaded for long: a launch
     * burst runs one to three seconds, and the longest unbroken stretch of the launcher tower's
     * leader above the threshold in either 2026-09-05 log was 2.78 s. A rule wanting an
     * uninterrupted fault window would have watched that tower run all day on one motor and said
     * nothing. Short bursts add up instead.
     *
     * <p>Replayed against that day's logs, 1.5 s trips the dead tower follower in both logs while
     * the two healthy followers reach 0.00 s and 0.50 s -- three times the margin.
     */
    private static final double FOLLOWER_DEAD_SECONDS = 1.5;

    /** Ceiling on one loop's contribution, so a disabled stretch cannot bank a fault. */
    private static final double FOLLOWER_MAX_LOOP_SECONDS = 0.5;

    /**
     * Raises an alert when a follower stops pulling while its leader is clearly working.
     *
     * <p>A permanent follower is invisible when it dies. It shares a gearbox with its leader, so
     * the mechanism keeps moving on one motor at half the torque and nothing on a dashboard looks
     * wrong. On 2026-09-05 the launcher tower ran the whole day on one motor with its second one's
     * power lead off; it took reading the difference between {@code BatteryLogger/Current/
     * Mechanisms/*} (leader plus followers) and {@code LauncherTower/SupplyCurrent} (leader only)
     * across three logs to see it, because that difference was the only place the fault existed.
     *
     * <p>Now it says so itself. Each follower's draw is published on its own key, and a follower
     * that draws nothing while its leader is loaded raises a Driver Station error.
     *
     * <p>Note this only catches a dead <em>power</em> path. A follower off the CAN bus entirely
     * never updates its signal, so it reads a constant zero and trips this the same way -- which is
     * the right outcome, even if the message names the wrong wire.
     *
     * @param index which follower, indexing {@code config.followerConfigs}
     * @param followerAmps that follower's supply current this loop
     * @param leaderAmps the leader's supply current this loop
     */
    private void checkFollowerAlive(int index, double followerAmps, double leaderAmps) {
        if (index >= followerAlerts.length || followerAlerts[index] == null) {
            return;
        }
        // On the dashboard: the only place a dead follower is visible.
        if (Telemetry.slowLogThisLoop()) {
            Telemetry.logDash(followerCurrentKeys[index], followerAmps, "amps");
        }

        double now = Timer.getFPGATimestamp();
        double dt =
                Math.min(Math.max(now - followerCheckLastSeconds, 0), FOLLOWER_MAX_LOOP_SECONDS);
        if (index == followerMotors.length - 1) {
            followerCheckLastSeconds = now;
        }

        if (followerAmps > FOLLOWER_DEAD_AMPS) {
            // Any draw at all means the power path is intact; forget everything before it.
            followerMismatchSeconds[index] = 0;
        } else if (leaderAmps > FOLLOWER_CHECK_LEADER_AMPS) {
            followerMismatchSeconds[index] += dt;
        }

        followerAlerts[index].set(followerMismatchSeconds[index] >= FOLLOWER_DEAD_SECONDS);
    }

    /**
     * Returns the name of the command currently scheduled on this subsystem, or {@code "none"} if
     * no command is running.
     *
     * @return the current command name
     */
    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }
        return "none";
    }

    /**
     * Returns a {@link Trigger} that is active whenever this subsystem's current command is its
     * default command.
     *
     * @return trigger that is {@code true} while the default command is running
     */
    public Trigger runningDefaultCommand() {
        return new Trigger(this::isRunningDefaultCommand);
    }
    /**
     * Returns {@code true} if the running default command condition is met.
     *
     * @return {@code true} if the running default command condition is met
     */
    private boolean isRunningDefaultCommand() {
        return this.getCurrentCommand() == this.getDefaultCommand();
    }

    /**
     * Returns the last closed-loop setpoint (in rotations) sent to the motor by this class.
     *
     * @return the most recent target position in rotations
     */
    public double getTarget() {
        return target;
    }

    /**
     * Returns the last closed-loop velocity setpoint (in rotations per second) sent to the motor by
     * this class.
     *
     * @return the most recent target velocity in rotations per second
     */
    public double getVelocityTargetRPS() {
        return velocityTarget;
    }

    // ── Triggers ───────────────────────────────────────────────────────────────

    /**
     * Returns a {@link Trigger} that is active when the motor is within {@code tolerance} rotations
     * of the last commanded target position.
     *
     * @param tolerance maximum allowable error in rotations
     * @return trigger that is {@code true} when position error is within tolerance
     */
    public Trigger atTargetPosition(DoubleSupplier tolerance) {
        return new Trigger(() -> isAtTargetPosition(tolerance));
    }

    /**
     * Returns {@code true} when the motor is within {@code tolerance} rotations of the last
     * commanded target position.
     *
     * @param tolerance maximum allowable error in rotations
     * @return {@code true} when position error is within tolerance
     */
    public boolean isAtTargetPosition(DoubleSupplier tolerance) {
        return Math.abs(getPositionRotations() - target) < tolerance.getAsDouble();
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is within {@code tolerance}
     * of {@code target} (both in rotations).
     *
     * @param target the desired position in rotations
     * @param tolerance maximum allowable error in rotations
     * @return trigger that is {@code true} when position is within tolerance of target
     */
    public Trigger atRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionRotations() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns {@code true} when the motor position is within {@code tolerance} of {@code target}
     * (both in rotations).
     *
     * @param target the desired position in rotations
     * @param tolerance maximum allowable error in rotations
     * @return {@code true} when position is within tolerance of target
     */
    public boolean isAtRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return Math.abs(getPositionRotations() - target.getAsDouble()) < tolerance.getAsDouble();
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is below {@code target +
     * tolerance} rotations.
     *
     * @param target reference position in rotations
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when position is below the threshold
     */
    public Trigger belowRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is above {@code target -
     * tolerance} rotations.
     *
     * @param target reference position in rotations
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when position is above the threshold
     */
    public Trigger aboveRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is within {@code tolerance}
     * of {@code target} (both as a percentage of max rotations).
     *
     * @param target the desired position as a percentage of max rotations
     * @param tolerance maximum allowable error as a percentage
     * @return trigger that is {@code true} when position is within tolerance of target
     */
    public Trigger atPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionPercentage() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is below {@code target +
     * tolerance} (as a percentage of max rotations).
     *
     * @param target reference position as a percentage
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when position is below the threshold
     */
    public Trigger belowPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is above {@code target -
     * tolerance} (as a percentage of max rotations).
     *
     * @param target reference position as a percentage
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when position is above the threshold
     */
    public Trigger abovePercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is within {@code tolerance}
     * of {@code target} (both in degrees).
     *
     * @param target the desired position in degrees
     * @param tolerance maximum allowable error in degrees
     * @return trigger that is {@code true} when position is within tolerance of target
     */
    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is below {@code target +
     * tolerance} degrees.
     *
     * @param target reference position in degrees
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when position is below the threshold
     */
    public Trigger belowDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is above {@code target -
     * tolerance} degrees.
     *
     * @param target reference position in degrees
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when position is above the threshold
     */
    public Trigger aboveDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor velocity is within {@code tolerance}
     * RPM of {@code target}.
     *
     * @param target the desired velocity in RPM
     * @param tolerance maximum allowable error in RPM
     * @return trigger that is {@code true} when velocity is within tolerance of target
     */
    public Trigger atVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> Math.abs(getVelocityRPM() - target.getAsDouble()) < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor velocity is below {@code target +
     * tolerance} RPM.
     *
     * @param target reference velocity in RPM
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when velocity is below the threshold
     */
    public Trigger belowVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor velocity is above {@code target -
     * tolerance} RPM.
     *
     * @param target reference velocity in RPM
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when velocity is above the threshold
     */
    public Trigger aboveVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor stator current is within {@code
     * tolerance} amps of {@code target}.
     *
     * @param target the desired stator current in amps
     * @param tolerance maximum allowable error in amps
     * @return trigger that is {@code true} when stator current is within tolerance of target
     */
    public Trigger atCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getStatorCurrent() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor stator current is below {@code target
     * + tolerance} amps.
     *
     * @param target reference current in amps
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when stator current is below the threshold
     */
    public Trigger belowCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor stator current is above {@code target
     * - tolerance} amps.
     *
     * @param target reference current in amps
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when stator current is above the threshold
     */
    public Trigger aboveCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    // ── Sensor Readings ────────────────────────────────────────────────────────

    /**
     * Refreshes every status signal this mechanism reads, once per robot loop, in one Phoenix call.
     *
     * <p>Every Phoenix getter such as {@code motor.getStatorCurrent()} defaults to refreshing its
     * signal, which is a JNI call each. With eight signals per mechanism read once or more a loop
     * that was over a hundred JNI calls per loop across the robot. CTRE's recommendation is one
     * {@code refreshAll} for the lot, which is what this does; every read in the same loop then
     * sees the same sample, which is the behaviour the per-value caches used to provide.
     */
    private void refreshSignalsOncePerLoop() {
        long loop = RobotLoop.count();
        if (loop == lastSignalRefreshLoop || loopSignals.length == 0) {
            return;
        }
        lastSignalRefreshLoop = loop;
        BaseStatusSignal.refreshAll(loopSignals);
    }

    /** This loop's value of one of the leader's signals, or 0 when there is no hardware. */
    private double signalValue(BaseStatusSignal signal) {
        if (!config.attached) {
            return 0;
        }
        refreshSignalsOncePerLoop();
        return signal.getValueAsDouble();
    }

    /**
     * Returns the stator current of the motor as of this loop.
     *
     * @return motor stator current in amps
     */
    public double getStatorCurrent() {
        return signalValue(statorCurrentSignal);
    }

    /**
     * Returns the supply current of the motor as of this loop.
     *
     * @return motor supply current in amps
     */
    public double getSupplyCurrent() {
        return signalValue(supplyCurrentSignal);
    }

    /**
     * Returns the applied motor voltage as of this loop.
     *
     * @return motor voltage in volts
     */
    public double getVoltage() {
        return signalValue(voltageSignal);
    }

    /**
     * Returns the motor temperature as of this loop.
     *
     * @return motor temperature in Celsius
     */
    public double getTemp() {
        return signalValue(tempSignal);
    }

    /**
     * Logs voltage, stator and supply current, temperature and connection under {@code prefix/...},
     * at 10 Hz. Subclasses call this from {@code periodic()} in place of the five log lines each
     * used to carry; the keys are built on the first call.
     *
     * @param prefix log key prefix, e.g. {@code "Hood"}
     */
    protected void logDiagnostics(String prefix) {
        logDiagnostics(prefix, false);
    }

    /**
     * As {@link #logDiagnostics(String)}, optionally also publishing the values to the dashboard.
     *
     * @param prefix log key prefix, e.g. {@code "Turret"}
     * @param dashboard {@code true} to publish through {@link Telemetry#logDash}
     */
    protected void logDiagnostics(String prefix, boolean dashboard) {
        if (!prefix.equals(diagnosticsPrefix)) {
            diagnosticsPrefix = prefix;
            voltageKey = prefix + "/Voltage";
            statorCurrentKey = prefix + "/StatorCurrent";
            supplyCurrentKey = prefix + "/SupplyCurrent";
            tempKey = prefix + "/Temp";
            motorConnectedKey = prefix + "/MotorConnected";
        }
        if (!Telemetry.slowLogThisLoop()) {
            // A feedforward fit needs the applied voltage on every velocity sample, not one in
            // five.
            if (config.isFastOutputLogging()) {
                Telemetry.log(voltageKey, getVoltage(), "volts");
            }
            return;
        }
        if (dashboard) {
            Telemetry.logDash(voltageKey, getVoltage(), "volts");
            Telemetry.logDash(statorCurrentKey, getStatorCurrent(), "amps");
            Telemetry.logDash(supplyCurrentKey, getSupplyCurrent(), "amps");
            Telemetry.logDash(tempKey, getTemp(), "deg_C");
            Telemetry.logDash(motorConnectedKey, isMotorConnected());
        } else {
            Telemetry.log(voltageKey, getVoltage(), "volts");
            Telemetry.log(statorCurrentKey, getStatorCurrent(), "amps");
            Telemetry.log(supplyCurrentKey, getSupplyCurrent(), "amps");
            Telemetry.log(tempKey, getTemp(), "deg_C");
            Telemetry.log(motorConnectedKey, isMotorConnected());
        }
    }

    // ── Unit Conversions ───────────────────────────────────────────────────────

    /**
     * Converts a percentage of the mechanism's maximum range into an absolute rotation count.
     *
     * @param percent position as a percentage of max rotations (0–100)
     * @return the equivalent position in rotations
     */
    public double percentToRotations(DoubleSupplier percent) {
        return (percent.getAsDouble() / 100) * config.maxRotations;
    }

    /**
     * Converts an absolute rotation count into a percentage of the mechanism's maximum range.
     *
     * @param rotations position in rotations
     * @return the equivalent percentage of max rotations (0–100)
     */
    public double rotationsToPercent(DoubleSupplier rotations) {
        return (rotations.getAsDouble() / config.maxRotations) * 100;
    }

    /**
     * Converts degrees to rotations (1 rotation = 360 degrees).
     *
     * @param degrees angle in degrees
     * @return the equivalent position in rotations
     */
    public double degreesToRotations(DoubleSupplier degrees) {
        return (degrees.getAsDouble() / 360);
    }

    /**
     * Converts rotations to degrees (1 rotation = 360 degrees).
     *
     * @param rotations position in rotations
     * @return the equivalent angle in degrees
     */
    public double rotationsToDegrees(DoubleSupplier rotations) {
        return 360 * rotations.getAsDouble();
    }

    // ── Position & Velocity ────────────────────────────────────────────────────

    /**
     * Returns the motor position in rotations as of this loop.
     *
     * @return motor position in rotations
     */
    public double getPositionRotations() {
        return signalValue(positionSignal);
    }

    /**
     * Returns the motor position in rotations projected from the signal's own timestamp to now
     * along the velocity signal, so a mechanism moving at speed reads where it is rather than where
     * it was when the CAN frame left the motor.
     *
     * @return latency-compensated motor position in rotations, or {@code 0} if not attached
     */
    public double getLatencyCompensatedPositionRotations() {
        if (!config.attached || positionStatusSignal == null) {
            return 0;
        }
        refreshSignalsOncePerLoop();
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                positionStatusSignal, velocityStatusSignal);
    }

    /**
     * Same as {@link #getLatencyCompensatedPositionRotations()} in degrees.
     *
     * @return latency-compensated motor position in degrees, or {@code 0} if not attached
     */
    public double getLatencyCompensatedPositionDegrees() {
        return rotationsToDegrees(this::getLatencyCompensatedPositionRotations);
    }

    /**
     * Returns the motor position as a percentage of max rotations as of this loop.
     *
     * @return motor position in percentage of max rotations (0–100)
     */
    public double getPositionPercentage() {
        return rotationsToPercent(this::getPositionRotations);
    }

    /**
     * Returns the motor position in degrees as of this loop.
     *
     * @return motor position in degrees
     */
    public double getPositionDegrees() {
        return rotationsToDegrees(this::getPositionRotations);
    }

    /**
     * Same as {@link #getPositionDegrees()}. The per-loop signal refresh is keyed on the robot loop
     * counter rather than the scheduler, so a caller that runs before {@code
     * CommandScheduler.run()} (Vision, in {@code Robot.robotPeriodic()}) already gets this loop's
     * sample. Kept so existing callers need not change.
     *
     * @return motor position in degrees, or {@code 0} if not attached
     */
    public double getPositionDegreesUncached() {
        return getPositionDegrees();
    }

    /**
     * Returns the motor velocity in RPM as of this loop.
     *
     * @return motor velocity in revolutions per minute
     */
    public double getVelocityRPM() {
        return Conversions.RPStoRPM(signalValue(velocitySignal));
    }

    // ── Command Factories ──────────────────────────────────────────────────────

    /**
     * Returns a {@link Command} that continuously drives the mechanism at the specified velocity
     * using closed-loop voltage control.
     *
     * @param velocityRPM the target velocity in revolutions per minute
     * @return a command that runs the mechanism at the given velocity
     */
    public Command runVelocity(DoubleSupplier velocityRPM) {
        return run(() -> setVelocity(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocity");
    }

    /**
     * Returns a {@link Command} that continuously drives the mechanism at the specified velocity
     * using closed-loop Torque Current FOC control (requires Phoenix Pro).
     *
     * @param velocityRPM the target velocity in revolutions per minute
     * @return a command that runs the mechanism at the given velocity using torque current FOC
     */
    public Command runVelocityTcFocRPM(DoubleSupplier velocityRPM) {
        return run(() -> setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocityTcFocRPM");
    }

    /**
     * Returns a {@link Command} that continuously applies an open-loop percent output to the
     * mechanism using voltage compensation.
     *
     * @param percent fractional output between -1 and +1
     * @return a command that runs the mechanism at the given percent output
     */
    public Command runPercentage(DoubleSupplier percent) {
        return run(() -> setPercentOutput(percent)).withName(getName() + ".runPercentage");
    }

    /**
     * Returns a {@link Command} that continuously applies the specified voltage to the mechanism,
     * bypassing any closed-loop control.
     *
     * @param voltage the desired voltage in volts
     * @return a command that applies the given voltage output
     */
    public Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltageOutput(voltage)).withName(getName() + ".runVoltage");
    }

    /**
     * Returns a {@link Command} that continuously applies the specified voltage to the mechanism,
     * bypassing closed-loop control <em>and</em> ignoring software limit switches.
     *
     * @param voltage the desired voltage in volts
     * @return a command that applies the given voltage output, ignoring software limits
     */
    public Command runVoltageNoSoftLimit(DoubleSupplier voltage) {
        return run(() -> setVoltageOutputNoSoftLimit(voltage))
                .withName(getName() + ".runVoltageNoSoftLimit");
    }

    /**
     * Returns a {@link Command} that continuously drives the mechanism at the specified torque
     * current using FOC control (requires Phoenix Pro).
     *
     * @param current the desired torque current in amps
     * @return a command that runs the mechanism at the given torque current
     */
    public Command runTorqueCurrentFoc(DoubleSupplier current) {
        return run(() -> setTorqueCurrentFoc(current)).withName(getName() + ".runTorqueCurrentFoc");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified position
     * using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * @param rotations the target position in rotations
     * @return a command that moves the mechanism to the given position
     */
    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runPoseRevolutions");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified position
     * using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * @param percent the target position as a percentage of max rotations (0–100)
     * @return a command that moves the mechanism to the given percentage position
     */
    public Command moveToPercentage(DoubleSupplier percent) {
        return run(() -> setMMPositionFoc(() -> percentToRotations(percent)))
                .withName(getName() + ".runPosePercentage");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified angular
     * position using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * @param degrees the target position in degrees
     * @return a command that moves the mechanism to the given position in degrees
     */
    public Command moveToDegrees(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(() -> degreesToRotations(degrees)))
                .withName(getName() + ".runPoseDegrees");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified position
     * using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * <p>Equivalent to {@link #moveToRotations(DoubleSupplier)} — prefer that method for clarity.
     *
     * @param rotations the target position in rotations
     * @return a command that moves the mechanism to the given position
     */
    public Command runFocRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runFOCPosition");
    }

    /**
     * Returns a {@link Command} that stops the mechanism and holds it stopped for its duration.
     *
     * @return a command that stops the mechanism
     */
    public Command runStop() {
        return run(this::stop).withName(getName() + ".runStop");
    }

    /**
     * Returns a {@link Command} that sets the mechanism to coast mode while active, then reverts to
     * brake mode when the command ends. Safe to run while the robot is disabled.
     *
     * @return a command that temporarily enables coast mode
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName(getName() + ".coastMode");
    }

    /**
     * Returns a {@link Command} that sets the mechanism to brake mode if it is currently in coast
     * mode. Safe to run while the robot is disabled.
     *
     * @return a command that ensures brake mode is active
     */
    public Command ensureBrakeMode() {
        return runOnce(() -> setBrakeMode(true))
                .onlyIf(
                        () ->
                                config.attached
                                        && config.talonConfig.MotorOutput.NeutralMode
                                                == NeutralModeValue.Coast)
                .ignoringDisable(true)
                .withName(getName() + ".ensureBrakeMode");
    }

    /**
     * Returns a {@link Command} that applies new supply and stator current limits to the mechanism.
     *
     * @param supplyLimit the new supply current limit in amps
     * @param statorLimit the new stator current limit in amps
     * @return a command that updates the current limits
     */
    protected Command runCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        return Commands.runOnce(() -> setCurrentLimits(supplyLimit, statorLimit));
    }

    // ── Motor Control (Protected) ──────────────────────────────────────────────

    /**
     * Immediately applies new supply and stator current limits to the motor configuration.
     *
     * @param supplyLimit the new supply current limit in amps
     * @param statorLimit the new stator current limit in amps
     */
    protected void setCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        applyCurrentLimit(supplyLimit, statorLimit);
    }

    /** Stops the motor output. Does nothing if the mechanism is not attached. */
    protected void stop() {
        if (isAttached()) {
            motor.stopMotor();
        }
    }

    /**
     * Sets the mechanism's reported position to zero (tares the motor encoder). Does nothing if the
     * mechanism is not attached.
     */
    protected void tareMotor() {
        if (isAttached()) {
            setMotorPosition(() -> 0);
        }
    }

    /**
     * Sets the motor's internal position register to the specified value without moving the motor.
     *
     * @param rotations the position to write to the motor in rotations
     */
    protected void setMotorPosition(DoubleSupplier rotations) {
        if (isAttached()) {
            motor.setPosition(rotations.getAsDouble());
        }
    }

    /**
     * Closed-loop velocity control using Motion Magic with Torque Current FOC (requires Phoenix
     * Pro).
     *
     * @param velocityRPS the target velocity in rotations per second
     */
    protected void setMMVelocityFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            MotionMagicVelocityTorqueCurrentFOC mm =
                    config.mmVelocityFOC.withVelocity(velocityTarget);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop velocity control using Torque Current FOC (requires Phoenix Pro).
     *
     * @param velocityRPS the target velocity in rotations per second
     */
    protected void setVelocityTorqueCurrentFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop velocity control using Torque Current FOC with an RPM input (requires Phoenix
     * Pro). The RPM value is converted to RPS internally before being sent to the motor.
     *
     * @param velocityRPM the target velocity in revolutions per minute
     */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPM) {
        if (isAttached()) {
            velocityTarget = Conversions.RPMtoRPS(velocityRPM.getAsDouble());
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop velocity control with voltage compensation.
     *
     * @param velocityRPS the target velocity in rotations per second
     */
    protected void setVelocity(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            VelocityVoltage output = config.velocityControl.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop velocity control with voltage compensation, taking RPM. The RPM value is
     * converted to RPS internally before being sent to the motor.
     *
     * @param velocityRPM the target velocity in revolutions per minute
     */
    protected void setVelocityRPM(DoubleSupplier velocityRPM) {
        setVelocity(() -> Conversions.RPMtoRPS(velocityRPM.getAsDouble()));
    }

    /**
     * Closed-loop position control with voltage compensation.
     *
     * @param rotations the target position in rotations
     */
    protected void setPosition(DoubleSupplier rotations) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            PositionVoltage output = config.positionControl.withPosition(target).withVelocity(0);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop position control with voltage compensation and an explicit velocity feedforward.
     * This is the correct primitive for tracking a continuously moving setpoint (e.g. a turret
     * aiming at a target while the robot drives), where profiling would introduce steady-state lag.
     *
     * @param rotations the target position in mechanism rotations
     * @param velocityRPS the velocity feedforward in mechanism rotations per second
     */
    protected void setPositionWithVelocity(DoubleSupplier rotations, DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            PositionVoltage output =
                    config.positionControl
                            .withPosition(target)
                            .withVelocity(velocityRPS.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop position control using Motion Magic with Torque Current FOC (requires Phoenix
     * Pro).
     *
     * @param rotations the target position in rotations
     */
    protected void setMMPositionFoc(DoubleSupplier rotations) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicTorqueCurrentFOC mm = config.mmPositionFOC.withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop position control using {@link PositionTorqueCurrentFOC} with an explicit velocity
     * feedforward. This is the correct primitive for tracking a continuously moving setpoint (e.g.
     * a turret aiming at a target while the robot drives), where profiling would introduce
     * steady-state lag.
     *
     * @param rotations the target position in mechanism rotations
     * @param velocityRPS the velocity feedforward in mechanism rotations per second
     */
    protected void setPositionFocWithVelocity(
            DoubleSupplier rotations, DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            PositionTorqueCurrentFOC req =
                    config.positionTorqueFOC
                            .withPosition(target)
                            .withVelocity(velocityRPS.getAsDouble());
            motor.setControl(req);
        }
    }

    /**
     * Closed-loop position control using Dynamic Motion Magic with Torque Current FOC (requires
     * Phoenix Pro). Trajectory parameters can be changed every loop cycle.
     *
     * @param rotations the target position in rotations
     * @param velocity the cruise velocity in rotations per second
     * @param acceleration the acceleration in rotations per second squared
     * @param jerk the jerk in rotations per second cubed
     */
    protected void setDynMMPositionFoc(
            DoubleSupplier rotations,
            DoubleSupplier velocity,
            DoubleSupplier acceleration,
            DoubleSupplier jerk) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            DynamicMotionMagicTorqueCurrentFOC mm =
                    config.dynamicMMPositionFOC
                            .withPosition(target)
                            .withVelocity(velocity.getAsDouble())
                            .withAcceleration(acceleration.getAsDouble())
                            .withJerk(jerk.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop position control using Dynamic Motion Magic with voltage compensation. Trajectory
     * parameters can be changed every loop cycle.
     *
     * @param rotations the target position in rotations
     * @param velocity the cruise velocity in rotations per second
     * @param acceleration the acceleration in rotations per second squared
     * @param jerk the jerk in rotations per second cubed
     */
    protected void setDynMMPositionVoltage(
            DoubleSupplier rotations,
            DoubleSupplier velocity,
            DoubleSupplier acceleration,
            DoubleSupplier jerk) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            DynamicMotionMagicVoltage mm =
                    config.dynamicMotionMagicVoltage
                            .withPosition(target)
                            .withVelocity(velocity.getAsDouble())
                            .withAcceleration(acceleration.getAsDouble())
                            .withJerk(jerk.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop position control using Motion Magic with voltage compensation (slot 0).
     *
     * @param rotations the target position in rotations
     */
    protected void setMMPosition(DoubleSupplier rotations) {
        setMMPosition(rotations, 0);
    }

    /**
     * Closed-loop position control using Motion Magic with voltage compensation and an explicit
     * PID/FF gain slot.
     *
     * @param rotations the target position in rotations
     * @param slot the gain slot to use (0, 1, or 2)
     */
    protected void setMMPosition(DoubleSupplier rotations, int slot) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot.withSlot(slot).withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Open-loop percent output control with voltage compensation. The output voltage is {@code
     * percent × voltageCompSaturation}.
     *
     * @param percent fractional output between -1 and +1
     */
    protected void setPercentOutput(DoubleSupplier percent) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl.withOutput(
                            config.voltageCompSaturation * percent.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Open-loop voltage control — applies the requested voltage directly without compensation
     * scaling.
     *
     * @param voltage the desired voltage in volts
     */
    protected void setVoltageOutput(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output = config.voltageControl.withOutput(voltage.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Open-loop voltage control that ignores software limit switches. Use with caution — this can
     * drive the mechanism past its configured travel limits.
     *
     * @param voltage the desired voltage in volts
     */
    protected void setVoltageOutputNoSoftLimit(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl
                            .withOutput(voltage.getAsDouble())
                            .withIgnoreSoftwareLimits(true);
            motor.setControl(output);
        }
    }

    /**
     * Applies a torque current setpoint using FOC control (requires Phoenix Pro).
     *
     * @param current the desired torque current in amps
     */
    public void setTorqueCurrentFoc(DoubleSupplier current) {
        if (isAttached()) {
            TorqueCurrentFOC output = config.torqueCurrentFOC.withOutput(current.getAsDouble());
            motor.setControl(output);
        }
    }

    // ── Hardware Configuration ─────────────────────────────────────────────────

    /**
     * Sets the motor's neutral mode to brake or coast and immediately applies the change to
     * hardware.
     *
     * @param isInBrake {@code true} to set brake mode; {@code false} to set coast mode
     */
    public void setBrakeMode(boolean isInBrake) {
        if (isAttached()) {
            config.configNeutralBrakeMode(isInBrake);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables the reverse software limit switch and immediately applies the change. The
     * threshold is read from the current configuration.
     *
     * @param enabled {@code true} to enable the reverse soft limit; {@code false} to disable it
     */
    public void toggleReverseSoftLimit(boolean enabled) {
        if (isAttached()) {
            double threshold = config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
            config.configReverseSoftLimit(threshold, enabled);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables a forward/reverse torque current limit and immediately applies the
     * change. When disabled, the peak torque current is reset to ±300 A (effectively unlimited).
     *
     * @param enabledLimit the torque current limit in amps when {@code enabled} is {@code true}
     * @param enabled {@code true} to apply the limit; {@code false} to remove it
     */
    public void toggleTorqueCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configForwardTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.configReverseTorqueCurrentLimit(-1 * enabledLimit.getAsDouble());
                config.configStatorCurrentLimit(enabledLimit.getAsDouble(), true);
                config.applyTalonConfig(motor);
            } else {
                config.configForwardTorqueCurrentLimit(300);
                config.configReverseTorqueCurrentLimit(-300);
                config.applyTalonConfig(motor);
            }
        }
    }

    /**
     * Enables or disables the supply current limit and immediately applies the change.
     *
     * @param enabledLimit the supply current limit in amps
     * @param enabled {@code true} to enable the limit; {@code false} to disable it
     */
    public void toggleSupplyCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configSupplyCurrentLimit(enabledLimit.getAsDouble(), true);
                config.applyTalonConfig(motor);
            } else {
                config.configSupplyCurrentLimit(enabledLimit.getAsDouble(), false);
                config.applyTalonConfig(motor);
            }
        }
    }

    /**
     * Applies new supply and stator current limits if the requested values differ from the
     * currently configured limits. The update is retried up to 10 times on failure.
     *
     * @param supplyLimit the new supply current limit in amps
     * @param statorLimit the new stator current limit in amps
     */
    public void applyCurrentLimit(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        if (isAttached()) {
            if (config.talonConfig.CurrentLimits.StatorCurrentLimit != statorLimit.getAsDouble()
                    || config.talonConfig.CurrentLimits.SupplyCurrentLimit
                            != supplyLimit.getAsDouble()) {
                config.configSupplyCurrentLimit(Math.abs(supplyLimit.getAsDouble()), true);
                config.configStatorCurrentLimit(Math.abs(statorLimit.getAsDouble()), true);
                config.configForwardTorqueCurrentLimit(Math.abs(statorLimit.getAsDouble()));
                config.configReverseTorqueCurrentLimit(-1 * Math.abs(statorLimit.getAsDouble()));
                int attempts = CanConfigBudget.maxAttempts();
                for (int i = 0; i < attempts; i++) {
                    StatusCode result =
                            CanConfigBudget.run(
                                    config.getName(),
                                    timeout ->
                                            motor.getConfigurator()
                                                    .apply(config.talonConfig, timeout));
                    if (!result.isOK()) {
                        System.out.println(
                                "Could not apply config changes to "
                                        + config.getName()
                                        + "\'s motor ");
                    } else {
                        break;
                    }
                }
            }
        }
    }

    // ── Diagnostic Commands ────────────────────────────────────────────────────

    /**
     * Returns a {@link Command} that measures the average stator current over its runtime and fires
     * a warning {@link Alert} if the average deviates from {@code expectedCurrent} by more than
     * {@code tolerance}.
     *
     * @param expectedCurrent the expected average stator current in amps
     * @param tolerance the maximum acceptable deviation in amps
     * @return a diagnostic command that checks average current
     */
    public Command checkAvgCurrent(DoubleSupplier expectedCurrent, DoubleSupplier tolerance) {
        return new Command() {
            double totalCurrent = 0;
            int count = 0;
            String alertText = config.name + " AvgCurrent Error";

            @Override
            public void initialize() {
                totalCurrent = 0;
                count = 0;
            }

            @Override
            public void execute() {
                totalCurrent += getStatorCurrent();
                count++;
            }

            @Override
            public void end(boolean interrupted) {
                double avgCurrent = totalCurrent / count;
                if (Math.abs(avgCurrent - expectedCurrent.getAsDouble())
                        > tolerance.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + avgCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    /**
     * Returns a {@link Command} that tracks the peak stator current over its runtime and fires a
     * warning {@link Alert} if the peak exceeds {@code expectedCurrent}.
     *
     * @param expectedCurrent the maximum acceptable peak stator current in amps
     * @return a diagnostic command that checks peak current
     */
    public Command checkMaxCurrent(DoubleSupplier expectedCurrent) {
        return new Command() {
            double maxCurrent = 0;
            String alertText = config.name + " MaxCurrent Error";

            @Override
            public void initialize() {
                maxCurrent = 0;
            }

            @Override
            public void execute() {
                double current = getStatorCurrent();
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (maxCurrent > expectedCurrent.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + maxCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    /**
     * Returns a {@link Command} that tracks the peak stator current over its runtime and fires a
     * warning {@link Alert} if the peak never reaches {@code expectedCurrent}. Use this to verify
     * that a mechanism drew at least the expected minimum load.
     *
     * @param expectedCurrent the minimum acceptable peak stator current in amps
     * @return a diagnostic command that checks whether a minimum current threshold was reached
     */
    public Command checkMinThresholdCurrent(DoubleSupplier expectedCurrent) {
        return new Command() {
            double maxCurrent = 0;
            String alertText = config.name + " Current Error";

            @Override
            public void initialize() {
                maxCurrent = 0;
            }

            @Override
            public void execute() {
                double current = getStatorCurrent();
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (maxCurrent < expectedCurrent.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected at least: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + maxCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    // ── Nested Classes ─────────────────────────────────────────────────────────

    /**
     * Configuration for a TalonFX follower motor that mirrors the leader.
     *
     * <p>A follower automatically copies the leader's output. Set {@code opposeLeader} to {@link
     * MotorAlignmentValue#Opposed} when the physical motor is mounted in the opposite direction and
     * must spin in reverse to produce the same mechanism motion.
     */
    public static class FollowerConfig {

        /** Human-readable name for this follower motor (used in alerts and logging). */
        @Getter private String name;

        /** CAN bus device ID and bus name for this follower motor. */
        @Getter private CanDeviceId id;

        /** Whether hardware is attached for this follower. */
        @Getter private boolean attached = true;

        /**
         * Alignment of the follower relative to the leader. Use {@link MotorAlignmentValue#Opposed}
         * when the follower is physically mounted in the opposite direction.
         */
        @Getter private MotorAlignmentValue opposeLeader = MotorAlignmentValue.Aligned;

        /**
         * Creates a follower motor configuration.
         *
         * @param name human-readable name for this follower
         * @param id CAN device ID
         * @param canbus CAN bus name (e.g., {@code "rio"} or {@code "canivore"})
         * @param opposeLeader alignment relative to the leader motor
         */
        public FollowerConfig(
                String name, int id, String canbus, MotorAlignmentValue opposeLeader) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    /**
     * Configuration for a {@link Mechanism}, encapsulating the TalonFX hardware configuration,
     * control request objects, and mechanism-level parameters (gear ratio, soft limits, PID/FF
     * gains, Motion Magic profile, current limits, etc.).
     *
     * <p>Subclass this in each concrete mechanism and call the {@code config*()} helpers in the
     * constructor to set mechanism-specific parameters before passing the config to the {@link
     * Mechanism} constructor.
     */
    public static class Config {

        /** Human-readable name for this mechanism (used in logging and alerts). */
        @Getter private String name;

        /**
         * Whether physical hardware is attached. Set to {@code false} to run in simulation-only
         * mode.
         */
        @Getter @Setter private boolean attached = true;

        /**
         * Publish the leader's output signals (duty cycle, motor voltage, torque current) at the
         * control rate and log its applied voltage on every loop instead of at 10 Hz. Set it on the
         * mechanisms whose feedforward is fit from logs: kS and kV survive a 10 Hz voltage, kA does
         * not, because the acceleration it multiplies is gone between one sample and the next.
         * Costs one status frame at 100 Hz on this motor and one double per loop in the log; the
         * followers are untouched.
         */
        @Getter @Setter private boolean fastOutputLogging = false;

        /** CAN bus device ID and bus name for the leader motor. */
        @Getter private CanDeviceId id;

        /** Full TalonFX hardware configuration applied to the leader motor on startup. */
        @Getter @Setter protected TalonFXConfiguration talonConfig;

        /** Total number of motors (leader + followers). */
        @Getter private int numMotors = 1;

        /**
         * Voltage compensation saturation value used by {@link Mechanism#setPercentOutput}.
         * Defaults to 12 V.
         */
        @Getter private double voltageCompSaturation = 12.0;

        /** Minimum mechanism position in rotations (used for range calculations). */
        @Getter private double minRotations = 0;

        /** Maximum mechanism position in rotations (used for range calculations). */
        @Getter private double maxRotations = 1;

        /** Configurations for follower motors. Empty by default (no followers). */
        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        // Pre-built control request objects — reused each loop to avoid GC pressure.

        @Getter
        private MotionMagicVelocityTorqueCurrentFOC mmVelocityFOC =
                new MotionMagicVelocityTorqueCurrentFOC(0);

        @Getter
        private MotionMagicTorqueCurrentFOC mmPositionFOC = new MotionMagicTorqueCurrentFOC(0);

        @Getter
        private PositionTorqueCurrentFOC positionTorqueFOC = new PositionTorqueCurrentFOC(0);

        @Getter
        private DynamicMotionMagicTorqueCurrentFOC dynamicMMPositionFOC =
                new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0);

        @Getter
        private DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
                new DynamicMotionMagicVoltage(0, 0, 0);

        @Getter
        private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0);

        @Getter private MotionMagicVoltage mmPositionVoltage = new MotionMagicVoltage(0);

        @Getter
        private MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);

        @Getter private VoltageOut voltageControl = new VoltageOut(0);
        @Getter private VelocityVoltage velocityControl = new VelocityVoltage(0);
        @Getter private PositionVoltage positionControl = new PositionVoltage(0);

        @Getter
        private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        @Getter private TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);

        /** Percent (duty-cycle) output control — prefer {@link #voltageControl} in most cases. */
        @Getter private DutyCycleOut percentOutput = new DutyCycleOut(0);

        // ── Constructor ───────────────────────────────────────────────────────

        /**
         * Creates a base mechanism configuration with default Talon settings. Hardware limit
         * switches are disabled by default.
         *
         * @param name human-readable name for this mechanism
         * @param id CAN device ID of the leader motor
         * @param canbus CAN bus name (e.g., {@code "rio"} or {@code "canivore"})
         */
        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();

            /* Put default config settings for all mechanisms here */
            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        // ── Config Helpers ────────────────────────────────────────────────────

        /**
         * Applies the current {@link TalonFXConfiguration} to the given motor and reports a warning
         * to the DriverStation if the apply fails.
         *
         * @param talon the TalonFX motor to configure
         */
        public void applyTalonConfig(TalonFX talon) {
            StatusCode result =
                    CanConfigBudget.run(
                            name, timeout -> talon.getConfigurator().apply(talonConfig, timeout));
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to " + name + "\'s motor ", false);
            }
        }

        /**
         * Sets the follower motor configurations.
         *
         * @param followers one or more {@link FollowerConfig} objects describing follower motors
         */
        public void setFollowerConfigs(FollowerConfig... followers) {
            followerConfigs = followers;
        }

        /**
         * Sets the voltage compensation saturation voltage used by percent-output control.
         *
         * @param voltageCompSaturation the saturation voltage in volts (typically 12.0)
         */
        public void configVoltageCompensation(double voltageCompSaturation) {
            this.voltageCompSaturation = voltageCompSaturation;
        }

        /**
         * Configures the motor output as counter-clockwise positive (default for most mechanisms
         * when viewed from the shaft end).
         */
        public void configCounterClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        /** Configures the motor output as clockwise positive (inverted relative to the default). */
        public void configClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        /**
         * Sets the peak forward output voltage.
         *
         * @param voltageLimit maximum forward voltage in volts
         */
        public void configForwardVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakForwardVoltage = voltageLimit;
        }

        /**
         * Sets the peak reverse output voltage.
         *
         * @param voltageLimit maximum reverse voltage in volts
         */
        public void configReverseVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakReverseVoltage = voltageLimit;
        }

        /**
         * Configures the supply current limit. The absolute value of {@code supplyLimit} is used,
         * so negative values are automatically corrected.
         *
         * @param supplyLimit the supply current limit in amps
         * @param enabled {@code true} to enable the limit
         */
        public void configSupplyCurrentLimit(double supplyLimit, boolean enabled) {
            if (supplyLimit < 0) {
                supplyLimit = -supplyLimit;
            }
            talonConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
        }

        /**
         * Configures the stator current limit. The absolute value of {@code statorLimit} is used,
         * so negative values are automatically corrected.
         *
         * @param statorLimit the stator current limit in amps
         * @param enabled {@code true} to enable the limit
         */
        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            if (statorLimit < 0) {
                statorLimit = -statorLimit;
            }
            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        /**
         * Sets the peak forward torque current limit. The absolute value is used so negative inputs
         * are corrected automatically.
         *
         * @param currentLimit peak forward torque current in amps
         */
        public void configForwardTorqueCurrentLimit(double currentLimit) {
            if (currentLimit < 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        }

        /**
         * Sets the open-loop ramp period for duty cycle, voltage, and torque control.
         *
         * @param seconds time to ramp from neutral to full output
         */
        public void configOpenLoopRamps(double seconds) {
            talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = seconds;
            talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = seconds;
            talonConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = seconds;
        }

        /**
         * Sets the closed-loop ramp period for duty cycle, voltage, and torque control.
         *
         * @param seconds time to ramp from neutral to full output
         */
        public void configClosedLoopRamps(double seconds) {
            talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = seconds;
            talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = seconds;
            talonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = seconds;
        }

        /**
         * Sets the peak reverse torque current limit. The value is forced negative so positive
         * inputs are corrected automatically.
         *
         * @param currentLimit peak reverse torque current in amps (sign is corrected if positive)
         */
        public void configReverseTorqueCurrentLimit(double currentLimit) {
            if (currentLimit > 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        }

        /**
         * Sets the lower supply current limit, used to reduce dissipation after the upper limit has
         * been triggered.
         *
         * @param currentLimit the lower supply current limit in amps
         */
        public void configLowerSupplyCurrentLimit(double currentLimit) {
            talonConfig.CurrentLimits.SupplyCurrentLowerLimit = currentLimit;
        }

        /**
         * Sets the time window for the lower supply current limit.
         *
         * @param time the time in seconds
         */
        public void configLowerSupplyCurrentTime(double time) {
            talonConfig.CurrentLimits.SupplyCurrentLowerTime = time;
        }

        /**
         * Sets the duty-cycle neutral deadband. Outputs below this magnitude are treated as zero.
         *
         * @param deadband deadband as a fraction of full output (e.g., {@code 0.001})
         */
        public void configNeutralDeadband(double deadband) {
            talonConfig.MotorOutput.DutyCycleNeutralDeadband = deadband;
        }

        /**
         * Sets the peak forward and reverse duty-cycle output limits.
         *
         * @param forward maximum forward output (0 to 1)
         * @param reverse maximum reverse output (-1 to 0)
         */
        public void configPeakOutput(double forward, double reverse) {
            talonConfig.MotorOutput.PeakForwardDutyCycle = forward;
            talonConfig.MotorOutput.PeakReverseDutyCycle = reverse;
        }

        /**
         * Configures the forward software limit switch.
         *
         * @param threshold the position threshold in rotations
         * @param enabled {@code true} to enable the limit
         */
        public void configForwardSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        }

        /**
         * Configures the reverse software limit switch.
         *
         * @param threshold the position threshold in rotations
         * @param enabled {@code true} to enable the limit
         */
        public void configReverseSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        }

        /**
         * Enables or disables continuous position wrap-around for closed-loop control. Useful for
         * mechanisms that rotate continuously (e.g., swerve azimuth).
         *
         * @param enabled {@code true} to enable continuous wrap
         */
        public void configContinuousWrap(boolean enabled) {
            talonConfig.ClosedLoopGeneral.ContinuousWrap = enabled;
        }

        /**
         * Configures optional Motion Magic velocity parameters (acceleration and feed-forward) for
         * both FOC and voltage velocity control requests.
         *
         * @param acceleration the velocity acceleration in rotations per second squared
         * @param feedforward the feed-forward term applied during velocity control
         */
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC =
                    mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage =
                    mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        /**
         * Configures the feed-forward term for Motion Magic position control requests.
         *
         * @param feedforward the feed-forward term to apply during position control
         */
        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
        }

        /**
         * Configures the Motion Magic cruise velocity, acceleration, and jerk limits.
         *
         * @param cruiseVelocity maximum cruise velocity in rotations per second
         * @param acceleration maximum acceleration in rotations per second squared
         * @param jerk maximum jerk in rotations per second cubed
         */
        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

        /**
         * Configures the sensor-to-mechanism gear ratio. This is the ratio of rotor rotations to
         * one full mechanism output rotation (or sensor rotations if a remote sensor is used).
         *
         * @param gearRatio the gear ratio (e.g., {@code 11.25} means 11.25 rotor turns per output
         *     rotation)
         */
        public void configGearRatio(double gearRatio) {
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        /**
         * Returns the currently configured sensor-to-mechanism gear ratio.
         *
         * @return the gear ratio
         */
        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        /**
         * Sets the motor neutral mode to brake or coast.
         *
         * @param isInBrake {@code true} for brake mode; {@code false} for coast mode
         */
        public void configNeutralBrakeMode(boolean isInBrake) {
            if (isInBrake) {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            } else {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            }
        }

        /**
         * Configures PID gains in slot 0.
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void configPIDGains(double kP, double kI, double kD) {
            configPIDGains(0, kP, kI, kD);
        }

        /**
         * Configures PID gains in the specified slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void configPIDGains(int slot, double kP, double kI, double kD) {
            talonConfigFeedbackPID(slot, kP, kI, kD);
        }

        /**
         * Configures feed-forward gains in slot 0.
         *
         * @param kS static friction compensation (volts or amps)
         * @param kV velocity feed-forward gain
         * @param kA acceleration feed-forward gain
         * @param kG gravity/load compensation gain
         */
        public void configFeedForwardGains(double kS, double kV, double kA, double kG) {
            configFeedForwardGains(0, kS, kV, kA, kG);
        }

        /**
         * Configures feed-forward gains in the specified slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kS static friction compensation (volts or amps)
         * @param kV velocity feed-forward gain
         * @param kA acceleration feed-forward gain
         * @param kG gravity/load compensation gain
         */
        public void configFeedForwardGains(int slot, double kS, double kV, double kA, double kG) {
            talonConfigFeedForward(slot, kV, kA, kS, kG);
        }

        /**
         * Configures the feedback sensor source using a rotorOffset of {@code 0}.
         *
         * @param source the feedback sensor source (e.g., remote CANcoder)
         */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        /**
         * Configures the feedback sensor source and its rotational offset.
         *
         * @param source the feedback sensor source
         * @param offset the feedback rotor offset in rotations
         */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }

        /**
         * Configures the gravity compensation type in slot 0.
         *
         * @param isArm {@code true} for {@link GravityTypeValue#Arm_Cosine} (rotating arm); {@code
         *     false} for {@link GravityTypeValue#Elevator_Static} (elevator)
         */
        public void configGravityType(boolean isArm) {
            configGravityType(0, isArm);
        }

        /**
         * Configures the gravity compensation type in the specified slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param isArm {@code true} for {@link GravityTypeValue#Arm_Cosine} (rotating arm); {@code
         *     false} for {@link GravityTypeValue#Elevator_Static} (elevator)
         */
        public void configGravityType(int slot, boolean isArm) {
            GravityTypeValue gravityType =
                    isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
            if (slot == 0) {
                talonConfig.Slot0.GravityType = gravityType;
            } else if (slot == 1) {
                talonConfig.Slot1.GravityType = gravityType;
            } else if (slot == 2) {
                talonConfig.Slot2.GravityType = gravityType;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid slot", false);
            }
        }

        /**
         * Sets the minimum and maximum rotation limits for the mechanism. These bounds are used by
         * unit-conversion helpers such as {@link Mechanism#percentToRotations}.
         *
         * @param minRotation the minimum position in rotations
         * @param maxRotation the maximum position in rotations
         */
        protected void configMinMaxRotations(double minRotation, double maxRotation) {
            this.minRotations = minRotation;
            this.maxRotations = maxRotation;
        }

        // ── Private Helpers ───────────────────────────────────────────────────

        /**
         * Applies feed-forward gains (kV, kA, kS, kG) to the specified TalonFX slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kV velocity feed-forward
         * @param kA acceleration feed-forward
         * @param kS static friction compensation
         * @param kG gravity compensation
         */
        private void talonConfigFeedForward(int slot, double kV, double kA, double kS, double kG) {
            if (slot == 0) {
                talonConfig.Slot0.kV = kV;
                talonConfig.Slot0.kA = kA;
                talonConfig.Slot0.kS = kS;
                talonConfig.Slot0.kG = kG;
            } else if (slot == 1) {
                talonConfig.Slot1.kV = kV;
                talonConfig.Slot1.kA = kA;
                talonConfig.Slot1.kS = kS;
                talonConfig.Slot1.kG = kG;
            } else if (slot == 2) {
                talonConfig.Slot2.kV = kV;
                talonConfig.Slot2.kA = kA;
                talonConfig.Slot2.kS = kS;
                talonConfig.Slot2.kG = kG;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid FeedForward slot", false);
            }
        }

        /**
         * Applies PID gains (kP, kI, kD) to the specified TalonFX slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        private void talonConfigFeedbackPID(int slot, double kP, double kI, double kD) {
            if (slot == 0) {
                talonConfig.Slot0.kP = kP;
                talonConfig.Slot0.kI = kI;
                talonConfig.Slot0.kD = kD;
            } else if (slot == 1) {
                talonConfig.Slot1.kP = kP;
                talonConfig.Slot1.kI = kI;
                talonConfig.Slot1.kD = kD;
            } else if (slot == 2) {
                talonConfig.Slot2.kP = kP;
                talonConfig.Slot2.kI = kI;
                talonConfig.Slot2.kD = kD;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid Feedback slot", false);
            }
        }
    }
}
