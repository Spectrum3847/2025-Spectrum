package frc.spectrumLib.leds;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.hardware.CanConfigBudget;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

/**
 * CANdle-based addressable LED subsystem that wraps a CTRE {@link CANdle} and exposes a rich
 * library of pattern factories (solid, stripe, blink, breathe, rainbow, chase, bounce, gradient,
 * ombre, wave, countdown, etc.).
 *
 * <p>Patterns are split into two categories:
 *
 * <ul>
 *   <li><b>Hardware animation patterns</b> ({@link #blink}, {@link #breathe}, {@link #rainbow},
 *       {@link #scrollingRainbow}, {@link #chase}, {@link #bounce}, {@link #fire}, {@link
 *       #rgbCycle}) — these use {@code setControl()} with CANdle's built-in animation engine. They
 *       run autonomously in firmware and require no per-loop CPU work.
 *   <li><b>Software patterns</b> ({@link #solid}, {@link #stripe}, {@link #gradient}, {@link
 *       #ombre}, {@link #wave}, {@link #countdown}, {@link #switchCountdown}, {@link #edges}) —
 *       these use {@code setControl(SolidColor)} which is a one-shot command resent each loop. When
 *       switching from a hardware animation to a software pattern, all animations are automatically
 *       cleared.
 * </ul>
 *
 * <p>Multiple {@code SpectrumLEDs} instances can share a single physical {@link CANdle} device by
 * passing the same {@link CANdle} reference in their {@link Config} objects and selecting
 * non-overlapping {@code startIdx}/{@code numLeds} ranges.
 *
 * <p>Patterns are applied via {@link #setPattern(CANdlePattern, int)}, which returns a {@link
 * Command} that runs continuously and respects the priority system ({@link #checkPriority(int)}).
 */
public class SpectrumLEDs implements Subsystem {

    // -------------------------------------------------------------------------
    // CANdlePattern functional interface
    // -------------------------------------------------------------------------

    /**
     * Functional interface for a LED pattern that drives a segment of a {@link CANdle} strip.
     *
     * <p>Implementations receive the live {@link CANdle} device, the first LED index in this
     * instance's segment ({@code startIdx}), and the number of LEDs in the segment ({@code
     * numLeds}). Hardware animation patterns call {@link CANdle#setControl}, software patterns call
     * {@link CANdle#setControl(SolidColor)} (one-shot, resent each loop).
     */
    @FunctionalInterface
    public interface CANdlePattern {
        /**
         * Apply this pattern to the given LED segment.
         *
         * @param candle the {@link CANdle} device to write to
         * @param startIdx the first LED index (inclusive); {@code 0} includes the 8 onboard LEDs
         * @param numLeds the number of LEDs in this segment
         */
        void applyTo(CANdle candle, int startIdx, int numLeds);
    }

    /**
     * Internal marker wrapper — returned by animation factory methods so that {@link
     * #setPattern(CANdlePattern, int)} can detect when a transition from animation to software
     * pattern occurs and clear the animation slots.
     */
    private static final class HardwareAnimPattern implements CANdlePattern {
        private final CANdlePattern impl;
        /**
         * Creates a new HardwareAnimPattern instance.
         *
         * @param impl the impl
         */
        HardwareAnimPattern(CANdlePattern impl) {
            this.impl = impl;
        }
        /** Apply to. */
        @Override
        public void applyTo(CANdle candle, int startIdx, int numLeds) {
            impl.applyTo(candle, startIdx, numLeds);
        }
    }

    /** Wraps a pattern lambda in a {@link HardwareAnimPattern} marker. */
    private static CANdlePattern hardwareAnim(CANdlePattern p) {
        return new HardwareAnimPattern(p);
    }

    // -------------------------------------------------------------------------
    // Config
    // -------------------------------------------------------------------------

    /**
     * Configuration for a {@link SpectrumLEDs} subsystem instance.
     *
     * <p>Use {@link #Config(String, int, int, CANBus)} to create a standalone instance that owns
     * and configures its {@link CANdle}, or {@link #Config(String, CANdle, int, int)} to share an
     * already-configured device across multiple subsystems targeting different LED segments.
     */
    public static class Config {
        /** Human-readable name used in telemetry. */
        @Getter private String name;

        /** Whether this LED strip is physically connected to the robot. */
        @Getter @Setter private boolean attached = true;

        /**
         * Pre-built {@link CANdle} to reuse. When non-null, {@link #deviceId} and {@link #canBus}
         * are ignored and no hardware configuration is applied by this instance.
         */
        @Getter @Setter private CANdle sharedCandle = null;

        /** CAN device ID used when no {@link #sharedCandle} is provided. */
        @Getter @Setter private int deviceId = 1;

        /** CAN bus used when no {@link #sharedCandle} is provided. */
        @Getter @Setter private CANBus canBus;

        /**
         * First LED index (inclusive) in the strip. Use {@code 0} to include the 8 onboard status
         * LEDs on the CANdle board itself; use {@code 8} to skip them.
         */
        @Getter @Setter private int startIdx = 0;

        /** Number of LEDs in the segment owned by this instance. */
        @Getter @Setter private int numLeds;

        /**
         * CANdle hardware animation slot (0–7) used by this instance's animation patterns. Each
         * {@link SpectrumLEDs} instance sharing a single {@link CANdle} must use a distinct slot,
         * otherwise their animations overwrite each other.
         */
        @Getter @Setter private int animationSlot = 0;

        /** LED strip type (RGB, RGBW, GRB, etc.). Ignored when {@link #sharedCandle} is set. */
        @Getter @Setter private StripTypeValue stripType = StripTypeValue.RGB;

        /**
         * Overall brightness scalar applied in hardware (0.0–1.0). Ignored when {@link
         * #sharedCandle} is set.
         */
        @Getter @Setter private double brightness = 1.0;

        /**
         * Behavior of the strip when CAN signal is lost. Ignored when {@link #sharedCandle} is set.
         */
        @Getter @Setter
        private LossOfSignalBehaviorValue lossOfSignalBehavior =
                LossOfSignalBehaviorValue.DisableLEDs;

        /**
         * Creates a configuration for a standalone LED instance that creates and owns its own
         * {@link CANdle}. Hardware configuration (strip type, brightness, loss-of-signal) is
         * applied automatically in the constructor.
         *
         * @param name human-readable name for telemetry
         * @param deviceId CAN device ID of the CANdle
         * @param numLeds number of LEDs on the external strip (not counting the 8 onboard LEDs)
         * @param canBus CAN bus the CANdle is on
         */
        public Config(String name, int deviceId, int numLeds, CANBus canBus) {
            this.name = name;
            this.deviceId = deviceId;
            this.numLeds = numLeds;
            this.canBus = canBus;
            this.startIdx = 8;
        }

        /**
         * Creates a configuration for a LED zone that shares an existing, already-configured {@link
         * CANdle} device. No hardware configuration is applied.
         *
         * @param name human-readable name for telemetry
         * @param sharedCandle the {@link CANdle} to reuse
         * @param startIdx first LED index (inclusive) in the shared strip for this zone
         * @param numLeds number of LEDs in this zone
         */
        public Config(String name, CANdle sharedCandle, int startIdx, int numLeds) {
            this.name = name;
            this.sharedCandle = sharedCandle;
            this.startIdx = startIdx;
            this.numLeds = numLeds;
        }
    }

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Active configuration for this instance. */
    @Getter private Config config;

    /** The {@link CANdle} device (owned or shared). */
    @Getter protected final CANdle candle;

    /** {@code true} if the most recently applied pattern was a hardware animation. */
    private boolean lastWasAnimation = false;

    /**
     * Default pattern shown when no other command requires this subsystem (orange blink).
     *
     * <p>Initialized in the constructor body (after {@link #config} is set) so that pattern
     * factories can safely reference the config.
     */
    protected final CANdlePattern defaultPattern;

    /**
     * The default command built by this class (displays {@link #defaultPattern} at lowest
     * priority). Installed via {@code setDefaultCommand} in the constructor; subclasses may install
     * their own default command to replace it. Use {@code getDefaultCommand()} (from {@link
     * Subsystem}) to query whichever default is currently installed.
     */
    protected final Command defaultCommand;

    /**
     * Trigger that is active while the currently installed default command (whichever one that is)
     * is the one running — i.e. no higher-priority pattern owns the subsystem.
     */
    public final Trigger defaultTrigger;

    /**
     * Priority level of the pattern command currently running. Higher values indicate higher
     * priority; {@link #setPattern(CANdlePattern, int)} stores this while a command runs and resets
     * it to {@code -1} when the command ends.
     */
    @Getter @Setter private int commandPriority = -1;

    /** Spectrum purple color constant ({@code RGB 130, 103, 185}). */
    public final Color purple = new Color(130, 103, 185);

    /** Convenience alias for {@link Color#kWhite}. */
    public final Color white = Color.kWhite;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Constructs the LED subsystem, configures the hardware (or reuses a shared device), and
     * registers with the WPILib {@link CommandScheduler}.
     *
     * @param config the configuration describing the device, segment range, and strip type
     */
    public SpectrumLEDs(Config config) {
        this.config = config;

        if (config.getSharedCandle() != null) {
            candle = config.getSharedCandle();
        } else {
            candle = new CANdle(config.getDeviceId(), config.getCanBus());
            CANdleConfiguration candleConfig =
                    new CANdleConfiguration()
                            .withLED(
                                    new LEDConfigs()
                                            .withStripType(config.getStripType())
                                            .withBrightnessScalar(config.getBrightness())
                                            .withLossOfSignalBehavior(
                                                    config.getLossOfSignalBehavior()));
            CanConfigBudget.run(
                    "CANdle " + config.getDeviceId(),
                    timeout -> candle.getConfigurator().apply(candleConfig, timeout));
        }

        // Pattern fields are initialized here (after config is set) so factory methods
        // can safely read config values.
        defaultPattern = blink(Color.kOrange, 1.0);
        defaultCommand = setPattern(defaultPattern, -1).withName("LEDs.defaultCommand");
        setDefaultCommand(defaultCommand);
        defaultTrigger =
                new Trigger(
                        () -> {
                            Command current = getCurrentCommand();
                            return current != null && current == getDefaultCommand();
                        });

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    // -------------------------------------------------------------------------
    // Subsystem API
    // -------------------------------------------------------------------------

    /**
     * Returns whether this LED strip is physically connected to the robot.
     *
     * @return {@code true} if attached
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * Returns {@code true} if the most recently applied pattern was a CANdle hardware animation
     * running in firmware, {@code false} if it was a software {@link SolidColor} pattern.
     */
    public boolean isAnimating() {
        return lastWasAnimation;
    }

    /**
     * Returns the currently running command that is applying a pattern to this subsystem, or {@code
     * null} if no command is currently running.
     *
     * @return the currently running command, or {@code null} if none
     */
    public String getCurrentCommandName() {
        Command cmd = getCurrentCommand();
        return cmd != null ? cmd.getName() : "None";
    }

    /**
     * Returns a {@link Trigger} that is active when the currently running command's priority is at
     * or below the given value. Use to gate lower-priority commands from overriding higher-priority
     * ones.
     *
     * @param priority the maximum priority level that allows the trigger to be active
     * @return trigger active when {@link #commandPriority} &le; priority
     */
    public Trigger checkPriority(int priority) {
        return new Trigger(() -> commandPriority <= priority);
    }

    /**
     * Returns a command that continuously applies {@code pattern} to the LED segment and records
     * the given {@code priority} while running. The command runs while the robot is disabled.
     *
     * <p>When switching from a hardware animation to a software ({@link SolidColor}) pattern, all
     * active animation slots are cleared automatically before the first software write.
     *
     * @param pattern the {@link CANdlePattern} to apply each loop cycle
     * @param priority priority level stored in {@link #commandPriority} while this command runs
     * @return a command that applies the pattern continuously
     */
    public Command setPattern(CANdlePattern pattern, int priority) {
        return run(() -> {
                    commandPriority = priority;
                    boolean isAnim = pattern instanceof HardwareAnimPattern;
                    // Clear this instance's animation slot once when transitioning to a software
                    // pattern. Only our own slot is cleared so other instances sharing the same
                    // CANdle keep their animations running.
                    if (lastWasAnimation && !isAnim) {
                        candle.setControl(new EmptyAnimation(config.getAnimationSlot()));
                    }
                    lastWasAnimation = isAnim;
                    pattern.applyTo(candle, config.getStartIdx(), config.getNumLeds());
                })
                .finallyDo(() -> commandPriority = -1)
                .ignoringDisable(true)
                .withName("LEDs.setPattern");
    }

    /**
     * Returns a command that continuously applies {@code pattern} to the LED segment at priority 0.
     *
     * @param pattern the {@link CANdlePattern} to apply
     * @return a command that applies the pattern continuously at the default priority
     */
    public Command setPattern(CANdlePattern pattern) {
        return setPattern(pattern, 0);
    }

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    /**
     * Converts a WPILib {@link Color} (0.0–1.0 double components) to a {@link RGBWColor} with
     * {@code W = 0}.
     */
    private static RGBWColor toRGBW(Color color) {
        return new RGBWColor(
                (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0);
    }

    // -------------------------------------------------------------------------
    // Hardware animation pattern factories
    // -------------------------------------------------------------------------

    /**
     * Blinking (strobe) pattern — alternates between {@code color} and off. Each half-cycle (on and
     * off) lasts {@code onTimeSecs} seconds.
     *
     * <p>Implemented using {@link StrobeAnimation}. Frame rate = {@code 1 / onTimeSecs} Hz.
     *
     * @param color the blink color
     * @param onTimeSecs duration in seconds of each on (and off) half-cycle
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern blink(Color color, double onTimeSecs) {
        RGBWColor rgbw = toRGBW(color);
        // Lazy: animation created on first applyTo call using the runtime startIdx/numLeds.
        StrobeAnimation[] holder = new StrobeAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new StrobeAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withFrameRate(Hertz.of(1.0 / onTimeSecs));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * Breathing (sinusoidal fade-in/out) pattern — fades between the peak {@code color} and off.
     *
     * <p>Implemented using {@link SingleFadeAnimation}. Each animation frame changes brightness by
     * 1%, so frame rate = {@code 200 / periodSecs} Hz for a complete 0→100→0% cycle.
     *
     * @param color the peak color at full brightness
     * @param periodSecs duration in seconds of one full breathe cycle
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern breathe(Color color, double periodSecs) {
        RGBWColor rgbw = toRGBW(color);
        SingleFadeAnimation[] holder = new SingleFadeAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new SingleFadeAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withFrameRate(Hertz.of(200.0 / periodSecs));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * Static rainbow — hue distributed evenly across the strip, advancing very slowly.
     *
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern rainbow() {
        return rainbow(1.0);
    }

    /**
     * Static rainbow with configurable brightness.
     *
     * @param brightness brightness scalar (0.0–1.0)
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern rainbow(double brightness) {
        RainbowAnimation[] holder = new RainbowAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new RainbowAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withBrightness(brightness)
                                        .withFrameRate(Hertz.of(3));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * Scrolling rainbow that advances quickly across the strip.
     *
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern scrollingRainbow() {
        RainbowAnimation[] holder = new RainbowAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new RainbowAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withBrightness(1.0)
                                        .withFrameRate(Hertz.of(60));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * Chase / color-flow pattern — progressively lights LEDs one at a time across the strip and
     * repeats.
     *
     * <p>Implemented using {@link ColorFlowAnimation}. Frame rate = {@code numLeds × speed} Hz so
     * that {@code speed} full cycles occur per second.
     *
     * @param color the chase color
     * @param speed desired number of full strip cycles per second
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern chase(Color color, double speed) {
        RGBWColor rgbw = toRGBW(color);
        ColorFlowAnimation[] holder = new ColorFlowAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new ColorFlowAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withFrameRate(Hertz.of(numLeds * speed));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * Bouncing dot pattern — a pocket of light travels back and forth across the strip.
     *
     * <p>Implemented using {@link LarsonAnimation} with {@link LarsonBounceValue#Back}. Frame rate
     * is computed so one back-and-forth cycle takes {@code durationSecs} seconds.
     *
     * @param color the dot color
     * @param durationSecs seconds per complete back-and-forth cycle
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern bounce(Color color, double durationSecs) {
        RGBWColor rgbw = toRGBW(color);
        LarsonAnimation[] holder = new LarsonAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        // One full cycle = 2 * (numLeds - 1) LED-position advances.
                        double frameRate = 2.0 * Math.max(numLeds - 1, 1) / durationSecs;
                        holder[0] =
                                new LarsonAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withColor(rgbw)
                                        .withSize(3)
                                        .withBounceMode(LarsonBounceValue.Back)
                                        .withFrameRate(Hertz.of(frameRate));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * Fire animation using the CANdle's built-in hardware animation engine.
     *
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern fire() {
        FireAnimation[] holder = new FireAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new FireAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withFrameRate(Hertz.of(60));
                    }
                    candle.setControl(holder[0]);
                });
    }

    /**
     * RGB color-cycle animation using the CANdle's built-in hardware animation engine.
     *
     * @return a hardware animation {@link CANdlePattern}
     */
    public CANdlePattern rgbCycle() {
        RgbFadeAnimation[] holder = new RgbFadeAnimation[1];
        return hardwareAnim(
                (candle, startIdx, numLeds) -> {
                    if (holder[0] == null) {
                        holder[0] =
                                new RgbFadeAnimation(startIdx, startIdx + numLeds - 1)
                                        .withSlot(config.getAnimationSlot())
                                        .withFrameRate(Hertz.of(30));
                    }
                    candle.setControl(holder[0]);
                });
    }

    // -------------------------------------------------------------------------
    // Software pattern factories (SolidColor one-shot, resent each loop)
    // -------------------------------------------------------------------------

    /**
     * Solid color pattern.
     *
     * <p>Uses a single {@link SolidColor} control request (one-shot, resent each loop).
     *
     * @param color the color to display
     * @return a software {@link CANdlePattern} showing a constant solid color
     */
    public CANdlePattern solid(Color color) {
        RGBWColor rgbw = toRGBW(color);
        SolidColor[] holder = new SolidColor[1];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor(startIdx, startIdx + numLeds - 1).withColor(rgbw);
            }
            candle.setControl(holder[0]);
        };
    }

    /**
     * Two-color stripe: the first {@code percent} fraction of LEDs shows {@code color1}, the
     * remainder shows {@code color2}.
     *
     * <p>Uses two {@link SolidColor} controls (one-shot each, resent each loop).
     *
     * @param percent fraction of the strip (0.0–1.0) assigned to {@code color1}
     * @param color1 color for the leading segment
     * @param color2 color for the trailing segment
     * @return a software {@link CANdlePattern} showing the two-color stripe
     */
    public CANdlePattern stripe(double percent, Color color1, Color color2) {
        RGBWColor rgbw1 = toRGBW(color1);
        RGBWColor rgbw2 = toRGBW(color2);
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                int split = Math.max(0, Math.min((int) Math.round(numLeds * percent), numLeds));
                holder[0] = new SolidColor[2];
                // Segment 1 (may be empty if split == 0)
                holder[0][0] =
                        (split > 0)
                                ? new SolidColor(startIdx, startIdx + split - 1).withColor(rgbw1)
                                : null;
                // Segment 2 (may be empty if split == numLeds)
                holder[0][1] =
                        (split < numLeds)
                                ? new SolidColor(startIdx + split, startIdx + numLeds - 1)
                                        .withColor(rgbw2)
                                : null;
            }
            for (SolidColor req : holder[0]) {
                if (req != null) candle.setControl(req);
            }
        };
    }

    /**
     * Linear gradient between two colors across the strip. Colors are pre-computed at first use.
     *
     * <p>Uses N {@link SolidColor} controls (one per LED, one-shot, resent each loop).
     *
     * @param color1 color at the start (index 0) of the segment
     * @param color2 color at the end of the segment
     * @return a software {@link CANdlePattern} showing the two-color gradient
     */
    public CANdlePattern gradient(Color color1, Color color2) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    double ratio = (numLeds <= 1) ? 0.0 : (double) i / (numLeds - 1);
                    int r = (int) (color1.red * 255 * (1 - ratio) + color2.red * 255 * ratio);
                    int g = (int) (color1.green * 255 * (1 - ratio) + color2.green * 255 * ratio);
                    int b = (int) (color1.blue * 255 * (1 - ratio) + color2.blue * 255 * ratio);
                    holder[0][i] =
                            new SolidColor(startIdx + i, startIdx + i)
                                    .withColor(new RGBWColor(r, g, b, 0));
                }
            }
            for (SolidColor req : holder[0]) {
                candle.setControl(req);
            }
        };
    }

    /**
     * Edge-highlight pattern — lights the first and last {@code length} LEDs with {@code color} and
     * turns off the center LEDs.
     *
     * <p>Uses two or three {@link SolidColor} controls (one-shot, resent each loop).
     *
     * @param color the color to apply to the edge LEDs
     * @param length the number of LEDs to illuminate at each end of the strip
     * @return a software {@link CANdlePattern} showing lit edges and a dark center
     */
    public CANdlePattern edges(Color color, int length) {
        RGBWColor rgbw = toRGBW(color);
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                int clampedLen = Math.min(length, numLeds / 2);
                if (clampedLen == 0) {
                    holder[0] = new SolidColor[0];
                } else {
                    int centerStart = startIdx + clampedLen;
                    int centerEnd = startIdx + numLeds - clampedLen - 1;
                    // left edge, right edge, (optional) center black
                    holder[0] = (centerStart <= centerEnd) ? new SolidColor[3] : new SolidColor[2];
                    holder[0][0] =
                            new SolidColor(startIdx, startIdx + clampedLen - 1).withColor(rgbw);
                    holder[0][1] =
                            new SolidColor(startIdx + numLeds - clampedLen, startIdx + numLeds - 1)
                                    .withColor(rgbw);
                    if (holder[0].length == 3) {
                        holder[0][2] =
                                new SolidColor(centerStart, centerEnd)
                                        .withColor(new RGBWColor(0, 0, 0, 0));
                    }
                }
            }
            for (SolidColor req : holder[0]) {
                candle.setControl(req);
            }
        };
    }

    /**
     * Animated ombre — transitions smoothly between two colors across the strip and scrolls the
     * blend point over time.
     *
     * <p>Uses N {@link SolidColor} controls (one per LED, one-shot, resent each loop with updated
     * colors). {@link RGBWColor} objects are created each loop since the type is immutable.
     *
     * @param startColor the leading color
     * @param endColor the trailing color
     * @return a software {@link CANdlePattern} showing the animated ombre
     */
    public CANdlePattern ombre(Color startColor, Color endColor) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }
            // Speed: 0.58 strip-lengths per second
            double phaseShift = (System.currentTimeMillis() / 1000.0) * 0.58 % 1.0;
            for (int i = 0; i < numLeds; i++) {
                double ratio = ((i + numLeds * phaseShift) / numLeds) % 1.0;
                int r = (int) (startColor.red * 255 * (1 - ratio) + endColor.red * 255 * ratio);
                int g = (int) (startColor.green * 255 * (1 - ratio) + endColor.green * 255 * ratio);
                int b = (int) (startColor.blue * 255 * (1 - ratio) + endColor.blue * 255 * ratio);
                holder[0][i].Color = new RGBWColor(r, g, b, 0);
                candle.setControl(holder[0][i]);
            }
        };
    }

    /**
     * Sinusoidal wave pattern blending between two colors.
     *
     * <p>Uses N {@link SolidColor} controls (one per LED, one-shot, resent each loop).
     *
     * @param c1 first wave color
     * @param c2 second wave color
     * @param cycleLength number of LEDs per wave period
     * @param durationSecs period of the time-based animation in seconds
     * @return a software {@link CANdlePattern} showing the wave
     */
    public CANdlePattern wave(Color c1, Color c2, double cycleLength, double durationSecs) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }
            double currentTime = Timer.getFPGATimestamp();
            double phase = (currentTime % durationSecs) / durationSecs;
            double x = (1 - phase) * 2.0 * Math.PI;
            double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
            double waveExponent = 0.4;
            for (int i = 0; i < numLeds; i++) {
                x += xDiffPerLed;
                double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
                if (Double.isNaN(ratio)) {
                    ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
                }
                if (Double.isNaN(ratio)) ratio = 0.5;
                int r = (int) (c1.red * 255 * (1 - ratio) + c2.red * 255 * ratio);
                int g = (int) (c1.green * 255 * (1 - ratio) + c2.green * 255 * ratio);
                int b = (int) (c1.blue * 255 * (1 - ratio) + c2.blue * 255 * ratio);
                holder[0][i].Color = new RGBWColor(r, g, b, 0);
                candle.setControl(holder[0][i]);
            }
        };
    }

    /**
     * Countdown pattern — LEDs transition from yellow to red and progressively turn off from the
     * end of the segment toward the beginning as time elapses.
     *
     * <p>Uses N {@link SolidColor} controls (one per LED, one-shot, resent each loop).
     *
     * @param countStartTimeSec supplies the FPGA timestamp (seconds) when the countdown began
     * @param durationInSeconds total countdown duration in seconds
     * @return a software {@link CANdlePattern} showing the countdown
     */
    public CANdlePattern countdown(DoubleSupplier countStartTimeSec, double durationInSeconds) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }
            // Read the supplier each loop (not at factory time) so patterns built at binding
            // time still measure from the correct start when the command eventually runs.
            double elapsed = Timer.getFPGATimestamp() - countStartTimeSec.getAsDouble();
            double progress = Math.min(elapsed / durationInSeconds, 1.0);
            int ledsOff = (int) (numLeds * progress);
            int green = (int) (255 * (1 - progress));
            for (int i = numLeds - 1; i >= 0; i--) {
                holder[0][i].Color =
                        (numLeds - i <= ledsOff)
                                ? new RGBWColor(0, 0, 0, 0)
                                : new RGBWColor(255, green, 0, 0);
                candle.setControl(holder[0][i]);
            }
        };
    }

    /**
     * Alliance switch countdown — cycles through alliance colors (and purple) on a hard-coded
     * match-time schedule, progressively turning off LEDs within each segment as time elapses.
     *
     * <p>Segment schedule (seconds remaining → color):
     *
     * <pre>
     *  140–130  purple
     *  130–105  startingColor
     *  105–80   opponent color
     *   80–55   startingColor
     *   55–30   opponent color
     *   30–0    purple
     * </pre>
     *
     * <p>Uses N {@link SolidColor} controls (one per LED, one-shot, resent each loop).
     *
     * @param startingColor the alliance color displayed during this robot's segments
     * @return a software {@link CANdlePattern} reflecting the current switch-countdown state
     */
    public CANdlePattern switchCountdown(Color startingColor) {
        SolidColor[][] holder = new SolidColor[1][];
        return (candle, startIdx, numLeds) -> {
            if (holder[0] == null) {
                holder[0] = new SolidColor[numLeds];
                for (int i = 0; i < numLeds; i++) {
                    holder[0][i] = new SolidColor(startIdx + i, startIdx + i);
                }
            }

            int[] times = {10, 25, 25, 25, 25, 30};
            double elapsed = 140 - Timer.getMatchTime();

            int shiftTime = 0;
            int cumulativeTime = 0;
            Color color = Color.kBlack;

            for (int i = 0; i < times.length; i++) {
                cumulativeTime += times[i];
                if (cumulativeTime > elapsed) {
                    shiftTime = times[i];
                    switch (i) {
                        case 0, 5 -> color = Color.kPurple;
                        case 1, 3 -> color = startingColor;
                        case 2, 4 -> color =
                                Color.kRed.equals(startingColor) ? Color.kBlue : Color.kRed;
                        default -> color = Color.kBlack;
                    }
                    break;
                }
            }

            double progress = 1.0 - (cumulativeTime - elapsed) / Math.max(shiftTime, 1);
            int ledsOff = (int) (numLeds * Math.min(progress, 1.0));
            RGBWColor segColor = toRGBW(color);

            for (int i = numLeds - 1; i >= 0; i--) {
                holder[0][i].Color =
                        (numLeds - i <= ledsOff) ? new RGBWColor(0, 0, 0, 0) : segColor;
                candle.setControl(holder[0][i]);
            }
        };
    }
}
