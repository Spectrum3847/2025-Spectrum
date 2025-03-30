package frc.spectrumLib.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.SpectrumRobot;
import frc.spectrumLib.SpectrumSubsystem;
import java.util.Map;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class SpectrumLEDs implements SpectrumSubsystem {

    // Example Animation List - https://github.com/Aircoookie/WLED/wiki/List-of-effects-and-palettes

    public static class Config {
        @Getter private String name;
        @Getter @Setter private boolean attached = true;
        @Getter @Setter private AddressableLED led;
        @Getter @Setter private AddressableLEDBuffer buffer;
        @Getter @Setter private AddressableLEDBufferView view;
        @Getter @Setter private int startingIndex = 0;
        @Getter @Setter private int endingIndex = 0;
        @Getter @Setter private int port = 0;
        @Getter @Setter private int length;
        // LED strip density
        @Getter @Setter private Distance ledSpacing = Meters.of(1 / 120.0);

        public Config(String name, int length) {
            this.name = name;
            this.length = length;
            this.startingIndex = 0;
            this.endingIndex = length - 1;
        }

        public Config(
                String name,
                AddressableLED l,
                AddressableLEDBuffer lb,
                int startingIndex,
                int endingIndex) {
            this.name = name;
            this.led = l;
            this.buffer = lb;
            this.startingIndex = startingIndex;
            this.endingIndex = endingIndex;
        }
    }

    @Getter private Config config;

    @Getter protected final AddressableLED led;
    @Getter protected final AddressableLEDBuffer ledBuffer;
    @Getter protected final AddressableLEDBufferView ledView;
    private boolean mainView = false;

    protected final LEDPattern defaultPattern = blink(Color.kOrange, 1);

    @Getter
    protected Command defaultCommand =
            setPattern(defaultPattern, -1).withName("LEDs.defaultCommand");

    public final Trigger defaultTrigger = new Trigger(() -> defaultCommand.isScheduled());

    @Getter @Setter private int commandPriority = 0;

    public final Color purple = new Color(130, 103, 185);
    public final Color white = Color.kWhite;

    public SpectrumLEDs(Config config) {
        this.config = config;

        // Must be a PWM header, not MXP or DIO
        if (config.getLed() == null) {
            led = new AddressableLED(config.port);
            // Length is expensive to set, so only set it once, then just update data
            ledBuffer = new AddressableLEDBuffer(config.length);
            led.setLength(ledBuffer.getLength());
            mainView = true;
        } else {
            led = config.getLed();
            ledBuffer = config.buffer;
        }

        ledView = ledBuffer.createView(config.startingIndex, config.endingIndex);

        // Set the data
        led.setData(ledBuffer);
        setPattern(defaultPattern);
        led.start();

        SpectrumRobot.add(this);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        // Set the LEDs only if this is the main view
        if (mainView) {
            led.setData(ledBuffer);
        }
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    public Trigger checkPriority(int priority) {
        return new Trigger(() -> commandPriority <= priority);
    }

    public Command setPattern(LEDPattern pattern, int priority) {
        return run(() -> {
                    commandPriority = priority;
                    pattern.applyTo(ledView);
                })
                .ignoringDisable(true)
                .withName("LEDs.setPattern");
    }

    public Command setPattern(LEDPattern pattern) {
        return setPattern(pattern, 0);
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        setDefaultCommand(
                setPattern(solid(Color.kOrange), 0)
                        .withName("SPECTRUM LED DEFAULT COMMAND SHOULD NOT BE RUNNING"));
    }

    /**
     * LED Pattern Stripe, takes in a double percent and sets the first length number of LEDs to one
     * color and the rest of the strip to another
     */
    public LEDPattern stripe(double percent, Color color1, Color color2) {
        return LEDPattern.steps(Map.of(0.00, color1, percent, color2));
    }

    /**
     * Creates a solid LED pattern with the specified color.
     *
     * @param color the color to be used for the solid LED pattern
     * @return an LEDPattern object representing the solid color pattern
     */
    public LEDPattern solid(Color color) {
        return LEDPattern.solid(color);
    }

    /**
     * Creates an LED pattern that blinks with the specified on-time duration.
     *
     * @param onTime the duration (in seconds) for which the LED stays on during each blink cycle
     * @return an LEDPattern that blinks with the specified on-time duration
     */
    public LEDPattern blink(Color c, double onTime) {
        return solid(c).blink(Seconds.of(onTime));
    }

    /**
     * Creates a breathing LED pattern with the specified period.
     *
     * @param period The period of the breathing effect in seconds.
     * @return An LEDPattern object representing the breathing effect.
     */
    public LEDPattern breathe(Color c, double period) {
        return solid(c).breathe(Seconds.of(period));
    }

    /**
     * Creates and returns a rainbow LED pattern with specified brightness and saturation.
     *
     * @return an LEDPattern object representing a rainbow pattern with maximum brightness (255) and
     *     medium saturation (128).
     */
    public LEDPattern rainbow() {
        return rainbow(255, 128);
    }

    public LEDPattern scrollingRainbow() {
        return rainbow().scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), config.getLedSpacing());
    }

    /**
     * Generates a rainbow LED pattern with the specified saturation and value.
     *
     * @param saturation the saturation level of the rainbow pattern (0-255)
     * @param value the brightness value of the rainbow pattern (0-255)
     * @return an LEDPattern object representing the rainbow pattern
     */
    public LEDPattern rainbow(int saturation, int value) {
        return LEDPattern.rainbow(saturation, value);
    }

    /**
     * Creates a gradient LED pattern using the specified colors.
     *
     * @param colors The array of colors to be used in the gradient pattern.
     * @return An LEDPattern object representing the gradient pattern.
     */
    public LEDPattern gradient(Color... colors) {
        return LEDPattern.gradient(GradientType.kContinuous, colors);
    }

    /**
     * Scrolls the given LED pattern at the specified speed.
     *
     * @param pattern the LEDPattern to be scrolled
     * @param speedMps the speed at which the pattern should scroll, in meters per second
     * @return a new LEDPattern that represents the scrolled pattern
     */
    public LEDPattern scroll(LEDPattern pattern, double speedMps) {
        return pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(speedMps), config.getLedSpacing());
    }

    /**
     * Creates an LED chase pattern with the specified color, percentage, and speed.
     *
     * @param color1 The color to be used in the chase pattern.
     * @param percent The percentage of the pattern that will be the specified color.
     * @param speed The speed at which the pattern will scroll, in Hertz.
     * @return An LEDPattern object representing the chase pattern.
     */
    public LEDPattern chase(Color color1, double percent, double speed) {
        return LEDPattern.steps(Map.of(0.00, color1, percent, Color.kBlack))
                .scrollAtRelativeSpeed(Frequency.ofBaseUnits(speed, Hertz));
    }

    /**
     * Creates a bouncing LED pattern with the specified color and duration.
     *
     * @param c the color of the bouncing LED
     * @param durationInSeconds the duration of one complete bounce cycle in seconds
     * @return an LEDPattern that applies the bouncing effect to the LEDs
     */
    public LEDPattern bounce(Color c, double durationInSeconds) {
        return new LEDPattern() {
            @Override
            public void applyTo(LEDReader reader, LEDWriter writer) {
                int bufLen = reader.getLength();
                long currentTime = System.currentTimeMillis();
                double cycleTime =
                        durationInSeconds
                                * 1000; // Convert time to milliseconds for the entire cycle
                double phase =
                        (currentTime % cycleTime) / cycleTime; // Phase of the cycle from 0 to 1

                // Determine direction and position based on the phase
                boolean backwards = phase > 0.5;
                double position = backwards ? 2 * (1 - phase) : 2 * phase;
                int ledPosition = (int) (position * bufLen);

                for (int tempLed = 0; tempLed < bufLen; tempLed++) {
                    if (tempLed == ledPosition) {
                        writer.setLED(tempLed, c);
                    } else if (tempLed == ledPosition - 1 || tempLed == ledPosition + 1) {
                        writer.setLED(
                                tempLed, new Color(c.red * 0.66, c.green * 0.66, c.blue * 0.66));
                    } else if (tempLed == ledPosition - 2 || tempLed == ledPosition + 2) {
                        writer.setLED(
                                tempLed, new Color(c.red * 0.33, c.green * 0.33, c.blue * 0.33));
                    } else {
                        writer.setLED(tempLed, Color.kBlack);
                    }
                }
            }
        };
    }

    /**
     * Creates an ombre LED pattern that transitions smoothly between two colors.
     *
     * @param startColor The starting color of the ombre effect.
     * @param endColor The ending color of the ombre effect.
     * @return An LEDPattern that applies the ombre effect to the LED strip.
     */
    public LEDPattern ombre(Color startColor, Color endColor) {
        return new LEDPattern() {
            @Override
            public void applyTo(LEDReader reader, LEDWriter writer) {
                int bufLen = reader.getLength();
                long currentTime = System.currentTimeMillis();
                // The speed factor here determines how quickly the ombre moves along the strip
                double phaseShift =
                        (currentTime / 1000.0)
                                * 0.58 // this is the speed, the higher the number the faster the
                                // ombre moves
                                % 1.0; // Modulo 1 to keep the phase within [0, 1]
                for (int tempLed = 0; tempLed < bufLen; tempLed++) {
                    // Adjust ratio to include the phaseShift, causing the ombre to move
                    double ratio =
                            ((tempLed + (bufLen * phaseShift))
                                    / bufLen
                                    % 1.0); // Modulo 1 to ensure the ratio loops within [0,
                    // 1]

                    // Interpolate the red, green, and blue components separately using double
                    // precision
                    double red = (startColor.red * (1 - ratio)) + (endColor.red * ratio);
                    double green = (startColor.green * (1 - ratio)) + (endColor.green * ratio);
                    double blue = (startColor.blue * (1 - ratio)) + (endColor.blue * ratio);

                    // Create a new color for the current LED
                    Color currentColor = new Color(red, green, blue);

                    // Set the color of the current LED
                    writer.setLED(tempLed, currentColor);
                }
            }
        };
    }

    /**
     * Creates a wave LED pattern that transitions between two colors over a specified cycle length
     * of LEDs and duration.
     *
     * @param c1 The first color in the wave pattern.
     * @param c2 The second color in the wave pattern.
     * @param cycleLength The length of the wave cycle in terms of LEDs.
     * @param durationSecs The duration of the entire wave pattern in seconds.
     * @return An LEDPattern that applies the wave effect to the LEDs.
     */
    public LEDPattern wave(Color c1, Color c2, double cycleLength, double durationSecs) {
        return new LEDPattern() {
            @Override
            public void applyTo(LEDReader reader, LEDWriter writer) {
                int bufLen = reader.getLength();
                double currentTime = Timer.getFPGATimestamp();
                double phase = (currentTime % durationSecs) / durationSecs;
                double x = (1 - phase) * 2.0 * Math.PI;
                double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
                double waveExponent = 0.4;

                for (int tempLed = 0; tempLed < bufLen; tempLed++) {
                    x += xDiffPerLed;

                    double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
                    if (Double.isNaN(ratio)) {
                        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
                    }
                    if (Double.isNaN(ratio)) {
                        ratio = 0.5;
                    }
                    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
                    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
                    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
                    writer.setLED(tempLed, new Color(red, green, blue));
                }
            }
        };
    }

    /**
     * Creates an LEDPattern that represents a countdown effect. The countdown starts from a
     * specified time and lasts for a given duration. During the countdown, the LEDs transition from
     * yellow to red, and progressively turn off from the end of the strip towards the beginning.
     *
     * @param countStartTimeSec A DoubleSupplier that provides the start time of the countdown in
     *     seconds.
     * @param durationInSeconds The total duration of the countdown in seconds.
     * @return An LEDPattern that applies the countdown effect to the LEDs.
     */
    public LEDPattern countdown(DoubleSupplier countStartTimeSec, double durationInSeconds) {
        double startTime = countStartTimeSec.getAsDouble();
        return new LEDPattern() {
            @Override
            public void applyTo(LEDReader reader, LEDWriter writer) {
                int bufLen = reader.getLength();
                double currentTimeSecs = Timer.getFPGATimestamp();
                double elapsedTimeInSeconds = currentTimeSecs - startTime;

                // Calculate the progress of the countdown
                double progress = elapsedTimeInSeconds / durationInSeconds;

                // Calculate the number of LEDs to turn off based on the progress
                int ledsToTurnOff = (int) (bufLen * progress);

                // Calculate the color transition from yellow to red based on the progress
                // Yellow (255, 255, 0) to Red (255, 0, 0)
                int red = 255; // Red component stays at 255
                int green = (int) (255 * (1 - progress)); // Green component decreases to 0
                Color countdownColor = new Color(red, green, 0);

                // Update the LEDs from the end of the strip towards the beginning
                for (int tempLed = bufLen - 1; tempLed >= 0; tempLed--) {
                    if (bufLen - tempLed <= ledsToTurnOff) {
                        // Turn off the LEDs progressively
                        writer.setLED(tempLed, Color.kBlack);
                    } else {
                        // Set the remaining LEDs to the countdown color
                        writer.setLED(tempLed, countdownColor);
                    }
                }

                // If the countdown is complete, ensure all LEDs are turned off
                if (progress >= 1.0) {
                    for (int i = 0; i < bufLen; i++) {
                        writer.setLED(i, Color.kBlack);
                    }
                }
            }
        };
    }

    public LEDPattern edges(Color c, double length) {
        return new LEDPattern() {
            public void applyTo(LEDReader reader, LEDWriter writer) {
                int bufLen = reader.getLength();
                for (int i = 0; i < bufLen; i++) {
                    if (i < length || i > bufLen - length - 1) {
                        writer.setLED(i, c);
                    } else {
                        writer.setLED(i, Color.kBlack);
                    }
                }
            }
        };
    }

    // LEDPattern Methods
    // reversed()
    // offsetBy(int offset)
    // scrollAtAbsoluteSpeed(Distance speed, Distance spacing)
    // scrollAtRelativeSpeed(Frequency velocity)
    // blink(Time onTime, Time offTime)
    // synchronizedBlink(BooleanSupplier signal)
    // breathe(Time period)
    // overlayOn(LEDPattern base)
    // blend(LEDPattern other)
    // mask(LEDPattern mask)
    // atBrightness(Dimensionless relativeBrightness)
    // progressMaskLayer(DoubleSupplier progressSupplier)
    // steps(Map<? extends Number, Color> steps)
}
