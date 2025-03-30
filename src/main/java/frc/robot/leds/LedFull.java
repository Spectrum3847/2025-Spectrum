package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import frc.spectrumLib.leds.SpectrumLEDs;
import lombok.Getter;

public class LedFull extends SpectrumLEDs {

    public static class LedFullConfig extends Config {
        public LedFullConfig() {
            super("LEDS", 30 * 2);
            setLedSpacing(Meters.of(1 / 120.0));
        }
    }

    protected LedFullConfig config;
    @Getter protected LedRight right;
    @Getter protected LedLeft left;

    public LedFull(LedFullConfig config) {
        super(config);
        this.config = config;

        right = new LedRight(new LedRight.LedConfig(getLed(), getLedBuffer()));
        left = new LedLeft(new LedLeft.LedConfig(getLed(), getLedBuffer()));
    }

    /**
     * Binds the triggers for the LED commands. This method overrides the bindTriggers method to
     * ensure that the LED commands are properly bound to their respective triggers.
     */
    @Override
    public void setupStates() {
        LedStates.bindTriggers();
    }

    /**
     * Sets up the default command for the LED subsystem. This method is called to assign the
     * default command that will run when no other commands are scheduled for the subsystem.
     */
    @Override
    public void setupDefaultCommand() {
        setDefaultCommand(defaultCommand);
    }
}
