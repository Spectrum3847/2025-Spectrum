package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.spectrumLib.leds.SpectrumLEDs;

public class LedLeft extends SpectrumLEDs {

    public static class LedConfig extends Config {
        public LedConfig(AddressableLED l, AddressableLEDBuffer lb) {
            super("LEDS Left", l, lb, lb.getLength() / 2, lb.getLength() - 1);
            setLedSpacing(Meters.of(1 / 120.0));
        }
    }

    protected LedConfig config;

    public LedLeft(LedConfig config) {
        super(config);
        this.config = config;
    }

    /**
     * Binds the triggers for the LED commands. This method overrides the bindTriggers method to
     * ensure that the LED commands are properly bound to their respective triggers.
     */
    @Override
    public void setupStates() {
        // LedStates.bindTriggers();
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
