package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.CurrentSuperState;

/**
 * The 60-LED strip on PWM 0, driven straight from the superstructure each loop: what the robot
 * holds, whether it is aligning, and when the score gate opens.
 */
public class Leds {
    private static final Color PURPLE = new Color(130, 0, 255);
    // Built once: a new pattern every loop is garbage the roboRIO has to collect.
    private static final LEDPattern DISABLED = LEDPattern.solid(PURPLE).breathe(Seconds.of(2));
    private static final LEDPattern SCORING = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.1));
    private static final LEDPattern ALIGNING = LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.2));
    private static final LEDPattern CORAL = LEDPattern.solid(Color.kWhite);
    private static final LEDPattern ALGAE = LEDPattern.solid(Color.kTeal);
    private static final LEDPattern EMPTY = LEDPattern.solid(PURPLE);

    private final AddressableLED led = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);
    private final SuperStructure superStructure;

    public Leds(SuperStructure superStructure) {
        this.superStructure = superStructure;
        led.setLength(buffer.getLength());
        led.start();
    }

    public void periodic() {
        pattern().applyTo(buffer);
        led.setData(buffer);
    }

    private LEDPattern pattern() {
        if (DriverStation.isDisabled()) {
            return DISABLED;
        }
        CurrentSuperState state = superStructure.getCurrentSuperState();
        if (state == CurrentSuperState.SCORING_CORAL || superStructure.isScoreGateOpen()) {
            return SCORING;
        }
        if (state == CurrentSuperState.ALIGN_CORAL || state == CurrentSuperState.REEF_ALGAE) {
            return ALIGNING;
        }
        return switch (superStructure.getHeldPiece()) {
            case CORAL -> CORAL;
            case ALGAE -> ALGAE;
            case NONE -> EMPTY;
        };
    }
}
