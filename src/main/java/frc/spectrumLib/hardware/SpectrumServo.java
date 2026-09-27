package frc.spectrumLib.hardware;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A WPILib {@link Servo} that also implements {@link Subsystem}, allowing it to be registered with
 * the command-based framework and participate in requirement checking.
 */
public class SpectrumServo extends Servo implements Subsystem {

    /**
     * Creates a SpectrumServo connected to the given PWM port on the RoboRIO.
     *
     * @param port PWM channel (0–9) the servo signal wire is plugged into
     */
    public SpectrumServo(int port) {
        super(port);
        CommandScheduler.getInstance().registerSubsystem(this);
    }
}
