package frc.spectrumLib;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SpectrumServo extends Servo implements Subsystem {

    public SpectrumServo(int port) {
        super(port);
    }
}
