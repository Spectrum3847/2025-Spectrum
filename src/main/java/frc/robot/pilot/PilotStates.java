package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

/** This class should have any command calls that directly call the Pilot */
public class PilotStates {
    private static Pilot pilot = Robot.getPilot();

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        pilot.setDefaultCommand(log(rumble(0, 1).withName("Pilot.noRumble")));
    }

    /** Set the states for the pilot controller */
    public static void setStates() {
        // Rumble whenever we reorient
        pilot.upReorient
                .or(pilot.downReorient, pilot.leftReorient, pilot.rightReorient)
                .onTrue(log(rumble(1, 0.5).withName("Pilot.reorientRumble")));
    }

    /** Command that can be used to rumble the pilot controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return pilot.rumbleCommand(intensity, durationSeconds).withName("Pilot.rumble");
    }

    /**
     * Command that can be used to turn on the slow mode. Slow mode modifies the fwd, left, and CCW
     * methods, we don't want these to require the pilot subsystem arm
     */
    public static Command slowMode() {
        return Commands.startEnd(() -> pilot.getSlowMode().setTrue(), () -> pilot.getSlowMode().setFalse())
                .withName("Pilot.setSlowMode");
    }

    /**
     * Command that can be used to turn on the turbo mode. Turbo mode modifies CCW methods, we don't
     * want these to require the pilot subsystem
     */
    public static Command turboMode() {
        return Commands.startEnd(() -> pilot.getTurboMode().setTrue(), () -> pilot.getTurboMode().setFalse())
                .withName("Pilot.setTurboMode");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
