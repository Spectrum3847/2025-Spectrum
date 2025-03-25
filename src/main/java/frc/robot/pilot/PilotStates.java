package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.intake.IntakeStates;
import frc.robot.vision.VisionStates;
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
        pilot.actionReady_RB.whileTrue(slowMode());
        pilot.visionPoseReset_LB_Select.onTrue(VisionStates.resetVisionPose());
        // Rumble whenever we reorient
        pilot.upReorient
                .or(pilot.downReorient, pilot.leftReorient, pilot.rightReorient)
                .onTrue(log(rumble(1, 0.5).withName("Pilot.reorientRumble")));
        IntakeStates.hasCoral.onTrue(log(rumble(1, 0.5).withName("Pilot.hasCoralRumble")));
        IntakeStates.hasAlgae.onTrue(log(rumble(1, 0.5).withName("Pilot.hasAlgaeRumble")));
        RobotStates.staged.onTrue(log(rumble(1, 0.5).withName("Pilot.stagedRumble")));
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
        return Commands.startEnd(
                        () -> pilot.getSlowMode().setState(true),
                        () -> pilot.getSlowMode().setState(false))
                .withName("Pilot.setSlowMode");
    }

    /**
     * Command that can be used to turn on the turbo mode. Turbo mode modifies CCW methods, we don't
     * want these to require the pilot subsystem
     */
    public static Command turboMode() {
        return Commands.startEnd(
                        () -> pilot.getTurboMode().setState(true),
                        () -> pilot.getTurboMode().setState(false))
                .withName("Pilot.setTurboMode");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
