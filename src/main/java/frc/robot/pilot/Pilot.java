package frc.robot.pilot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.telemetry.Telemetry;

/**
 * The pilot drives and does every intake and score. The operator only picks the level and branch.
 *
 * <pre>
 *   LT hold   ground coral (side picked from the drive direction)
 *   LB hold   station coral (side picked from which end faces the station)
 *   RT hold   auto-align to the nearest reef face and score at the operator's pick
 *   RB hold   stage the operator's pick; release to score (manual)
 *   A  hold   pull algae from the nearest reef face (high or low is automatic)
 *   X  hold   ground algae
 *   B  hold   stage net; release to throw
 *   Y  hold   stage processor; release to score
 *   Select    home everything
 *   D-pad     reorient: the robot front now faces away / left / toward / right
 *   Disabled: A coast, B brake, Select reset pose from vision
 * </pre>
 */
public class Pilot extends Gamepad {
    public final Trigger groundCoral_LT = leftTrigger.and(teleop);
    public final Trigger stationCoral_LB = leftBumper.and(teleop);
    public final Trigger autoScore_RT = rightTrigger.and(teleop);
    public final Trigger stageCoral_RB = rightBumper.and(teleop);
    public final Trigger reefAlgae_A = A.and(teleop);
    public final Trigger groundAlgae_X = X.and(teleop);
    public final Trigger net_B = B.and(teleop);
    public final Trigger processor_Y = Y.and(teleop);
    public final Trigger home_select = select.and(teleop);

    public final Trigger upReorient = upDpad.and(teleop);
    public final Trigger leftReorient = leftDpad.and(teleop);
    public final Trigger downReorient = downDpad.and(teleop);
    public final Trigger rightReorient = rightDpad.and(teleop);

    public final Trigger coast_A = A.and(disabled);
    public final Trigger brake_B = B.and(disabled);
    public final Trigger visionPoseReset_select = select.and(disabled);

    public static class PilotConfig extends Config {
        public PilotConfig() {
            super("Pilot", 0);
            setLeftStickDeadzone(0.1);
            setLeftStickExp(3.0);
            setRightStickDeadzone(0.1);
            setRightStickExp(3.0);
            setRightStickScalar(3 * Math.PI);
            setTriggersDeadzone(0.1);
            setTriggersExp(1);
            setTriggersScalar(1);
        }
    }

    public Pilot(PilotConfig config) {
        super(config);
        config.setLeftStickScalar(
                Robot.getConfig().swerve.getLinearSpeedAt12Volts().in(MetersPerSecond));
        leftStickCurve.setScalar(config.getLeftStickScalar());
        rightStickCurve.setScalar(config.getRightStickScalar());
        setDefaultCommand(
                Commands.runOnce(() -> rumbleController(0, 0), this)
                        .andThen(Commands.idle(this))
                        .withName("Pilot.noRumble"));
        Telemetry.print("Pilot Subsystem Initialized");
    }

    /** Forward, driver's view, m/s. */
    public double getDriveFwdPositive() {
        return leftStickCurve.calculate(-getLeftY());
    }

    /** Left, driver's view, m/s. */
    public double getDriveLeftPositive() {
        return -leftStickCurve.calculate(getLeftX());
    }

    /** Counter-clockwise, rad/s. */
    public double getDriveCCWPositive() {
        return -rightStickCurve.calculate(getRightX());
    }
}
