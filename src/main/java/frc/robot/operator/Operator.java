package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.telemetry.Telemetry;

/**
 * The operator picks what the pilot scores and runs the climb.
 *
 * <pre>
 *   A / B / X / Y       select L1 / L2 / L3 / L4 (latched, shown on the dashboard)
 *   D-pad left/right    select the left / right branch (latched)
 *   LT hold             eject whatever is in the claw
 *   Start hold          climb prep; while held, RT winds the climb in and LT lets it out
 *   Select              home everything
 *   Disabled: A coast, B brake
 * </pre>
 */
public class Operator extends Gamepad {
    private static final double CLIMB_UP_SCALE = 0.35;
    private static final double CLIMB_DOWN_SCALE = 0.15;

    public final Trigger climbHeld = start.and(teleop);
    public final Trigger notClimbing = climbHeld.negate();

    public final Trigger l1_A = A.and(teleop).and(notClimbing);
    public final Trigger l2_B = B.and(teleop).and(notClimbing);
    public final Trigger l3_X = X.and(teleop).and(notClimbing);
    public final Trigger l4_Y = Y.and(teleop).and(notClimbing);
    public final Trigger leftBranch = leftDpad.and(teleop);
    public final Trigger rightBranch = rightDpad.and(teleop);
    public final Trigger eject_LT = leftTrigger.and(teleop).and(notClimbing);
    public final Trigger climbDrive = climbHeld.and(leftTrigger.or(rightTrigger));
    public final Trigger home_select = select.and(teleop);

    public final Trigger coast_A = A.and(disabled);
    public final Trigger brake_B = B.and(disabled);

    public static class OperatorConfig extends Config {
        public OperatorConfig() {
            super("Operator", 1);
            setTriggersDeadzone(0.0);
        }
    }

    public Operator(OperatorConfig config) {
        super(config);
        Telemetry.print("Operator Subsystem Initialized");
    }

    /** Climb output: RT winds in, LT lets out. */
    public double getClimbPercent() {
        return getRightTriggerAxis() * CLIMB_UP_SCALE - getLeftTriggerAxis() * CLIMB_DOWN_SCALE;
    }
}
