package frc.robot.subsystems.twist;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.hardware.SpectrumCANcoder;
import frc.spectrumLib.hardware.SpectrumCANcoderConfig;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import lombok.Setter;

/**
 * Rolls the claw about the arm. It can turn past a full revolution, so a target is an angle on the
 * circle (0..360) and this class picks which way round to go: the claw must never swing through the
 * elevator, and at the reef it must not swing through the branch it is scoring on.
 *
 * <p>The route logic is carried over unchanged from the 2025 code, which was tuned on the robot.
 * The one fix is that the direction is chosen once when the target changes, instead of being
 * re-derived from wherever the twist has got to, which could flip it mid-move.
 */
public class Twist extends Mechanism {

    public static class TwistConfig extends Config {
        @Getter private final double initPosition = 0;
        /** Within this of the target the twist goes straight there, whichever way that is. */
        @Getter private final double routeTolerance = 45;

        @Getter private final double tolerance = 5;

        @Getter private final double currentLimit = 10;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double positionKp = 1000;
        @Getter private final double positionKd = 70;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 1.8;
        @Getter private final double positionKa = 2;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 4.2;
        @Getter private final double mmAcceleration = 32;
        @Getter private final double mmJerk = 0;

        @Getter private final double sensorToMechanismRatio = 22.4;

        @Getter @Setter private boolean CANcoderAttached = false;
        @Getter @Setter private double CANcoderOffset = 0;

        public TwistConfig() {
            super("Twist", 44, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-.25, 0.75);
            configReverseSoftLimit(-.25, false);
            configForwardSoftLimit(0.75, false);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(true);
            configClockwise_Positive();
        }
    }

    public enum WantedState {
        /** Route around the elevator. */
        POSITION,
        /** Route around the reef branch: used once the claw is at the reef. */
        POSITION_AT_REEF,
        OFF
    }

    @Getter private WantedState wantedState = WantedState.OFF;
    /** Target on the circle, degrees in [0, 360). */
    @Getter private double targetDegrees = 90;

    private boolean rear = false;
    private double routedTarget = Double.NaN;
    private WantedState routedState = WantedState.OFF;
    private boolean clockwise = false;

    @Getter private final TwistConfig config;

    public Twist(TwistConfig config) {
        super(config);
        this.config = config;
        if (isAttached()) {
            SpectrumCANcoder canCoder = null;
            if (config.isCANcoderAttached() && RobotBase.isReal()) {
                canCoder =
                        new SpectrumCANcoder(
                                44,
                                new SpectrumCANcoderConfig(
                                        config.sensorToMechanismRatio,
                                        1,
                                        config.CANcoderOffset,
                                        true,
                                        false),
                                motor,
                                config,
                                SpectrumCANcoder.CANCoderFeedbackType.FusedCANcoder);
            }
            if (canCoder != null
                    && canCoder.canCoderResponseOK(
                            canCoder.getCanCoder().getAbsolutePosition().getStatus())) {
                double absolute = canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble();
                setMotorPosition(() -> absolute);
            } else {
                setMotorPosition(() -> degreesToRotations(config::getInitPosition));
            }
            // No gravity about the arm's own axis; the limits only need to allow a few turns.
            ArmConfig simConfig =
                    new ArmConfig(0.3, 0.3, config.sensorToMechanismRatio, 0.15, -1080, 1080, 0)
                            .setSimulatedGravity(false);
            simConfig.setSimMOI(0.02);
            new ArmSim(simConfig, RobotSim.leftView, motor, "4Twist");
        }
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    /**
     * @param state how to route
     * @param degrees target on the circle
     * @param rear true when the arm is working the rear side, which mirrors the route rules
     */
    public void setWantedState(WantedState state, double degrees, boolean rear) {
        wantedState = state;
        targetDegrees = mod360(degrees);
        this.rear = rear;
    }

    public boolean isAtTarget() {
        double error = Math.IEEEremainder(getPositionDegrees() - targetDegrees, 360);
        return Math.abs(error) < config.tolerance;
    }

    @Override
    public void periodic() {
        if (wantedState == WantedState.OFF) {
            stop();
            routedTarget = Double.NaN;
        } else {
            double current = getPositionDegrees();
            if (targetDegrees != routedTarget || wantedState != routedState) {
                routedTarget = targetDegrees;
                routedState = wantedState;
                clockwise =
                        wantedState == WantedState.POSITION_AT_REEF
                                ? clockwiseAwayFromBranch(targetDegrees, current, rear)
                                : clockwiseAwayFromElevator(targetDegrees, current, rear);
            }
            double output = route(targetDegrees, current, clockwise, config.routeTolerance);
            setMMPositionFoc(() -> degreesToRotations(() -> output));
        }
        logBatteryUsage();
        Telemetry.log("Twist/WantedState", wantedState.toString());
        Telemetry.log("Twist/TargetDegrees", targetDegrees, "deg");
        Telemetry.log("Twist/Clockwise", clockwise);
        Telemetry.logDash("Twist/Degrees", getPositionDegrees(), "deg");
        Telemetry.log("Twist/AtTarget", isAtTarget());
        logDiagnostics("Twist");
    }

    public static double mod360(double degrees) {
        double m = degrees % 360;
        return m < 0 ? m + 360 : m;
    }

    /**
     * Continuous motor-degree setpoint that reaches {@code target} on the circle from {@code
     * current}: straight there when within {@code tolerance}, else the long way round in the given
     * direction.
     */
    static double route(double target, double current, boolean clockwise, double tolerance) {
        double currentMod = mod360(current);
        double diff = currentMod - target;
        if (Math.abs(diff) < tolerance) {
            return current - diff;
        }
        if (Math.abs(diff) > 360 - tolerance) {
            // Either side of the 0/360 seam.
            return diff > 0 ? current + (360 - diff) : current - (360 + diff);
        }
        if (clockwise) {
            return current - (diff > 0 ? diff : 360 + diff);
        }
        return current + (diff < 0 ? -diff : 360 - diff);
    }

    /** Direction that keeps the claw from swinging through the elevator (180..360 side). */
    static boolean clockwiseAwayFromElevator(double target, double current, boolean rear) {
        double t = 45; // matches the route tolerance
        double degreesMod = mod360(target);
        double currentMod = mod360(current);
        boolean targetPastElevator = degreesMod > 180 + t && degreesMod < 360 - t;
        boolean currentPastElevator = currentMod > 180 + t && currentMod < 360 - t;

        if (targetPastElevator && currentMod < 180 + t) {
            return !rear;
        }
        if (degreesMod < 180 + t && currentPastElevator) {
            return !rear;
        }
        if (targetPastElevator && currentPastElevator) {
            return degreesMod >= currentMod;
        }
        return degreesMod < currentMod && currentMod < 360 - t;
    }

    /** Direction that keeps the claw from swinging through the branch it is scoring on. */
    static boolean clockwiseAwayFromBranch(double target, double current, boolean rear) {
        double degreesMod = mod360(target);
        double currentMod = mod360(current);
        if (degreesMod < 180 && currentMod > 180) {
            return !rear;
        }
        if (degreesMod > 180 && currentMod < 180) {
            return rear;
        }
        if (degreesMod > 180 && currentMod > 180) {
            return degreesMod >= currentMod;
        }
        return degreesMod < currentMod;
    }

    /** The target for the same claw orientation with the arm on the rear side. */
    public static double mirrorForRear(double degrees) {
        return degrees + 180 > 270 ? degrees - 180 : degrees + 180;
    }
}
