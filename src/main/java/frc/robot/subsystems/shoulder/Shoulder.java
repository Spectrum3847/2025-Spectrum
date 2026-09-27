package frc.robot.subsystems.shoulder;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
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
 * The arm joint on the elevator carriage. Angles are degrees with 0 pointing straight down and
 * positive swinging toward the robot front; 180 is straight up. The rear side is the same angle
 * negated.
 */
public class Shoulder extends Mechanism {

    public static class ShoulderConfig extends Config {
        /** Motor degrees = shoulder degrees + offset. */
        @Getter private final double offset = -90;

        @Getter private final double initPosition = 0;
        @Getter private final double tolerance = 3.5;

        @Getter private final double currentLimit = 60;
        @Getter private final double torqueCurrentLimit = 80;
        @Getter private final double positionKp = 1500;
        @Getter private final double positionKd = 170;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.06;
        @Getter private final double positionKa = 0.001;
        @Getter private final double positionKg = 20.83333;
        @Getter private final double mmCruiseVelocity = 10;
        @Getter private final double mmAcceleration = 50;
        @Getter private final double mmJerk = 0;
        @Getter private final double slowMmAcceleration = 2;
        @Getter private final double slowMmJerk = 50;

        @Getter private final double sensorToMechanismRatio = 61.71428571;

        @Getter @Setter private boolean CANcoderAttached = false;
        @Getter @Setter private double CANcoderOffset = 0;
        @Getter private final double CANcoderRotorToSensorRatio = 61.71428571 * 1.2;
        @Getter private final double CANcoderSensorToMechanismRatio = 1 / 1.2;

        public ShoulderConfig() {
            super("Shoulder", 42, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-torqueCurrentLimit);
            configMinMaxRotations(-1, 0.5);
            configReverseSoftLimit(-1, true);
            configForwardSoftLimit(0.5, true);
            configNeutralBrakeMode(true);
            if (RobotBase.isSimulation()) {
                configCounterClockwise_Positive();
            } else {
                configClockwise_Positive();
            }
            configGravityType(true);
        }
    }

    public enum WantedState {
        POSITION,
        SLOW_POSITION,
        OFF
    }

    @Getter private WantedState wantedState = WantedState.OFF;
    @Getter private double targetDegrees = 0;
    @Getter private final ShoulderConfig config;
    @Getter private ArmSim sim;

    public Shoulder(ShoulderConfig config) {
        super(config);
        this.config = config;
        if (isAttached()) {
            SpectrumCANcoder canCoder = null;
            if (config.isCANcoderAttached() && RobotBase.isReal()) {
                canCoder =
                        new SpectrumCANcoder(
                                42,
                                new SpectrumCANcoderConfig(
                                        config.CANcoderRotorToSensorRatio,
                                        config.CANcoderSensorToMechanismRatio,
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
                double absolute =
                        canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                                / config.CANcoderSensorToMechanismRatio;
                setMotorPosition(() -> absolute);
            } else {
                setMotorPosition(
                        () -> degreesToRotations(() -> config.initPosition + config.offset));
            }
            sim =
                    new ArmSim(
                            new ArmConfig(
                                            0.8,
                                            1.1,
                                            config.sensorToMechanismRatio,
                                            0.3,
                                            -360,
                                            270,
                                            -90)
                                    .setMount(Robot.getElevator().getSim(), true),
                            RobotSim.leftView,
                            motor,
                            "2Shoulder");
        }
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    public void setWantedState(WantedState state, double degrees) {
        wantedState = state;
        targetDegrees = degrees;
    }

    /** Current angle in shoulder degrees (0 down, positive toward the front). */
    public double getDegrees() {
        return getPositionDegrees() - config.offset;
    }

    public boolean isAt(double degrees) {
        return Math.abs(getDegrees() - degrees) < config.tolerance;
    }

    public boolean isAtTarget() {
        return isAt(overTop(targetDegrees));
    }

    /**
     * Keeps a big swing going over the top instead of through the elevator: a target past -90 while
     * the arm is past +90 is the same place one turn up, and the other way round.
     */
    private double overTop(double degrees) {
        double current = getDegrees();
        if (degrees < -90 && current > 90) {
            return degrees + 360;
        } else if (degrees > 90 && current < -90) {
            return degrees - 360;
        }
        return degrees;
    }

    @Override
    public void periodic() {
        double motorRotations = degreesToRotations(() -> overTop(targetDegrees) + config.offset);
        switch (wantedState) {
            case OFF -> stop();
            case POSITION -> setMMPositionFoc(() -> motorRotations);
            case SLOW_POSITION -> setDynMMPositionFoc(
                    () -> motorRotations,
                    config::getMmCruiseVelocity,
                    config::getSlowMmAcceleration,
                    config::getSlowMmJerk);
        }
        logBatteryUsage();
        Telemetry.log("Shoulder/WantedState", wantedState.toString());
        Telemetry.log("Shoulder/TargetDegrees", targetDegrees, "deg");
        Telemetry.logDash("Shoulder/Degrees", getDegrees(), "deg");
        Telemetry.log("Shoulder/AtTarget", isAtTarget());
        logDiagnostics("Shoulder");
    }
}
