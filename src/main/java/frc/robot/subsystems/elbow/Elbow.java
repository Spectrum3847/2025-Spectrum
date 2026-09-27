package frc.robot.subsystems.elbow;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * The joint at the end of the shoulder arm. Angles are degrees in (-180, 180]; 180 is the stowed
 * position, negative angles are toward the front, and the rear side is the same angle negated.
 */
public class Elbow extends Mechanism {

    public static class ElbowConfig extends Config {
        /** Motor degrees = elbow degrees (wrapped to 0..360) + offset. */
        @Getter private final double offset = -90.0;

        @Getter private final double initPosition = 180.0;
        @Getter private final double tolerance = 3.5;

        @Getter private final double currentLimit = 60;
        @Getter private final double torqueCurrentLimit = 80;
        @Getter private final double positionKp = 1400;
        @Getter private final double positionKd = 160;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.7;
        @Getter private final double positionKa = 0.002;
        @Getter private final double positionKg = 22;
        @Getter private final double mmCruiseVelocity = 1;
        @Getter private final double mmAcceleration = 20;
        @Getter private final double mmJerk = 100;
        @Getter private final double slowMmAcceleration = 5;
        @Getter private final double slowMmJerk = 60;
        /** Gentler profile for dropping onto the carpet to ground intake. */
        @Getter private final double groundMmAcceleration = 3;

        @Getter private final double groundMmJerk = 60;

        @Getter private final double sensorToMechanismRatio = 61.71428571;

        public ElbowConfig() {
            super("Elbow", 43, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-torqueCurrentLimit);
            configMinMaxRotations(0, 0.6);
            configReverseSoftLimit(0, true);
            configForwardSoftLimit(0.6, true);
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
        /** The ground intake profile, so the claw does not slam into the carpet. */
        GROUND_POSITION,
        OFF
    }

    @Getter private WantedState wantedState = WantedState.OFF;
    @Getter private double targetDegrees = 180;
    @Getter private final ElbowConfig config;
    @Getter private ArmSim sim;

    public Elbow(ElbowConfig config) {
        super(config);
        this.config = config;
        if (isAttached()) {
            setMotorPosition(() -> toMotorRotations(config.initPosition));
            ArmConfig simConfig =
                    new ArmConfig(
                                    0.8,
                                    0.8,
                                    config.sensorToMechanismRatio,
                                    0.4,
                                    90.0 - 360.0,
                                    360.0 - 90.0,
                                    90)
                            .setColor(new Color8Bit(Color.kAqua))
                            .setMount(Robot.getShoulder().getSim(), true);
            // Sized so the sim's gravity torque matches what kG holds on the real forearm.
            simConfig.setSimMOI(1.0);
            simConfig.setSimCGLength(0.6);
            sim = new ArmSim(simConfig, RobotSim.leftView, motor, "3Elbow");
        }
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    private double toMotorRotations(double degrees) {
        double wrapped = degrees < 0 ? degrees + 360 : degrees;
        return degreesToRotations(() -> wrapped + config.offset);
    }

    public void setWantedState(WantedState state, double degrees) {
        wantedState = state;
        targetDegrees = degrees;
    }

    /** Current angle in elbow degrees, in (-180, 180]. */
    public double getDegrees() {
        double deg = getPositionDegrees() - config.offset;
        return deg > 180 ? deg - 360 : deg;
    }

    public boolean isAt(double degrees) {
        // Compare on the circle: 180 and -180 are the same place.
        double error = Math.IEEEremainder(getDegrees() - degrees, 360);
        return Math.abs(error) < config.tolerance;
    }

    public boolean isAtTarget() {
        return isAt(targetDegrees);
    }

    @Override
    public void periodic() {
        double motorRotations = toMotorRotations(targetDegrees);
        switch (wantedState) {
            case OFF -> stop();
            case POSITION -> setMMPositionFoc(() -> motorRotations);
            case SLOW_POSITION -> setDynMMPositionFoc(
                    () -> motorRotations,
                    config::getMmCruiseVelocity,
                    config::getSlowMmAcceleration,
                    config::getSlowMmJerk);
            case GROUND_POSITION -> setDynMMPositionFoc(
                    () -> motorRotations,
                    config::getMmCruiseVelocity,
                    config::getGroundMmAcceleration,
                    config::getGroundMmJerk);
        }
        logBatteryUsage();
        Telemetry.log("Elbow/WantedState", wantedState.toString());
        Telemetry.log("Elbow/TargetDegrees", targetDegrees, "deg");
        Telemetry.logDash("Elbow/Degrees", getDegrees(), "deg");
        Telemetry.log("Elbow/AtTarget", isAtTarget());
        logDiagnostics("Elbow");
    }
}
