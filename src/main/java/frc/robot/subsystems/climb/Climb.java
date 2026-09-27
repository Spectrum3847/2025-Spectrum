package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.Timer;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.hardware.SpectrumServo;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.function.DoubleSupplier;
import lombok.Getter;

/** Deep-cage climber: two motors on one arm, plus a servo latch. */
public class Climb extends Mechanism {

    public static class ClimbConfig extends Config {
        @Getter private final double maxRotations = 0.335;
        @Getter private final double minRotations = -0.06;
        /** Stowed, in degrees (0 is horizontal). */
        @Getter private final double home = 90;

        @Getter private final double latchOpen = 1;
        @Getter private final double latchClosed = 0;
        /** Time the arm gets to swing out before the latch closes on the cage. */
        @Getter private final double prepSeconds = 1.5;

        @Getter private final double currentLimit = 60;
        @Getter private final double torqueCurrentLimit = 180;
        @Getter private final double positionKp = 190;
        @Getter private final double positionKd = 40;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.3;
        @Getter private final double positionKa = 0.001;
        @Getter private final double positionKg = 2.9;
        @Getter private final double mmCruiseVelocity = 1;
        @Getter private final double mmAcceleration = 10;
        @Getter private final double mmJerk = 0;

        public ClimbConfig() {
            super("Climb", 55, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(99.5555555555);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-torqueCurrentLimit);
            configMinMaxRotations(minRotations, maxRotations);
            configReverseSoftLimit(minRotations, true);
            configForwardSoftLimit(maxRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configGravityType(true);
            setFollowerConfigs(
                    new FollowerConfig(
                            "ClimbBottom", 56, Rio.CANIVORE, MotorAlignmentValue.Aligned));
        }
    }

    public enum WantedState {
        /** Stowed with the latch open. */
        HOME,
        /** Swung out to the cage; the latch closes once it has had time to get there. */
        PREP,
        /** Driven by the operator's triggers, latch closed. */
        MANUAL,
        OFF
    }

    @Getter private WantedState wantedState = WantedState.OFF;
    private DoubleSupplier manualPercent = () -> 0;
    private double stateStartSeconds = 0;
    private final SpectrumServo latch = new SpectrumServo(9);
    @Getter private final ClimbConfig config;

    public Climb(ClimbConfig config) {
        super(config);
        this.config = config;
        if (isAttached()) {
            setMotorPosition(() -> 0.25);
        }
        latch.set(config.latchOpen);
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    public void setWantedState(WantedState state) {
        if (state != wantedState) {
            stateStartSeconds = Timer.getFPGATimestamp();
        }
        wantedState = state;
    }

    /** The operator axis {@link WantedState#MANUAL} follows; bound once at startup. */
    public void setManualInput(DoubleSupplier percent) {
        manualPercent = percent;
    }

    @Override
    public void periodic() {
        switch (wantedState) {
            case OFF -> stop();
            case HOME -> {
                setMMPositionFoc(() -> degreesToRotations(config::getHome));
                latch.set(config.latchOpen);
            }
            case PREP -> {
                setMMPositionFoc(config::getMinRotations);
                boolean arrived = Timer.getFPGATimestamp() - stateStartSeconds > config.prepSeconds;
                latch.set(arrived ? config.latchClosed : config.latchOpen);
            }
            case MANUAL -> {
                setPercentOutput(manualPercent);
                latch.set(config.latchClosed);
            }
        }
        logBatteryUsage();
        Telemetry.log("Climb/WantedState", wantedState.toString());
        Telemetry.log("Climb/Rotations", getPositionRotations());
        logDiagnostics("Climb");
    }
}
