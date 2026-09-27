package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/** The claw rollers. Every state is a torque current; positive pulls coral in. */
public class Intake extends Mechanism {

    public static class IntakeConfig extends Config {
        /** Stalled roller: slower than this with more current than {@link #hasPieceCurrent}. */
        @Getter private final double hasPieceVelocityRPM = 50;

        @Getter private final double hasPieceCurrent = 80;

        @Getter private final double currentLimit = 44;
        @Getter private final double torqueCurrentLimit = 200;

        public IntakeConfig() {
            super("Intake", 5, Rio.CANIVORE);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }
    }

    /** Torque currents carried over from the 2025 code. */
    public enum WantedState {
        OFF(0),
        HOLD_CORAL(28),
        HOLD_ALGAE(-85),
        STATION_CORAL(100),
        GROUND_CORAL(200),
        INTAKE_ALGAE(-85),
        SCORE_CORAL(-25),
        SCORE_L1(-30),
        SCORE_NET(180),
        SCORE_PROCESSOR(100),
        EJECT(-30);

        final double amps;

        WantedState(double amps) {
            this.amps = amps;
        }

        boolean intaking() {
            return this == STATION_CORAL || this == GROUND_CORAL || this == INTAKE_ALGAE;
        }
    }

    @Getter private WantedState wantedState = WantedState.OFF;
    private double stateStartSeconds = 0;
    @Getter private final IntakeConfig config;

    public Intake(IntakeConfig config) {
        super(config);
        this.config = config;
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    public void setWantedState(WantedState state) {
        if (state != wantedState) {
            stateStartSeconds = Timer.getFPGATimestamp();
        }
        wantedState = state;
    }

    /**
     * True while a piece is stalling the rollers during an intake state. In simulation there is no
     * stall, so a piece "arrives" half a second into any intake.
     */
    public boolean hasPiece() {
        if (!wantedState.intaking()) {
            return false;
        }
        if (RobotBase.isSimulation()) {
            return Timer.getFPGATimestamp() - stateStartSeconds > 0.5;
        }
        return Math.abs(getVelocityRPM()) < config.hasPieceVelocityRPM
                && Math.abs(getStatorCurrent()) > config.hasPieceCurrent;
    }

    @Override
    public void periodic() {
        if (wantedState == WantedState.OFF) {
            stop();
        } else {
            setTorqueCurrentFoc(() -> wantedState.amps);
        }
        logBatteryUsage();
        Telemetry.log("Intake/WantedState", wantedState.toString());
        Telemetry.logDash("Intake/HasPiece", hasPiece());
        Telemetry.log("Intake/VelocityRPM", getVelocityRPM(), "RPM");
        logDiagnostics("Intake");
    }
}
