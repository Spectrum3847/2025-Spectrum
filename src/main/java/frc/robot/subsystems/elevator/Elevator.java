package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Two-motor elevator. Positions are motor rotations above the bottom hard stop; the setpoint table
 * lives in {@link frc.robot.subsystems.SuperStructure.Pose}, not here.
 */
public class Elevator extends Mechanism {

    public static class ElevatorConfig extends Config {
        @Getter private final double maxRotations = 21.1;
        @Getter private final double minRotations = 0.3;
        @Getter private final double fullExtend = maxRotations * .999;

        /** Rotations counted as "at target" by the superstructure's gates. */
        @Getter private final double tolerance = 1.15;

        /** Above this the carriage stops holding position against the hard stop. */
        @Getter private final double restRotations = 0.3;

        @Getter private final double positionKp = 100;
        @Getter private final double positionKd = 8;
        @Getter private final double positionKa = 0.2;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 5;
        @Getter private final double positionKg = 25.3;
        @Getter private final double mmCruiseVelocity = 70;
        @Getter private final double mmAcceleration = 400;
        @Getter private final double mmJerk = 4500;
        @Getter private final double slowMmAcceleration = 55;
        @Getter private final double slowMmJerk = 550;

        @Getter private final double currentLimit = 60;
        @Getter private final double torqueCurrentLimit = 160;

        /* Sim */
        @Getter private final double simGearing = 1.7;
        @Getter private final double simDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);

        public ElevatorConfig() {
            super("Elevator", 40, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "ElevatorRear", 41, Rio.CANIVORE, MotorAlignmentValue.Opposed));
        }
    }

    public enum WantedState {
        /** Motion Magic to {@link #setTarget}, at full speed. */
        POSITION,
        /** Motion Magic to {@link #setTarget}, with the gentle acceleration auto uses. */
        SLOW_POSITION,
        /** Output off; the carriage coasts down in brake mode. */
        OFF
    }

    @Getter private WantedState wantedState = WantedState.OFF;
    @Getter private double targetRotations = 0;
    @Getter private final ElevatorConfig config;
    @Getter private LinearSim sim;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;
        if (isAttached()) {
            setMotorPosition(() -> 0);
            sim =
                    new LinearSim(
                            new LinearConfig(
                                            0.8,
                                            0.35,
                                            config.simGearing,
                                            config.simDrumRadiusMeters)
                                    .setAngle(90)
                                    .setMovingLength(50)
                                    .setStaticLength(50)
                                    .setMaxHeight(37.5),
                            RobotSim.leftView,
                            motor,
                            "1Elevator");
        }
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    public void setWantedState(WantedState state, double rotations) {
        wantedState = state;
        targetRotations = Math.min(rotations, config.fullExtend);
    }

    public boolean isAt(double rotations) {
        return Math.abs(getPositionRotations() - rotations) < config.tolerance;
    }

    public boolean isAtTarget() {
        return isAt(targetRotations);
    }

    @Override
    public void periodic() {
        switch (wantedState) {
            case OFF -> stop();
            case POSITION, SLOW_POSITION -> {
                // Resting on the bottom stop: holding 0 there only heats the motor.
                if (targetRotations <= config.restRotations
                        && getPositionRotations() <= config.restRotations) {
                    stop();
                } else if (wantedState == WantedState.SLOW_POSITION) {
                    setDynMMPositionFoc(
                            () -> targetRotations,
                            config::getMmCruiseVelocity,
                            config::getSlowMmAcceleration,
                            config::getSlowMmJerk);
                } else {
                    setMMPositionFoc(() -> targetRotations);
                }
            }
        }
        logBatteryUsage();
        Telemetry.log("Elevator/WantedState", wantedState.toString());
        Telemetry.log("Elevator/TargetRotations", targetRotations);
        Telemetry.logDash("Elevator/Rotations", getPositionRotations());
        Telemetry.log("Elevator/AtTarget", isAtTarget());
        logDiagnostics("Elevator");
    }

    /** Zero the carriage where it sits; used from the pit with it on the stop. */
    public Command zeroCommand() {
        return runOnce(this::tareMotor).ignoringDisable(true).withName("Elevator.zero");
    }
}
