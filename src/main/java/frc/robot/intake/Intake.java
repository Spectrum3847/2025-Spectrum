package frc.robot.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Intake extends Mechanism {

    public static class IntakeConfig extends Config {

        @Getter private double hasGamePieceVelocity = 50;
        @Getter private double hasGamePieceCurrent = 80;
        @Getter private double hasAlgaeCurrent = 80;
        @Getter private double scoreDelay = 0.2;

        // Algae Voltages and Current
        @Getter @Setter private double algaeIntakeVoltage = -9.0;
        @Getter @Setter private double algaeIntakeSupplyCurrent = 30.0;
        @Getter @Setter private double algaeIntakeTorqueCurrent = -85.0;

        @Getter @Setter private double algaeScoreVoltage = 12.0;
        @Getter @Setter private double algaeScoreSupplyCurrent = 30.0;
        @Getter @Setter private double algaeScoreTorqueCurrent = 180.0;

        // Coral Voltages and Current
        @Getter @Setter private double coralHoldVoltage = 9.0;
        @Getter @Setter private double coralHoldSupplyCurrent = 30.0;
        @Getter @Setter private double coralHoldTorqueCurrent = 28.0;

        @Getter @Setter private double coralIntakeVoltage = 12.0;
        @Getter @Setter private double coralIntakeSupplyCurrent = 30.0;
        @Getter @Setter private double coralIntakeTorqueCurrent = 100.0;

        @Getter @Setter private double coralGroundVoltage = 12.0;
        @Getter @Setter private double coralGroundSupplyCurrent = 40.0;
        @Getter @Setter private double coralGroundTorqueCurrent = 200.0;

        @Getter @Setter private double coralScoreVoltage = -1;
        @Getter @Setter private double coralScoreSupplyCurrent = 12.0;
        @Getter @Setter private double coralScoreTorqueCurrent = -25.0;

        @Getter @Setter private double coralL1ScoreVoltage = -8;
        @Getter @Setter private double coralL1ScoreSupplyCurrent = 15.0;
        @Getter @Setter private double coralL1ScoreTorqueCurrent = -30.0;

        /* Intake config values */
        @Getter private double currentLimit = 44;
        @Getter private double torqueCurrentLimit = 200;
        @Getter private double velocityKp = 12; // 0.156152;
        @Getter private double velocityKv = 0.2; // 0.12;
        @Getter private double velocityKs = 14;

        /* Sim Configs */
        @Getter private double intakeX = 0.8; // relative to elbow at 0 degrees
        @Getter private double intakeY = 1.3; // relative to elbow at 0 degrees
        @Getter private double wheelDiameter = 5.0;

        public IntakeConfig() {
            super("Intake", 5, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }
    }

    private IntakeConfig config;
    private CoralIntakeSim sim;

    public Intake(IntakeConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        IntakeStates.setStates();
    }

    public void setupDefaultCommand() {
        IntakeStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
            // builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
            builder.addDoubleProperty("Velocity RPM", this::getVelocityRPM, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
        }
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command defaultHoldOrStop() {
        return run(
                () -> {
                    if (RobotStates.coral.getAsBoolean()) {
                        // setVoltageOutput(() -> config.getCoralHoldVoltage());
                        // setCurrentLimits(
                        //         () -> config.getCoralHoldSupplyCurrent(),
                        //         () -> config.getCoralHoldTorqueCurrent());
                        setTorqueCurrentFoc(config::getCoralHoldTorqueCurrent);
                    } else if (RobotStates.algae.getAsBoolean()) {
                        setTorqueCurrentFoc(config::getAlgaeIntakeTorqueCurrent);
                    } else {
                        stop();
                    }
                });
    }

    public boolean hasIntakeGamePiece() {
        double motorOutput = getVelocityRPM();
        double motorCurrent = getStatorCurrent();
        return (Math.abs(motorOutput) < config.hasGamePieceVelocity
                && Math.abs(motorCurrent) > config.hasGamePieceCurrent);
    }

    public Command runTorqueFOC(DoubleSupplier torque) {
        return run(() -> setTorqueCurrentFoc(torque));
    }

    public Command intakeCoral(DoubleSupplier torque, DoubleSupplier current) {
        return new FunctionalCommand(
                () -> setCurrentLimits(current, torque),
                () -> {
                    if (hasIntakeGamePiece()) {
                        setVoltageAndCurrentLimits(
                                config::getCoralHoldVoltage,
                                config::getCoralHoldSupplyCurrent,
                                config::getCoralHoldTorqueCurrent);
                    } else {
                        setCurrentLimits(current, torque);
                        setTorqueCurrentFoc(torque);
                    }
                },
                bool -> {},
                () -> false,
                this);
    }

    public Command intakeAlgae(DoubleSupplier torque, DoubleSupplier current) {
        return new FunctionalCommand(
                () -> setCurrentLimits(current, torque),
                () -> {
                    if (hasIntakeGamePiece()) {
                        setVoltageAndCurrentLimits(
                                config::getAlgaeIntakeVoltage,
                                config::getAlgaeIntakeSupplyCurrent,
                                config::getAlgaeIntakeTorqueCurrent);
                    } else {
                        setCurrentLimits(current, torque);
                        setTorqueCurrentFoc(() -> -1 * torque.getAsDouble());
                    }
                },
                bool -> {},
                () -> false,
                this);
    }

    public void setVoltageAndCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supply, DoubleSupplier torque) {
        setVoltageOutput(voltage);
        setCurrentLimits(supply, torque);
    }

    public Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return runVoltage(voltage).alongWith(runCurrentLimits(supplyCurrent, torqueCurrent));
    }

    public Command runTCcurrentLimits(DoubleSupplier torqueCurrent, DoubleSupplier supplyCurrent) {
        return runTorqueCurrentFoc(torqueCurrent)
                .alongWith(runCurrentLimits(supplyCurrent, torqueCurrent));
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
            sim = new CoralIntakeSim(RobotSim.leftView, motor.getSimState());
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class CoralIntakeSim extends RollerSim {
        public CoralIntakeSim(Mechanism2d mech, TalonFXSimState coralRollerMotorSim) {
            super(
                    new RollerConfig(config.wheelDiameter)
                            .setPosition(config.intakeX, config.intakeY)
                            .setMount(Robot.getElbow().getSim()),
                    mech,
                    coralRollerMotorSim,
                    config.getName());
        }
    }
}
