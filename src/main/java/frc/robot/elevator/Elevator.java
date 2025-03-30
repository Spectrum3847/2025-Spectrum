package frc.robot.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Elevator extends Mechanism {

    public static class ElevatorConfig extends Config {
        @Getter @Setter private boolean isPhoton = false;

        /* Elevator constants in rotations */
        @Getter @Setter private double maxRotations = 21.1;

        @Getter @Setter private double minRotations = 0.3;

        /* Elevator positions in rotations */
        @Getter @Setter private double fullExtend = maxRotations * .999;
        @Getter @Setter private double home = 0;

        @Getter @Setter private double clawGroundAlgaeIntake = 0;
        @Getter @Setter private double clawGroundCoralIntake = 0;

        @Getter @Setter private double stationIntake = 0;
        @Getter @Setter private double stationExtendedIntake = 0;

        @Getter @Setter private double processorAlgae = 0;
        @Getter @Setter private double l2Algae = 1;
        @Getter @Setter private double l3Algae = 12;
        @Getter @Setter private double netAlgae = fullExtend;

        @Getter @Setter private double l1Coral = 0;
        @Getter @Setter private double l2Coral = 8.6; // 0.68;
        @Getter @Setter private double l2Score = 6.2; // l2Coral;
        @Getter @Setter private double l3Coral = 20; // 12.8; // 10.7;
        @Getter @Setter private double l3Score = 17.6; // l3Coral - 1;
        @Getter @Setter private double l4Coral = fullExtend;
        @Getter @Setter private double l4Score = l4Coral - 3;

        @Getter @Setter private double exl1Coral = 0.3;
        @Getter @Setter private double exl2Coral = 8.6; // 1
        @Getter @Setter private double exl2Score = 6.2; // 0.3;
        @Getter @Setter private double exl3Coral = 20; // 12.8;
        @Getter @Setter private double exl3Score = 17.6; // 11.8;
        @Getter @Setter private double exl4Coral = fullExtend;
        @Getter @Setter private double exl4Score = exl4Coral - 3;

        @Getter private double triggerTolerance = 0.95;
        @Getter private double elevatorIsUpHeight = 5;
        @Getter private double elevatorIsHighHeight = 10;
        @Getter private double initPosition = 0;
        @Getter private double holdMaxSpeedRPM = 1000;

        /* Elevator config settings */
        @Getter private final double zeroSpeed = -0.2;
        @Getter private final double positionKp = 100;
        @Getter private final double positionKd = 8; // 6
        @Getter private final double positionKa = 0.2;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 5;
        @Getter private final double positionKg = 25.3;
        @Getter private final double mmCruiseVelocity = 70;
        @Getter private final double mmAcceleration = 400;
        @Getter private final double mmJerk = 4500;

        @Getter private double currentLimit = 60;
        @Getter private double torqueCurrentLimit = 160;

        /* Sim properties */
        @Getter private double kElevatorGearing = 1.7;
        @Getter private double kCarriageMass = 13.6078;
        @Getter private double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double initialX = 0.8;
        @Getter private double initialY = 0.35;
        @Getter private double angle = 90;
        @Getter private double staticLength = 50;
        @Getter private double movingLength = 50;

        public ElevatorConfig() {
            super("ElevatorFront", 40, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            setFollowerConfigs(new FollowerConfig("ElevatorRear", 41, Rio.CANIVORE, true));
        }

        /** Use these method to set the config for the mechanism on each robot */
        public void configSupplyCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            configSupplyCurrentLimit(currentLimit, true);
        }
    }

    private ElevatorConfig config;
    @Getter private ElevatorSim sim;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        ElevatorStates.setStates();
    }

    public void setupDefaultCommand() {
        ElevatorStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
            // builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
            builder.addDoubleProperty("MotorVoltage", this::getVoltage, null);
        }
    }

    private void setInitialPosition() {
        if (isAttached()) {
            motor.setPosition(config.getInitPosition());
            followerMotors[0].setPosition(config.getInitPosition());
        }
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the elevator. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Elevator.holdPosition");
                addRequirements(Elevator.this);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }

            @Override
            public void initialize() {
                holdPosition = getPositionRotations();
                stop();
            }

            @Override
            public void execute() {
                double currentPosition = getPositionRotations();
                if (Math.abs(currentPosition)
                        < 0.3) { // Added so it doesn't try to hold when all the way down
                    stop();
                } else if (Math.abs(getVelocityRPM()) > config.holdMaxSpeedRPM) {
                    stop(); // Don't hold if moving too fast
                    holdPosition = currentPosition; // Update to a new hold position
                } else {
                    setMMPositionFoc(() -> holdPosition);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command move(DoubleSupplier shrinkRotations, DoubleSupplier exRotations) {
        return run(
                () -> {
                    if (!RobotStates.shrink.getAsBoolean()) {
                        setMMPositionFoc(exRotations);
                    } else {
                        setMMPositionFoc(shrinkRotations);
                    }
                });
    }

    public Command zeroElevatorRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        b -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Elevator.zeroElevatorRoutine");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) { // Only run simulation if it's attached
            sim = new ElevatorSim(motor.getSimState(), RobotSim.leftView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) { // Only run if it's attached
            sim.simulationPeriodic();
        }
    }

    class ElevatorSim extends LinearSim {
        public ElevatorSim(TalonFXSimState elevatorMotorSim, Mechanism2d mech) {
            super(
                    new LinearConfig(
                                    config.initialX,
                                    config.initialY,
                                    config.kElevatorGearing,
                                    config.kElevatorDrumRadiusMeters)
                            .setAngle(config.angle)
                            .setMovingLength(config.getMovingLength())
                            .setStaticLength(config.getStaticLength())
                            .setMaxHeight(30.5 + 7),
                    mech,
                    elevatorMotorSim,
                    "1" + config.getName()); // added 1 to the name to create it first
        }
    }
}
