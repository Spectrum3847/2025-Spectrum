package frc.robot.intakePivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import lombok.Getter;
import lombok.Setter;

public class IntakePivot extends Mechanism {

    public static class IntakePivotConfig extends Config {

        //TODO: tune these values

        @Getter @Setter private double home = 0;
        @Getter @Setter private double groundCoralIntake = 4;

        @Getter @Setter private double tolerance = 3.5;

        @Getter @Setter private double offset = -90;
        @Getter @Setter private double initPosition = 0;

        /* IntakePivot config settings */
        @Getter @Setter private double zeroSpeed = -0.1;
        @Getter @Setter private double holdMaxSpeedRPM = 18;

        @Getter @Setter private double currentLimit = 60;
        @Getter @Setter private double torqueCurrentLimit = 80;
        @Getter @Setter private double positionKp = 1500;
        @Getter @Setter private double positionKd = 170;
        @Getter @Setter private double positionKv = 0;
        @Getter @Setter private double positionKs = 0.06;
        @Getter @Setter private double positionKa = 0.001;
        @Getter @Setter private double positionKg = 20.83333; // 12.5 * 1.666666
        @Getter @Setter private double mmCruiseVelocity = 10;
        @Getter @Setter private double mmAcceleration = 50;
        @Getter @Setter private double mmJerk = 0;
        @Getter @Setter private double slowMmAcceleration = 2;
        @Getter @Setter private double slowMmJerk = 50;

        @Getter @Setter private double sensorToMechanismRatio = 61.71428571; // 102.857;
        @Getter @Setter private double rotorToSensorRatio = 1;


        public IntakePivotConfig() {
            // TODO: change id
            super("IntakePivot", 42, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(-1, 0.5);
            configReverseSoftLimit(-1, true);
            configForwardSoftLimit(0.5, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            configGravityType(true);
        }
    }
    
    private IntakePivotConfig config;
    // private IntakePivotSim sim;

    public IntakePivot(IntakePivotConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();

        // simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        IntakePivotStates.setStates();
    }

    public void setupDefaultCommand() {
        IntakePivotStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty(
                    "Position Degrees", () -> (getPositionDegrees() - config.offset), null);
            // builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("MotorVoltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
        }
    }

    void setInitialPosition() {
            motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
    }

    public Command resetToIntialPos() {
        return runOnce(this::setInitialPosition)
                .ignoringDisable(true)
                .withName("Reset to Initial position");
    }

    @Override
    public Trigger belowDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        (getPositionDegrees() - config.getOffset())
                                < (degrees.getAsDouble() - tolerance.getAsDouble()));
    }

    @Override
    public Trigger aboveDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - config.getOffset())
                                > (degrees.getAsDouble() + tolerance.getAsDouble()));
    }

    @Override
    public Trigger atDegrees(DoubleSupplier degrees, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - config.getOffset() - degrees.getAsDouble())
                                < tolerance.getAsDouble());
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the Pivot. */
    public Command runHoldIntakePivot() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("IntakePivot.holdPosition");
                addRequirements(Robot.getIntakePivot());
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
                if (Math.abs(getVelocityRPM()) > config.holdMaxSpeedRPM) {
                    stop();
                    holdPosition = getPositionRotations();
                } else {
                    setDynMMPositionFoc(
                            () -> holdPosition,
                            () -> config.getMmCruiseVelocity(),
                            () -> config.getMmAcceleration(),
                            () -> 20);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(offsetPosition(degrees)).withName(getName() + ".runPoseDegrees");
    }

    public double checkMoveOverTop(DoubleSupplier degrees) {
        double newDeg = degrees.getAsDouble();
        if (newDeg < -90 && (getPositionDegrees() - config.offset) > 90) {
            newDeg += 360;
        } else if (newDeg > 90 && (getPositionDegrees() - config.offset) < -90) {
            newDeg -= 360;
        }
        return newDeg;
    }

    public Command move(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(degrees))
                .withName("IntakePivot.move");
    }

    public DoubleSupplier getOffsetRotations(DoubleSupplier degrees) {
        return () -> degreesToRotations(offsetPosition(() -> checkMoveOverTop(degrees)));
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // void simulationInit() {
    //     if (isAttached()) {
    //         sim = new IntakePivotSim(motor.getSimState(), RobotSim.leftView);
    //         // m_CANcoder.setPosition(0);
    //     }
    // }

    // @Override
    // public void simulationPeriodic() {
    //     if (isAttached()) {
    //         sim.simulationPeriodic();
    //         // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
    //     }
    // }

    // class IntakePivotSim extends ArmSim {
    //     public IntakePivotSim(TalonFXSimState intakePivotMotorSim, Mechanism2d mech) {
    //         super(
    //                 new ArmConfig(
    //                                 config.intakePivotX,
    //                                 config.intakePivotY,
    //                                 config.simRatio,
    //                                 config.length,
    //                                 -360,
    //                                 360.0 - 90.0,
    //                                 -90)
    //                         .setMount(Robot.getElevator().getSim(), true),
    //                 mech,
    //                 intakePivotMotorSim,
    //                 "2" + config.getName()); // added 2 to the name to create it second
    //     }
    // }
}
