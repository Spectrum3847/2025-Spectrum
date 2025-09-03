package frc.robot.twist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.SpectrumCANcoderConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Twist extends Mechanism {

    public static class TwistConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        // Positions set as percentage of Twist
        @Getter private final int initializedPosition = 20;

        @Getter private final double stageDelay = 0.05;

        /* twist positions in percentage of max rotation || 0 is horizontal */

        @Getter private final double home = 90;
        @Getter private final double lollipopCoral = 90;
        @Getter private final double stationIntake = 0; // 179.9;
        @Getter private final double algaeIntake = 179.9;
        @Getter private final double groundAlgaeIntake = 0;
        @Getter private final double groundCoralIntake = 179.9;
        @Getter private final double leftCoral = 90;
        @Getter private final double rightCoral = 270; // -90;
        @Getter private final double l1Coral = 0;
        @Getter private final double processorAlgae = 0;
        @Getter private final double net = algaeIntake;
        @Getter private final double climbPrep = 179.9;

        @Getter private final double initPosition = 0;
        @Getter private double triggerTolerance = 5;
        @Getter private double twistTolerance = 45;

        /* Twist config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

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

        @Getter @Setter private double sensorToMechanismRatio = 22.4;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 22.4;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 1;

        @Getter @Setter private double CANcoderOffset = 0;
        @Getter @Setter private boolean CANcoderAttached = false;

        /* Sim properties */
        @Getter private double twistX = 0.525;
        @Getter private double twistY = 1.35;
        @Getter @Setter private double simRatio = 22.4;
        @Getter private double coralLength = 0.255;
        @Getter private double algaeLength = 0.205;
        @Getter private double coralLineWeight = 15.0;
        @Getter private double algaeLineWeight = 4.0;
        @Getter private double coralBaseAngle = -60;
        @Getter private double algaeBaseAngle = 50;
        @Getter private Color8Bit coralColor = new Color8Bit(Color.kBlue);
        @Getter private Color8Bit algaeColor = new Color8Bit(Color.kAqua);

        public TwistConfig() {
            super("Twist", 44, Rio.CANIVORE); // Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-.25, 0.75);
            configReverseSoftLimit(getMinRotations(), false);
            configForwardSoftLimit(getMaxRotations(), false);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(true);
            configClockwise_Positive();
        }

        public TwistConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    @Getter private TwistConfig config;
    private SpectrumCANcoder canCoder;
    private SpectrumCANcoderConfig canCoderConfig;
    @Getter private TwistSim sim;
    CANcoderSimState canCoderSim;

    public Twist(TwistConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            if (config.isCANcoderAttached() && !Robot.isSimulation()) {
                canCoderConfig =
                        new SpectrumCANcoderConfig(
                                config.getCANcoderRotorToSensorRatio(),
                                config.getCANcoderSensorToMechanismRatio(),
                                config.getCANcoderOffset(),
                                config.isCANcoderAttached());
                canCoder =
                        new SpectrumCANcoder(
                                44,
                                canCoderConfig,
                                motor,
                                config,
                                SpectrumCANcoder.CANCoderFeedbackType.FusedCANcoder);
            }

            setInitialPosition();
        }

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    public void setupStates() {
        TwistStates.setStates();
    }

    public void setupDefaultCommand() {
        TwistStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty("Degrees", this::getPositionDegrees, null);
            // builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
            // builder.addDoubleProperty("Front-TX", Robot.getVision().frontLL::getTagTx, null);
            // builder.addDoubleProperty("Front-TA", Robot.getVision().frontLL::getTagTA, null);
            // builder.addDoubleProperty(
            //        "Front-Rotation", Robot.getVision().frontLL::getTagRotationDegrees, null);
            // builder.addDoubleProperty(
            //        "Front-ClosestTag", Robot.getVision().frontLL::getClosestTagID, null);
        }
    }

    private void setInitialPosition() {
        if (canCoder != null) {
            if (canCoder.isAttached()
                    && canCoder.canCoderResponseOK(
                            canCoder.getCanCoder().getAbsolutePosition().getStatus())) {
                motor.setPosition(
                        canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                                / config.getCANcoderSensorToMechanismRatio());
            } else {
                motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
            }
        } else {
            motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
        }
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the Twist. */
    public Command runHoldTwist() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Twist.holdPosition");
                addRequirements(Twist.this);
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
        };
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(degrees).withName(getName() + ".runPoseDegrees");
    }

    private void setDegrees(DoubleSupplier degrees) {
        setMMPositionFoc(() -> degreesToRotations(degrees));
    }

    public Command move(DoubleSupplier targetDegrees, boolean clockwise) {
        return run(
                () -> {
                    double currentDegrees = getPositionDegrees();
                    // Normalize targetDegrees to be within 0 to 360
                    double target = (targetDegrees.getAsDouble() % 360);
                    // Normalize currentDegrees to be within 0 to 360
                    double currentMod = (currentDegrees % 360);
                    if (currentMod < 0) {
                        currentMod += 360;
                    }

                    double output = currentDegrees;

                    if (Math.abs(currentMod - target)
                            < config.getTwistTolerance()) { // check if the difference is within
                        // tolerance
                        if (currentMod - target > 0) {
                            output = currentDegrees - (currentMod - target);
                        } else {
                            output = currentDegrees + (target - currentMod);
                        }
                    } else if (Math.abs(currentMod - target) > 360 - config.getTwistTolerance()
                            && Math.abs(Math.abs(currentMod - target) - 360)
                                    < config.getTwistTolerance()) { // check if the difference is
                        // within tolerance and currentMod or target is near 0 and 360
                        if (currentMod < target) {
                            if (currentMod - (target - 360) > 0) {
                                output = currentDegrees + (currentMod - (target - 360));
                            } else {
                                output = currentDegrees - (currentMod - (target - 360));
                            }
                        } else {
                            if ((currentMod - 360) - target > 0) {
                                output = currentDegrees + ((currentMod - 360) - target);
                            } else {
                                output = currentDegrees - ((currentMod - 360) - target);
                            }
                        }
                    } else {
                        if (clockwise) {
                            // Calculate the closest clockwise position
                            if (currentMod > target) {
                                output = currentDegrees - (currentMod - target);
                            } else if (currentMod < target) {
                                output = currentDegrees - (360 + currentMod - target);
                            }
                        } else {
                            // Calculate the closest counterclockwise position
                            if (currentMod < target) {
                                output = currentDegrees + (target - currentMod);
                            } else if (currentMod > target) {
                                output = currentDegrees + (360 + target - currentMod);
                            }
                        }
                    }

                    final double out = output;
                    setDegrees(() -> out);
                });
    }

    // public Command move(DoubleSupplier targetDegrees, BooleanSupplier clockwise) {
    //     return move(targetDegrees, clockwise.getAsBoolean());
    // }

    public Command moveAwayFromElevatorCheckReverse(DoubleSupplier degrees) {
        return Commands.either(
                        moveAwayFromElevatorReversed(() -> adjustTargetToReverse(degrees))
                                .until(() -> RobotStates.reverse.getAsBoolean() != true),
                        moveAwayFromElevator(() -> adjustTargetToReverse(degrees))
                                .until(() -> RobotStates.reverse.getAsBoolean() != false),
                        () -> RobotStates.reverse.getAsBoolean())
                .repeatedly();
    }

    public Command moveAwayFromElevator(DoubleSupplier degrees) {
        return Commands.either(
                move(degrees, true),
                move(degrees, false),
                () -> checkClockwiseFromElevator(degrees, false));
    }

    public Command moveAwayFromElevatorReversed(DoubleSupplier degrees) {
        return Commands.either(
                move(degrees, true),
                move(degrees, false),
                () -> checkClockwiseFromElevator(degrees, true));
    }

    public Command homeAwayFromElevator(DoubleSupplier degrees) {
        return Commands.either(
                move(degrees, true),
                move(degrees, false),
                () -> checkClockwiseFromElevator(degrees, !RobotStates.reverse.getAsBoolean()));
    }

    public Command moveAwayFromBranchCheckReversed(DoubleSupplier degrees) {
        return Commands.either(
                move(() -> adjustTargetToReverse(degrees), true),
                move(() -> adjustTargetToReverse(degrees), false),
                () ->
                        checkClockwiseAwayFromBranch(
                                () -> adjustTargetToReverse(degrees),
                                RobotStates.reverse.getAsBoolean()));
    }
    // @deprecated
    public double netTurretDegrees() {
        // uses the robot pose to always point the twist away from the driver station
        double offset = 180;
        if (!Field.isBlue()) {
            offset += 180;
        }
        if (Field.isBlue() != Robot.getSwerve().inXzone(0, Field.getHalfLength()).getAsBoolean()) {
            offset += 180;
        }
        return (-Robot.getSwerve().getRobotPose().getRotation().getDegrees() + offset) % 360;
    }

    public Command netTurret() {
        return move(() -> adjustTargetToReverse(() -> netTurretDegrees()), netTurretClockwise());
    }

    public Boolean netTurretClockwise() {
        boolean clockwise = false;
        double currentDegrees = getPositionDegrees();
        double target = netTurretDegrees();
        // Calculate the closest clockwise position
        double clockwiseOutput, counterclockwiseOutput;
        if (currentDegrees > target) {
            clockwiseOutput = currentDegrees - (currentDegrees - target);
        } else {
            clockwiseOutput = currentDegrees - (360 + currentDegrees - target);
        }
        if (currentDegrees < target) {
            counterclockwiseOutput = currentDegrees + (target - currentDegrees);
        } else {
            counterclockwiseOutput = currentDegrees + (360 + target - currentDegrees);
        }
        // Compare the two outputs and set clockwise accordingly to get to the target faster
        clockwise =
                Math.abs(currentDegrees - clockwiseOutput)
                        < Math.abs(currentDegrees - counterclockwiseOutput);
        return clockwise;
    }

    public double adjustTargetToReverse(DoubleSupplier degrees) {
        if (RobotStates.reverse.getAsBoolean()) {
            if (degrees.getAsDouble() + 180 > 270) {
                return degrees.getAsDouble() - 180;
            }
            return degrees.getAsDouble() + 180;
        }
        return degrees.getAsDouble();
    }

    private boolean checkClockwiseFromElevator(DoubleSupplier degrees, boolean reverse) {
        double degreesMod = degrees.getAsDouble() % 360;
        double currentMod = getPositionDegrees() % 360;

        if (currentMod < 0) {
            currentMod += 360;
        }

        // Telemetry.print("TARGET: " + degreesMod);
        // Telemetry.print("CURRENT: " + currentMod);

        // check if moving from within to past elevator
        if (degreesMod > 180 + config.getTwistTolerance()
                && degreesMod < 360 - config.getTwistTolerance()
                && currentMod < 180 + config.getTwistTolerance()) {
            if (reverse) {
                // Telemetry.print("1: false");
                return false;
            }
            // System.out.println("1: true");
            return true;
        }

        // check if moving from past elevator back to within
        if (degreesMod < 180 + config.getTwistTolerance()
                && currentMod > 180 + config.getTwistTolerance()
                && currentMod < 360 - config.getTwistTolerance()) {
            if (reverse) {
                // Telemetry.print("2: false");
                return false;
            }
            // Telemetry.print("2: true");
            return true;
        }

        if (degreesMod > 180 + config.getTwistTolerance()
                && degreesMod < 360 - config.getTwistTolerance()
                && currentMod > 180 + config.getTwistTolerance()
                && currentMod < 360 - config.getTwistTolerance()) {
            if (degreesMod < currentMod) {
                // Telemetry.print("3: false");
                return false;
            }
            // Telemetry.print("3: true");
            return true;
        }

        if (degreesMod < currentMod && currentMod < 360 - config.getTwistTolerance()) {
            // Telemetry.print("4: true");
            return true;
        }
        // Telemetry.print("4: false");
        return false;
    }

    private boolean checkClockwiseAwayFromBranch(DoubleSupplier degrees, boolean reverse) {
        double degreesMod = degrees.getAsDouble() % 360;
        double currentMod = getPositionDegrees() % 360;

        // Telemetry.print("TARGET: " + degreesMod);
        // Telemetry.print("CURRENT: " + currentMod);

        if (currentMod < 0) {
            currentMod += 360;
        }

        if (degreesMod < 180 && currentMod > 180) {
            if (reverse) {
                return false;
            }
            return true;
        }

        if (degreesMod > 180 && currentMod < 180) {
            if (reverse) {
                return true;
            }
            return false;
        }

        if (degreesMod > 180 && currentMod > 180) {
            if (degreesMod < currentMod) {
                return false;
            }
            return true;
        }

        if (degreesMod < currentMod) {
            return true;
        }
        return false;
    }

    public DoubleSupplier getIfReversedDegrees(DoubleSupplier degrees) {
        return () ->
                (RobotStates.reverse.getAsBoolean())
                        ? degrees.getAsDouble() + 180
                        : degrees.getAsDouble();
    }

    public Command twistHome() {
        return homeAwayFromElevator(config::getHome).withName("Twist.home");
    }

    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(((getPositionDegrees() % 360) + 360) % 360 - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new TwistSim(motor.getSimState(), RobotSim.leftView, this);

            // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        }
    }
}
