package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import lombok.Getter;
import lombok.Setter;

public class SwerveConfig {

    @Getter
    private final double scoreOffsetFromReef =
            Units.inchesToMeters(8.0 + 18.5); // Offset + half of robot length with bumpers

    @Getter private final double simLoopPeriod = 0.005; // 5 ms
    @Getter @Setter private double robotWidth = Units.inchesToMeters(29.5);
    @Getter @Setter private double robotLength = Units.inchesToMeters(29.5);

    @Getter @Setter private double maxAngularRate = 1.5 * Math.PI; // rad/s
    @Getter @Setter private double deadband = 0.00;

    @Getter @Setter
    private double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.74603174603;

    @Getter @Setter private double steerGearRatio = 21.428571428571427; // 12.8;

    @Getter @Setter
    // Estimated at first, then fudge-factored to make odom match record
    private Distance wheelRadius = Inches.of(3.815 / 2); // 0.0484505 m

    // Theoretical free speed (m/s) at 12v applied output;
    @Getter @Setter
    private LinearVelocity speedAt12Volts =
            MetersPerSecond.of(
                    (95 / driveGearRatio) * 2 * Math.PI * wheelRadius.in(Meters)); // 4.28699874;

    @Getter private double kSdrive = 0.10; // 0.13
    @Getter private double kSsteer = 0.25; // 0.2

    // -----------------------------------------------------------------------
    // PID Controller Constants
    // -----------------------------------------------------------------------
    @Getter private double maxAngularVelocity = 2 * Math.PI; // rad/s
    @Getter private double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2); // rad/s^2
    @Getter private double kPRotationController = 2; // 4.5 // 6.5 // 8.0;
    @Getter private double kIRotationController = 0.0;
    @Getter private double kDRotationController = 0.0; // 0.2
    @Getter private double rotationTolerance = Units.degreesToRadians(1); // rads
    @Getter private double rotationVelocityTolerance = Units.degreesToRadians(3); // rads/s

    @Getter private double kPHoldController = 0.15;
    @Getter private double kIHoldController = 0.0;
    @Getter private double kDHoldController = 0.0;

    @Getter private double kPTranslationController = 3.0;
    @Getter private double kITranslationController = 0.0;
    @Getter private double kDTranslationController = 0.0;

    @Getter private double translationTolerance = Units.inchesToMeters(0.6); // 0.4
    @Getter private double translationVelocityTolerance = Units.inchesToMeters(0.8); // 0.75

    @Getter
    private Constraints translationConstraints =
            new Constraints(
                    speedAt12Volts.baseUnitMagnitude() * 0.9,
                    Math.pow(speedAt12Volts.baseUnitMagnitude(), 2));

    @Getter private double kPTagCenterController = 1.3;
    @Getter private double kITagCenterController = 0.0;
    @Getter private double kDTagCenterController = 0.00;
    @Getter private double tagCenterTolerance = Units.inchesToMeters(0.5); // meters

    @Getter private double kPTagDistanceController = 0.1; // 0.15;
    @Getter private double kITagDistanceController = 0.0;
    @Getter private double kDTagDistanceController = 0.00;
    @Getter private double tagDistanceTolerance = 0.3; // Area

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    @Getter private final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    @Getter private final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    @Getter
    private Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(4000.0)
                    .withKI(0)
                    .withKD(50.0)
                    .withKS(0.15)
                    .withKV(1.5)
                    .withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // new Slot0Configs().withKP(100).withKI(0).withKD(0.5).withKS(0.1).withKV(1.91).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    @Getter
    private Slot0Configs driveGains =
            new Slot0Configs()
                    .withKP(50.0)
                    .withKI(0)
                    .withKD(0.0)
                    .withKS(2.261118000000002)
                    .withKA(0.0)
                    .withKV(0.0);
    // new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    @Getter
    private ClosedLoopOutputType steerClosedLoopOutput =
            ClosedLoopOutputType.TorqueCurrentFOC; // .Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    @Getter
    private ClosedLoopOutputType driveClosedLoopOutput =
            ClosedLoopOutputType.TorqueCurrentFOC; // .Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    @Getter @Setter private Current slipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    @Getter private TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();

    @Getter
    private TalonFXConfiguration steerInitialConfigs =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can
                                    // set a relatively low stator current limit to help avoid
                                    // brownouts without
                                    // impacting performance.
                                    .withStatorCurrentLimit(Amps.of(60))
                                    .withStatorCurrentLimitEnable(true));

    @Getter private CANcoderConfiguration canCoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    @Getter private Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    @Getter private double coupleRatio = 3.125 * 14.0 / 13.0; // copied from 254-2024

    @Getter @Setter private boolean steerMotorReversed = true;
    @Getter @Setter private boolean invertLeftSide = false;
    @Getter @Setter private boolean invertRightSide = true;

    @Getter @Setter private CANBus canBus = new CANBus("*", "./logs/spectrum.hoot");
    @Getter private int pigeonId = 0;

    // These are only used for simulation
    @Getter private double steerInertia = 0.01;
    @Getter private double driveInertia = 0.01;
    // Simulated voltage necessary to overcome friction
    @Getter private Voltage steerFrictionVoltage = Volts.of(0.25);
    @Getter private Voltage driveFrictionVoltage = Volts.of(0.25);

    @Getter private SwerveDrivetrainConstants drivetrainConstants;

    @Getter
    private SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constantCreator;

    private final double wheelBaseInches = 23.75;
    private final double trackWidthInches = 23.75;

    // Front Left
    @Getter private int frontLeftDriveMotorId = 1;
    @Getter private int frontLeftSteerMotorId = 2;
    @Getter private int frontLeftEncoderId = 3;
    @Getter private Angle frontLeftEncoderOffset = Rotations.of(-0.83544921875);

    @Getter private Distance frontLeftXPos = Inches.of(wheelBaseInches / 2);
    @Getter private Distance frontLeftYPos = Inches.of(trackWidthInches / 2);

    // Front Right
    @Getter private int frontRightDriveMotorId = 11;
    @Getter private int frontRightSteerMotorId = 12;
    @Getter private int frontRightEncoderId = 13;
    @Getter private Angle frontRightEncoderOffset = Rotations.of(-0.15234375);

    @Getter private Distance frontRightXPos = Inches.of(wheelBaseInches / 2);
    @Getter private Distance frontRightYPos = Inches.of(-trackWidthInches / 2);

    // Back Left
    @Getter private int backLeftDriveMotorId = 21;
    @Getter private int backLeftSteerMotorId = 22;
    @Getter private int backLeftEncoderId = 23;
    @Getter private Angle backLeftEncoderOffset = Rotations.of(-0.4794921875);

    @Getter private Distance backLeftXPos = Inches.of(-wheelBaseInches / 2);
    @Getter private Distance backLeftYPos = Inches.of(trackWidthInches / 2);

    // Back Right
    @Getter private int backRightDriveMotorId = 31;
    @Getter private int backRightSteerMotorId = 32;
    @Getter private int backRightEncoderId = 33;
    @Getter private Angle backRightEncoderOffset = Rotations.of(-0.84130859375);

    @Getter private Distance backRightXPos = Inches.of(-wheelBaseInches / 2);
    @Getter private Distance backRightYPos = Inches.of(-trackWidthInches / 2);

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontLeft;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontRight;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backLeft;

    @Getter
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backRight;

    @Getter @Setter private double targetHeading = 0;

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    []
            modules;

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            [] getModules() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            modules = new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
        } else {
            throw new IllegalStateException("One or more SwerveModuleConstants are null");
        }
        return modules;
    }

    public SwerveConfig() {
        updateConfig();
    }

    public SwerveConfig updateConfig() {
        drivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withCANBusName(canBus.getName())
                        .withPigeon2Id(pigeonId)
                        .withPigeon2Configs(pigeonConfigs);

        constantCreator =
                new SwerveModuleConstantsFactory<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(driveGearRatio)
                        .withSteerMotorGearRatio(steerGearRatio)
                        .withWheelRadius(wheelRadius)
                        .withSlipCurrent(slipCurrent)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12Volts(speedAt12Volts)
                        .withSteerInertia(steerInertia)
                        .withDriveInertia(driveInertia)
                        .withSteerFrictionVoltage(steerFrictionVoltage)
                        .withDriveFrictionVoltage(driveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(coupleRatio)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs);

        frontLeft =
                constantCreator.createModuleConstants(
                        frontLeftSteerMotorId,
                        frontLeftDriveMotorId,
                        frontLeftEncoderId,
                        frontLeftEncoderOffset,
                        frontLeftXPos,
                        frontLeftYPos,
                        invertLeftSide,
                        steerMotorReversed,
                        false);

        frontRight =
                constantCreator.createModuleConstants(
                        frontRightSteerMotorId,
                        frontRightDriveMotorId,
                        frontRightEncoderId,
                        frontRightEncoderOffset,
                        frontRightXPos,
                        frontRightYPos,
                        invertRightSide,
                        steerMotorReversed,
                        false);

        backLeft =
                constantCreator.createModuleConstants(
                        backLeftSteerMotorId,
                        backLeftDriveMotorId,
                        backLeftEncoderId,
                        backLeftEncoderOffset,
                        backLeftXPos,
                        backLeftYPos,
                        invertLeftSide,
                        steerMotorReversed,
                        false);

        backRight =
                constantCreator.createModuleConstants(
                        backRightSteerMotorId,
                        backRightDriveMotorId,
                        backRightEncoderId,
                        backRightEncoderOffset,
                        backRightXPos,
                        backRightYPos,
                        invertRightSide,
                        steerMotorReversed,
                        false);

        return this;
    }

    public SwerveConfig configEncoderOffsets(
            double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftEncoderOffset = Rotations.of(frontLeft);
        frontRightEncoderOffset = Rotations.of(frontRight);
        backLeftEncoderOffset = Rotations.of(backLeft);
        backRightEncoderOffset = Rotations.of(backRight);
        return updateConfig();
    }
}
