// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.spectrumLib.SpectrumSubsystem;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements SpectrumSubsystem, NTSendable {
    private SwerveConfig config;
    private Notifier simNotifier = null;
    private double lastSimTime;
    private RotationController rotationController;

    @Getter
    protected SwerveModuleState[] setpoints =
            new SwerveModuleState[] {}; // This currently doesn't do anything

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedPilotPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();

    // Logging publisher
    StructArrayPublisher<SwerveModuleState> publisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
                    .publish();

    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config The configuration object containing drivetrain constants and module
     *     configurations.
     */
    public Swerve(SwerveConfig config) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.getDrivetrainConstants(),
                config.getModules());
        // this.robotConfig = robotConfig;
        this.config = config;
        configurePathPlanner();

        rotationController = new RotationController(config);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        SendableRegistry.add(this, "Swerve");
        SmartDashboard.putData(this);
        Robot.add(this);
        this.register();
        registerTelemetry(this::log);
        Telemetry.print(getName() + " Subsystem Initialized: ");
    }

    protected void log(SwerveDriveState state) {
        publisher.set(state.ModuleStates);
    }

    /**
     * This method is called periodically and is used to update the pilot's perspective. It ensures
     * that the swerve drive system is aligned correctly based on the pilot's view.
     */
    @Override
    public void periodic() {
        setPilotPerspective();
    }

    public void setupStates() {
        SwerveStates.setStates();
    }

    public void setupDefaultCommand() {
        SwerveStates.setupDefaultCommand();
    }

    /**
     * The `initSendable` function sets up properties for a SmartDashboard type "SwerveDrive" with
     * position and velocity double values.
     *
     * @param builder The `builder` parameter is an instance of the `NTSendableBuilder` class
     */
    @Override
    public void initSendable(NTSendableBuilder builder) {
        SmartDashboard.putData(
                "Swerve Drive",
                new Sendable() {
                    @Override
                    public void initSendable(SendableBuilder builder) {
                        builder.setSmartDashboardType("SwerveDrive");

                        addModuleProperties(builder, "Front Left", 0);
                        addModuleProperties(builder, "Front Right", 1);
                        addModuleProperties(builder, "Back Left", 2);
                        addModuleProperties(builder, "Back Right", 3);

                        builder.addDoubleProperty("Robot Angle", () -> getRotationRadians(), null);
                    }
                });
    }

    private void addModuleProperties(SendableBuilder builder, String moduleName, int moduleNumber) {
        builder.addDoubleProperty(
                moduleName + " Angle",
                () -> getModule(moduleNumber).getCurrentState().angle.getRadians(),
                null);
        builder.addDoubleProperty(
                moduleName + " Velocity",
                () -> getModule(moduleNumber).getCurrentState().speedMetersPerSecond,
                null);
    }

    /**
     * The function `getRobotPose` returns the robot's pose after checking and updating it.
     *
     * @return The `getRobotPose` method is returning the robot's current pose after calling the
     *     `seedCheckedPose` method with the current pose as an argument.
     */
    public Pose2d getRobotPose() {
        Pose2d pose = getState().Pose;
        return keepPoseOnField(pose);
    }

    private Pose2d keepPoseOnField(Pose2d pose) {
        double halfRobot = config.getRobotLength() / 2;
        double x = pose.getX();
        double y = pose.getY();

        // Ensure the robot stays within the field
        double newX = Util.limit(x, halfRobot, Field.getFieldLength() - halfRobot);
        double newY = Util.limit(y, halfRobot, Field.getFieldWidth() - halfRobot);

        // If position changed, update the pose and reset
        if (x != newX || y != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
            resetPose(pose);
        }

        //176.746 half the length of the reef 
        //y=(

        double minX = Units.inchesToMeters(144.003);
        double maxX = Units.inchesToMeters(209.489);
        double minY = Units.inchesToMeters(130.145);
        double maxY = Units.inchesToMeters(186.857);

        // If the robot is inside the forbidden zone, push it out
        if (newX >= minX && newX <= maxX && newY >= minY && newY <= maxY) {
            // Move the robot to the closest valid boundary
            if (x < minX) newX = minX - halfRobot; // Push left
            if (x > maxX) newX = maxX + halfRobot; // Push right
            if (y < minY) newY = minY - halfRobot; // Push down
            if (y > maxY) newY = maxY + halfRobot; // Push up
        } 
 
        if (x != newX && y != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
            resetPose(pose);
        }

        return pose;
    }

    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }

    public Trigger inYzone(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getY(), () -> minYmeter, () -> maxYmeter));
    }

    /**
     * This method is used to check if the robot is in the X zone of the field flips the values if
     * Red Alliance
     *
     * @param minXmeter
     * @param maxXmeter
     * @return
     */
    public Trigger inXzoneAlliance(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(Field.flipXifRed(getRobotPose().getX()), minXmeter, maxXmeter));
    }

    /**
     * This method is used to check if the robot is in the Y zone of the field flips the values if
     * Red Alliance
     *
     * @param minYmeter
     * @param maxYmeter
     * @return
     */
    public Trigger inYzoneAlliance(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(Field.flipYifRed(getRobotPose().getY()), minYmeter, maxYmeter));
    }

    // Used to set a control request to the swerve module, ignores disable so commands are
    // continuous.
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).ignoringDisable(true);
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private void setPilotPerspective() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedPilotPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            allianceColor -> {
                                this.setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? config.getRedAlliancePerspectiveRotation()
                                                : config.getBlueAlliancePerspectiveRotation());
                                hasAppliedPilotPerspective = true;
                            });
        }
    }

    protected void reorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

    protected Command reorientPilotAngle(double angleDegrees) {
        return runOnce(
                () -> {
                    double output;
                    output = Field.flipTrueAngleIfRed(angleDegrees);
                    reorient(output);
                });
    }

    protected double getClosestCardinal() {
        double heading = getRotation().getRadians();
        if (heading > -Math.PI / 4 && heading <= Math.PI / 4) {
            return 0;
        } else if (heading > Math.PI / 4 && heading <= 3 * Math.PI / 4) {
            return 90;
        } else if (heading > 3 * Math.PI / 4 || heading <= -3 * Math.PI / 4) {
            return 180;
        } else {
            return 270;
        }
    }

    protected double getClosest45() {
        double angleRadians = getRotation().getRadians();
        double angleDegrees = Math.toDegrees(angleRadians);

        // Normalize the angle to be within 0 to 360 degrees
        angleDegrees = angleDegrees % 360;
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        // Round to the nearest multiple of 45 degrees
        double closest45Degrees = Math.round(angleDegrees / 45.0) * 45.0;

        // Convert back to radians and return as a Rotation2d
        return Rotation2d.fromDegrees(closest45Degrees).getRadians();
    }

    protected Command cardinalReorient() {
        return runOnce(
                () -> {
                    double angleDegrees = getClosestCardinal();
                    reorient(angleDegrees);
                });
    }

    // --------------------------------------------------------------------------------
    // Rotation Controller
    // --------------------------------------------------------------------------------
    double getRotationControl(double goalRadians) {
        return rotationController.calculate(goalRadians, getRotationRadians());
    }

    void resetRotationController() {
        rotationController.reset(getRotationRadians());
    }

    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    double calculateRotationController(DoubleSupplier targetRadians) {
        return rotationController.calculate(targetRadians.getAsDouble(), getRotationRadians());
    }

    // --------------------------------------------------------------------------------
    // Path Planner
    // --------------------------------------------------------------------------------
    private void configurePathPlanner() {
        // Seed robot to mid field at start (Paths will change this starting position)
        resetPose(
                new Pose2d(
                        Units.feetToMeters(27.0),
                        Units.feetToMeters(27.0 / 2.0),
                        config.getBlueAlliancePerspectiveRotation()));
        double driveBaseRadius = .4;
        for (var moduleLocation : getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        RobotConfig robotConfig = null; // Initialize with null in case of exception
        try {
            robotConfig =
                    RobotConfig.fromGUISettings(); // Takes config from Robot Config on Pathplanner
            // Settings
        } catch (Exception e) {
            e.printStackTrace(); // Fallback to a default configuration
        }

        AutoBuilder.configure(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                speeds ->
                        this.setControl(
                                AutoRequest.withSpeeds(
                                        speeds)), // Consumer of ChassisSpeeds to drive the robot
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0), Robot.kDefaultPeriod),
                robotConfig,
                () ->
                        DriverStation.getAlliance().orElse(Alliance.Blue)
                                == Alliance.Red, // Assume the path needs to be flipped for Red vs
                // Blue, this is normally
                // the case
                this); // Subsystem for requirements
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - lastSimTime;
                            lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        simNotifier.startPeriodic(config.getSimLoopPeriod());
    }
}
