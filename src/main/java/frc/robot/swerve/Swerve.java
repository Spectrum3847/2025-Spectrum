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
import edu.wpi.first.networktables.StructPublisher;
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
    @Getter private SwerveConfig config;
    private Notifier simNotifier = null;
    private double lastSimTime;
    private RotationController rotationController;
    private TagCenterAlignController tagCenterAlignController;
    private TagDistanceAlignController tagDistanceAlignController;
    private TranslationXController xController;
    private TranslationYController yController;

    @Getter
    protected SwerveModuleState[] setpoints =
            new SwerveModuleState[] {}; // This currently doesn't do anything

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedPilotPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();

    // Logging publisher
    StructArrayPublisher<SwerveModuleState> moduleStatePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
                    .publish();
    StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault().getStructTopic("SwervePose", Pose2d.struct).publish();

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
        tagCenterAlignController = new TagCenterAlignController(config);
        tagDistanceAlignController = new TagDistanceAlignController(config);
        xController = new TranslationXController(config);
        yController = new TranslationYController(config);

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
        moduleStatePublisher.set(state.ModuleStates);
    }

    /**
     * This method is called periodically and is used to update the pilot's perspective. It ensures
     * that the swerve drive system is aligned correctly based on the pilot's view.
     */
    @Override
    public void periodic() {
        posePublisher.set(getRobotPose());
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
        builder.addDoubleProperty("Pose X", () -> getRobotPose().getX(), null);
        builder.addDoubleProperty("Pose Y", () -> getRobotPose().getY(), null);

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

    // Keep the robot on the field
    private Pose2d keepPoseOnField(Pose2d pose) {
        double halfRobot = config.getRobotLength() / 2;
        double x = pose.getX();
        double y = pose.getY();

        double newX = Util.limit(x, halfRobot, Field.getFieldLength() - halfRobot);
        double newY = Util.limit(y, halfRobot, Field.getFieldWidth() - halfRobot);

        if (x != newX || y != newY) {
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

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
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
                    output = Field.flipAngleIfRed(angleDegrees);
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

    protected double getClosestFieldAngle() {
        // Step 1: Read the angle in radians
        double angleRadians = getRotation().getRadians();

        // Step 2: Convert the angle from radians to degrees
        double angleDegrees = Math.toDegrees(angleRadians);

        // Step 3: Define a table of angles in degrees
        double[] angleTable = {0, 180, 126, -126, 54, -54, 60, -60, 120, -120, 90, -90};

        // Step 4: Find the nearest angle from the table
        double closestAngle = angleTable[0];
        double minDifference = getAngleDifference(angleDegrees, closestAngle);

        for (double angle : angleTable) {
            double difference = getAngleDifference(angleDegrees, angle);
            if (difference < minDifference) {
                minDifference = difference;
                closestAngle = angle;
            }
        }

        // Step 5: Return the nearest angle in Radians
        return Math.toRadians(closestAngle);
    }

    protected Command cardinalReorient() {
        return runOnce(
                () -> {
                    double angleDegrees = getClosestCardinal();
                    reorient(angleDegrees);
                });
    }

    public boolean frontClosestToAngle(double angleDegrees) {
        double heading = getRotation().getDegrees();
        double flippedHeading;
        if (heading > 0) {
            flippedHeading = heading - 180;
        } else {
            flippedHeading = heading + 180;
        }
        double frontDifference = getAngleDifference(heading, angleDegrees);
        double flippedDifference = getAngleDifference(flippedHeading, angleDegrees);

        return frontDifference < flippedDifference;
    }

    // Helper method to calculate the shortest angle difference
    private double getAngleDifference(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2) % 360;
        return diff > 180 ? 360 - diff : diff;
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
    // Tag Center Align Controller
    // --------------------------------------------------------------------------------
    // void resetTagCenterAlignController(double currentMeters) {
    //     tagCenterAlignController.reset(currentMeters);
    // }

    double calculateTagCenterAlignController(
            DoubleSupplier targetMeters, DoubleSupplier currentMeters) {
        return tagCenterAlignController.calculate(
                targetMeters.getAsDouble(), currentMeters.getAsDouble());
    }

    public boolean atTagCenterGoal(double currentMeters) {
        return tagCenterAlignController.atGoal(currentMeters);
    }

    // --------------------------------------------------------------------------------
    // Tag Distance Align Controller
    // --------------------------------------------------------------------------------
    void resetTagDistanceAlignController(double currentMeters) {
        tagDistanceAlignController.reset(currentMeters);
    }

    double calculateTagDistanceAlignController(DoubleSupplier targetArea) {
        boolean front = true;
        if (Robot.getVision().frontLL.targetInView()) {
            front = true;
        } else if (Robot.getVision().backLL.targetInView()) {
            front = false;
        }

        double output =
                tagDistanceAlignController.calculate(
                        targetArea.getAsDouble(), Robot.getVision().getTagTA());

        if (Robot.getVision().tagsInView()) {
            return front ? output : -output;
        } else {
            return 0;
        }
    }

    public boolean atTagDistanceGoal(double currentArea) {
        return tagDistanceAlignController.atGoal(currentArea);
    }

    // --------------------------------------------------------------------------------
    // Translation X Controller
    // --------------------------------------------------------------------------------
    void resetXController() {
        xController.reset(getRobotPose().getX());
    }

    DoubleSupplier calculateXController(DoubleSupplier targetMeters) {
        return () -> xController.calculate(targetMeters.getAsDouble(), getRobotPose().getX());
    }

    // --------------------------------------------------------------------------------
    // Translation Y Controller
    // --------------------------------------------------------------------------------
    void resetYController() {
        yController.reset(getRobotPose().getY());
    }

    DoubleSupplier calculateYController(DoubleSupplier targetMeters) {
        return () -> yController.calculate(targetMeters, getRobotPose()::getY);
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
                        new PIDConstants(6, 0, 0),
                        new PIDConstants(8, 0, 0.2),
                        Robot.kDefaultPeriod),
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
