package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.spectrumLib.vision.LimelightHelpers.LimelightResults;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * Provides a high-level interface to a single Limelight camera for AprilTag-based pose estimation
 * and basic targeting.
 *
 * <p>All methods are safe to call when the camera is not attached ({@link #isAttached()} returns
 * {@code false}); they return zero / false / empty values in that case.
 *
 * <p>NetworkTables reads are snapshotted per robot loop: the first getter to need a topic reads it
 * once and every later getter in the same loop reuses that sample, so pose, tag count, raw
 * fiducials and timestamp always describe the same camera frame. The owning subsystem must call
 * {@link #invalidate()} exactly once per loop before any getter. Returned {@link PoseEstimate} and
 * {@link RawFiducial} instances are shared for the rest of the loop and must not be mutated.
 *
 * <p>Two pose estimation flavors are supported:
 *
 * <ul>
 *   <li><b>MegaTag1</b> — full 3-D pose estimation using one or more AprilTags.
 *   <li><b>MegaTag2</b> — fused estimate that incorporates the robot heading supplied via {@link
 *       #setRobotOrientation(double)}.
 * </ul>
 */
public class Limelight {

    /* Limelight Configuration */

    /**
     * Configuration for a single Limelight camera, including its network-table name and physical
     * mounting position on the robot.
     *
     * <p>The Lombok {@code @Accessors(chain = true)} annotation allows fluent setter calls: {@code
     * config.setName("limelight").setAttached(true)}.
     */
    @Accessors(chain = true)
    public static class LimelightConfig {
        /** Must match to the name given in LL dashboard */
        @Getter @Setter private String name;

        /** Whether this camera is physically connected to the robot. */
        @Getter @Setter private boolean attached = true;

        /**
         * Whether pose measurements from this camera are currently being fused into the estimator.
         */
        @Getter @Setter private boolean isIntegrating;

        /** Physical Config */
        /**
         * Forward offset of the camera from the robot center in meters (positive = toward front).
         */
        @Getter private double forward, right, up; // meters

        /** Orientation of the camera in degrees (roll/pitch/yaw relative to robot frame). */
        @Getter private double roll, pitch, yaw; // degrees

        /**
         * Creates a configuration for the named Limelight camera.
         *
         * @param name the network-table name assigned to the camera in the LL dashboard
         */
        public LimelightConfig(String name) {
            this.name = name;
        }

        /**
         * Sets the position of the robot in meters
         *
         * @param forward (meters) forward from center of robot
         * @param right (meters) right from center of robot
         * @param up (meters) up from center of robot
         * @return the LimelightConfig object
         */
        public LimelightConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /**
         * Sets the rotation of the limelight in degrees
         *
         * @param roll (degrees) roll of limelight || positive is rotated right
         * @param pitch (degrees) pitch of limelight || positive is camera tilted up
         * @param yaw (yaw) yaw of limelight || positive is rotated left
         * @return the LimelightConfig object
         */
        public LimelightConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }
    }

    /* Debug */
    /** Formatter used for printing pose coordinates to SmartDashboard. */
    private final DecimalFormat df = new DecimalFormat();

    /** Active configuration for this Limelight instance. */
    private LimelightConfig config;

    /** Whether pose measurements from this camera are currently being integrated. */
    @Getter @Setter private boolean isIntegrating = false;

    /** Network-table name of this camera (mirrors {@link LimelightConfig#getName()}). */
    @Getter private String cameraName = "default";

    /** Human-readable string describing the current integration status, logged for diagnostics. */
    @Getter @Setter private String logStatus = "";

    /** Human-readable string describing the currently visible tag(s), logged for diagnostics. */
    @Getter @Setter private String tagStatus = "";

    /* ::: Per-loop NetworkTables snapshot :::
     * Lazily filled on first use within a robot loop and cleared by invalidate(). null (or a
     * false *Cached flag) means "not read yet this loop". Main robot thread only; no
     * synchronization. */

    /** Raw botpose_wpiblue sample (value + server timestamp); null until first MT1 read. */
    private TimestampedDoubleArray mt1Sample;

    /** MegaTag1 estimate parsed from {@link #mt1Sample}; null until derived. */
    private PoseEstimate mt1Estimate;

    /** MegaTag1 Pose3d derived from {@link #mt1Sample}; null until derived. */
    private Pose3d mt1Pose3d;

    /** MegaTag2 estimate from botpose_orb_wpiblue; null until first MT2 read. */
    private PoseEstimate mt2Estimate;

    private boolean tvCached;
    private boolean tvValue;
    private boolean taCached;
    private double taValue;

    /* ::: Per-loop integration telemetry, maintained by the owning subsystem ::: */

    /** Whether an estimate from this camera was fused into the pose estimator this loop. */
    @Getter @Setter private boolean integratedThisLoop;

    /**
     * Age in seconds (capture time to now) of the last estimate this camera produced this loop, or
     * NaN if it produced none. A large value means the camera's timestamps are not trustworthy.
     */
    @Getter @Setter private double lastEstimateAgeSeconds = Double.NaN;

    /**
     * Clears the per-loop NetworkTables snapshot so the next getter re-reads the camera.
     *
     * <p>Must be called exactly once per robot loop, before any pose/target getter, by the owning
     * subsystem (see {@code Vision.periodic()}). Between calls, every getter on this object returns
     * data from the same NetworkTables sample. Safe to call on a detached camera.
     */
    public void invalidate() {
        mt1Sample = null;
        mt1Estimate = null;
        mt1Pose3d = null;
        mt2Estimate = null;
        tvCached = false;
        taCached = false;
        integratedThisLoop = false;
        lastEstimateAgeSeconds = Double.NaN;
    }

    /** Raw MegaTag1 sample for this loop. Caller must have checked {@link #isAttached()}. */
    private TimestampedDoubleArray mt1Sample() {
        if (mt1Sample == null) {
            mt1Sample =
                    LimelightHelpers.getLimelightDoubleArrayEntry(
                                    config.getName(), "botpose_wpiblue")
                            .getAtomic();
        }
        return mt1Sample;
    }

    /** MegaTag1 estimate for this loop. Caller must have checked {@link #isAttached()}. */
    private PoseEstimate mt1Estimate() {
        if (mt1Estimate == null) {
            TimestampedDoubleArray sample = mt1Sample();
            mt1Estimate = LimelightHelpers.parsePoseEstimate(sample.value, sample.timestamp, false);
        }
        return mt1Estimate;
    }

    /** MegaTag1 Pose3d for this loop. Caller must have checked {@link #isAttached()}. */
    private Pose3d mt1Pose3d() {
        if (mt1Pose3d == null) {
            mt1Pose3d = LimelightHelpers.toPose3D(mt1Sample().value);
        }
        return mt1Pose3d;
    }

    /** MegaTag2 estimate for this loop. Caller must have checked {@link #isAttached()}. */
    private PoseEstimate mt2Estimate() {
        if (mt2Estimate == null) {
            mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.getName());
        }
        return mt2Estimate;
    }

    /**
     * Constructs a Limelight wrapper from a fully populated {@link LimelightConfig}.
     *
     * @param config the camera configuration
     */
    public Limelight(LimelightConfig config) {
        this.config = config;
        cameraName = config.getName();
    }

    /**
     * Constructs a Limelight wrapper with a default configuration for the given camera name.
     *
     * @param name the network-table name of the camera
     */
    public Limelight(String name) {
        cameraName = name;
        config = new LimelightConfig(name);
    }

    /**
     * Constructs a Limelight wrapper, explicitly setting whether the camera is attached.
     *
     * @param name the network-table name of the camera
     * @param attached {@code true} if the camera is physically present on the robot
     */
    public Limelight(String name, boolean attached) {
        cameraName = name;
        config = new LimelightConfig(name).setAttached(attached);
    }

    /**
     * Constructs a Limelight wrapper and immediately sets its active pipeline.
     *
     * @param name the network-table name of the camera
     * @param pipeline the pipeline index to activate (refer to robot vision configuration for
     *     pipeline indexes)
     */
    public Limelight(String name, int pipeline) {
        this(name);
        cameraName = name;
        setLimelightPipeline(pipeline);
    }

    /**
     * Constructs a Limelight wrapper with an explicit configuration and immediately sets its active
     * pipeline.
     *
     * @param name the network-table name of the camera
     * @param pipeline the pipeline index to activate
     * @param config the fully populated {@link LimelightConfig} to use
     */
    public Limelight(String name, int pipeline, LimelightConfig config) {
        this(name);
        cameraName = name;
        this.config = config;
        setLimelightPipeline(pipeline);
    }

    /**
     * Returns the network-table name of this camera.
     *
     * @return the camera name as configured in the LL dashboard
     */
    public String getName() {
        return config.getName();
    }

    /**
     * Returns whether this camera is physically connected to the robot.
     *
     * @return {@code true} if attached
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    /* ::: Basic Information Retrieval ::: */
    /**
     * Get the horizontal offset from crosshair to target
     *
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2:
     *     -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTX(config.getName());
    }

    /**
     * Get the vertical offset from crosshair to target
     *
     * @return Vertical Offset From Crosshair To Target in degrees (LL1: -20.5 degrees to 20.5
     *     degrees / LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTY(config.getName());
    }

    /**
     * Returns whether the LL has any valid targets (April tags or other vision targets)
     *
     * @return Whether the LL has any valid targets (April tags or other vision targets)
     */
    public boolean targetInView() {
        if (!isAttached()) {
            return false;
        }
        if (!tvCached) {
            tvValue = LimelightHelpers.getTV(config.getName());
            tvCached = true;
        }
        return tvValue;
    }

    /**
     * Checks if the LL sees multiple tags
     *
     * @return whether the LL sees multiple tags or not
     */
    public boolean multipleTagsInView() {
        if (!isAttached()) {
            return false;
        }
        return getTagCountInView() > 1;
    }

    /**
     * Returns the number of AprilTags included in the current MegaTag1 pose estimate.
     *
     * @return number of tags contributing to the current pose estimate, or {@code 0} if not
     *     attached or no estimate is available
     */
    public double getTagCountInView() {
        if (!isAttached()) {
            return 0;
        }
        return mt1Estimate().tagCount;
    }

    /**
     * Gets the ID of the apriltag most centered in the LL's view (or based on different
     *
     * @return the tag ID of the apriltag most centered in the LL's view (or based on different
     *     criteria set in LL dasbhoard)
     */
    public double getClosestTagID() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getFiducialID(config.getName());
    }

    /**
     * Returns the area of the primary target as a percentage of the camera image (0–100).
     *
     * @return target area percentage, or {@code 0} if not attached
     */
    public double getTargetSize() {
        if (!isAttached()) {
            return 0;
        }
        if (!taCached) {
            taValue = LimelightHelpers.getTA(config.getName());
            taCached = true;
        }
        return taValue;
    }

    /* ::: Pose Retrieval ::: */

    /**
     * Get the corresponding LL Pose3d (MEGATAG1) for the alliance in DriverStation.java
     *
     * @return the corresponding LL Pose3d (MEGATAG1) for the alliance in DriverStation.java
     */
    public Pose3d getMegaTag1_Pose3d() {
        if (!isAttached()) {
            return Pose3d.kZero;
        }
        return mt1Pose3d();
    }

    /**
     * Get the corresponding LL Pose2d (MEGATAG1) for the alliance in DriverStation.java
     *
     * @return the corresponding LL Pose3d (MEGATAG2) for the alliance in DriverStation.java
     */
    public Pose2d getMegaTag2_Pose2d() {
        if (!isAttached()) {
            return Pose2d.kZero;
        }
        return mt2Estimate().pose;
    }

    /**
     * Returns the full MegaTag1 {@link PoseEstimate}, including timestamp, tag count, and raw
     * fiducials. Returns an empty estimate when not attached or when no estimate is available.
     *
     * @return the MegaTag1 pose estimate in the WPILib Blue origin frame
     */
    public PoseEstimate getMegaTag1_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }
        return mt1Estimate();
    }

    /**
     * Returns the full MegaTag2 {@link PoseEstimate} (heading-fused), including timestamp and tag
     * count. Returns an empty estimate when not attached or when no estimate is available.
     *
     * @return the MegaTag2 pose estimate in the WPILib Blue origin frame
     */
    public PoseEstimate getMegaTag2_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }
        return mt2Estimate();
    }

    /**
     * Returns {@code true} when the pose estimate is considered accurate — i.e., multiple tags are
     * visible and the combined target area exceeds a minimum threshold.
     *
     * @return {@code true} if the pose estimate meets the accuracy criteria
     */
    public boolean hasAccuratePose() {
        if (!isAttached()) {
            return false;
        }
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    /**
     * Get the distance of the 2d vector from the camera to closest apriltag
     *
     * @return the distance of the 2d vector from the camera to closest apriltag
     */
    public double getDistanceToTagFromCamera() {
        if (!isAttached()) {
            return 0;
        }
        Pose3d cameraInTargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(config.name);
        return Math.hypot(cameraInTargetSpace.getX(), cameraInTargetSpace.getZ());
    }

    /**
     * Returns the raw fiducial data for all currently detected AprilTags.
     *
     * @return array of {@link RawFiducial} entries from the MegaTag1 estimate; empty array if not
     *     attached or no estimate is available
     */
    public RawFiducial[] getRawFiducial() {
        if (!isAttached()) {
            return new RawFiducial[0];
        }
        // Same per-loop snapshot as every other MegaTag1 getter; this used to do a second full
        // NetworkTables read and parse of botpose_wpiblue on each call.
        RawFiducial[] fiducials = mt1Estimate().rawFiducials;
        return fiducials == null ? new RawFiducial[0] : fiducials;
    }

    /**
     * Returns the timestamp of the MEGATAG1 pose estimation from the Limelight camera.
     *
     * @return The timestamp of the pose estimation in seconds.
     */
    public double getMegaTag1PoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        return mt1Estimate().timestampSeconds;
    }

    /**
     * Returns the timestamp of the MEGATAG2 pose estimation from the Limelight camera.
     *
     * @return The timestamp of the pose estimation in seconds.
     */
    public double getMegaTag2PoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        return mt2Estimate().timestampSeconds;
    }

    /**
     * Returns the latency of the pose estimation from the Limelight camera.
     *
     * @return The latency of the pose estimation in seconds.
     */
    @Deprecated(forRemoval = true)
    public double getPoseLatency() {
        if (!isAttached()) {
            return 0;
        }
        return Units.millisecondsToSeconds(
                LimelightHelpers.getBotPose_wpiBlue(config.getName())[6]);
    }

    /*
     * Custom Helpers
     */

    /**
     * get distance in meters to a target
     *
     * @param targetHeight meters
     * @return distance in meters to a target
     */
    public double getDistanceToTarget(double targetHeight) {
        if (!isAttached()) {
            return 0;
        }
        return (targetHeight - config.up)
                / Math.tan(Units.degreesToRadians(config.pitch + getVerticalOffset()));
    }

    /**
     * Marks this camera as actively integrating and updates the log status message.
     *
     * @param message a human-readable description of why integration is valid
     */
    public void sendValidStatus(String message) {
        config.isIntegrating = true;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    /**
     * Marks this camera as not integrating and updates the log status message.
     *
     * @param message a human-readable description of why integration is invalid
     */
    public void sendInvalidStatus(String message) {
        config.isIntegrating = false;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    /*
     * Utility Wrappers
     */

    /**
     * @return The latest LL results as a LimelightResults object.
     */
    @SuppressWarnings("unused")
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(config.name);
    }

    /**
     * Sets the LL pipeline to the given index.
     *
     * @param pipelineIndex the pipeline index (refer to robot vision configuration for pipeline
     *     indexes)
     */
    public void setLimelightPipeline(int pipelineIndex) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setPipelineIndex(config.name, pipelineIndex);
    }

    /**
     * Sets the robot orientation in degrees for the Limelight's internal IMU.
     *
     * <p>Does not flush NetworkTables; the caller is responsible for a single flush after all
     * per-loop writes (see {@code Vision.periodic()}).
     */
    public void setRobotOrientation(double degrees) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation_NoFlush(config.name, degrees, 0, 0, 0, 0, 0);
    }

    public void updateCameraPose(Pose3d pose) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setCameraPose_RobotSpace(
                config.name,
                pose.getX(),
                -pose.getY(),
                pose.getZ(),
                Units.radiansToDegrees(pose.getRotation().getX()),
                Units.radiansToDegrees(pose.getRotation().getY()),
                Units.radiansToDegrees(pose.getRotation().getZ()));
    }

    /**
     * Writes this camera's configured mount pose ({@link LimelightConfig#withTranslation} and
     * {@link LimelightConfig#withRotation}) to the camera over NetworkTables, making the code the
     * source of truth for the offsets instead of the values typed into the web UI.
     *
     * <p>The six values are stored in exactly the convention the camera wants -- forward, right, up
     * in metres, then roll, pitch, yaw in degrees -- so they go across unchanged. That is the same
     * order and sign as the boxes in the web UI, so what is in {@code VisionConfig} can be read
     * straight off against what a camera is showing.
     *
     * <p>A Limelight keeps its own copy in flash and falls back to it on boot, so this has to be
     * republished rather than written once: a camera that reboots mid-match otherwise silently
     * reverts to whatever was last typed into it. See the resend loop in {@code Vision.periodic()}.
     *
     * <p>Does not flush NetworkTables; the caller flushes once after all per-loop writes.
     */
    public void pushConfiguredCameraPose() {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setCameraPose_RobotSpace(
                config.name,
                config.forward,
                config.right,
                config.up,
                config.roll,
                config.pitch,
                config.yaw);
    }

    /**
     * Sets the robot orientation and yaw rate for the Limelight's internal IMU fusion (MegaTag2).
     *
     * <p>Does not flush NetworkTables; the caller is responsible for a single flush after all
     * per-loop writes (see {@code Vision.periodic()}).
     *
     * @param degrees robot heading in degrees (positive counter-clockwise)
     * @param angularRate current yaw rate in degrees per second
     */
    public void setRobotOrientation(double degrees, double angularRate) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation_NoFlush(config.name, degrees, angularRate, 0, 0, 0, 0);
    }

    /**
     * Sets the IMU mode on the Limelight.
     *
     * @param mode the IMU mode index (refer to the LL documentation for valid values)
     */
    public void setIMUmode(int mode) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetIMUMode(config.name, mode);
    }

    /**
     * Returns the X offset of the primary target in robot space (meters along the robot's
     * left/right axis).
     *
     * @return target X translation in meters, or {@code -99999} if not attached or no target in
     *     view
     */
    public double getTagTx() {
        if (!isAttached()) {
            return -99999;
        }

        if (!targetInView()) {
            return -99999;
        }

        double tx = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getX();

        return tx;
    }

    /**
     * Returns the area of the primary target as a percentage of the camera image.
     *
     * @return target area (0–100 %), or {@code -99999} if not attached or no target in view
     */
    public double getTagTA() {
        if (!isAttached()) {
            return -99999;
        }
        if (!targetInView()) {
            return -99999;
        }

        return getTargetSize();
    }

    /**
     * Returns the Z-axis rotation of the primary target in robot space (yaw in radians, converted
     * to degrees).
     *
     * @return target yaw rotation in degrees, or {@code -99999} if not attached or no target in
     *     view
     */
    public double getTagRotationDegrees() {
        if (!isAttached()) {
            return -99999;
        }
        if (!targetInView()) {
            return -99999;
        }

        double rotationRadians =
                LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getRotation().getZ();

        return Math.toDegrees(rotationRadians);
    }

    /**
     * Sets the LED mode of the LL.
     *
     * @param enabled true to enable the LED mode, false to disable it
     */
    public void setLEDMode(boolean enabled) {
        if (!isAttached()) {
            return;
        }
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(config.getName());
        } else {
            LimelightHelpers.setLEDMode_ForceOff(config.getName());
        }
    }

    /** Set LL LED's to blink */
    public void blinkLEDs() {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setLEDMode_ForceBlink(config.getName());
    }

    /** Checks if the camera is connected by looking for an empty botpose array from camera. */
    public boolean isCameraConnected() {
        if (!isAttached()) {
            return false;
        }
        try {
            return mt1Sample().value.length >= 6;
        } catch (Exception e) {
            System.err.println("Avoided crashing statement in Limelight.java: isCameraConnected()");
            return false;
        }
    }

    /** Prints the vision, estimated, and odometry pose to SmartDashboard */
    public void printDebug() {
        if (!isAttached()) {
            return;
        }
        Pose3d botPose3d = getMegaTag1_Pose3d();
        SmartDashboard.putString("LimelightX", df.format(botPose3d.getTranslation().getX()));
        SmartDashboard.putString("LimelightY", df.format(botPose3d.getTranslation().getY()));
        SmartDashboard.putString("LimelightZ", df.format(botPose3d.getTranslation().getZ()));
        SmartDashboard.putString(
                "LimelightRoll", df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
        SmartDashboard.putString(
                "LimelightPitch",
                df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
        SmartDashboard.putString(
                "LimelightYaw", df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
    }
}
