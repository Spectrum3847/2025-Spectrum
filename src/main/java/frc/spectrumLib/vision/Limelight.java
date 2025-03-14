package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.Vision.VisionConfig;
import frc.spectrumLib.vision.LimelightHelpers.LimelightResults;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import lombok.Getter;
import lombok.Setter;

public class Limelight {

    /* Limelight Configuration */

    public static class LimelightConfig {
        /** Must match to the name given in LL dashboard */
        @Getter @Setter private String name;

        @Getter @Setter private boolean attached = true;

        @Getter @Setter private boolean isIntegrating;
        /** Physical Config */
        @Getter private double forward, right, up; // meters

        @Getter private double roll, pitch, yaw; // degrees

        public LimelightConfig(String name) {
            this.name = name;
        }

        /**
         * @param forward (meters) forward from center of robot
         * @param right (meters) right from center of robot
         * @param up (meters) up from center of robot
         * @return
         */
        public LimelightConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /**
         * @param roll (degrees) roll of limelight || positive is rotated right
         * @param pitch (degrees) pitch of limelight || positive is camera tilted up
         * @param yaw (yaw) yaw of limelight || positive is rotated left
         * @return
         */
        public LimelightConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }
    }

    /* Debug */
    private final DecimalFormat df = new DecimalFormat();
    private LimelightConfig config;
    @Getter @Setter private boolean isIntegrating = false;
    @Getter private String cameraName = "default";
    @Getter @Setter private String logStatus = "";
    @Getter @Setter private String tagStatus = "";

    public Limelight(LimelightConfig config) {
        this.config = config;
    }

    public Limelight(String name) {
        cameraName = name;
        config = new LimelightConfig(name);
    }

    public Limelight(String name, boolean attached) {
        cameraName = name;
        config = new LimelightConfig(name).setAttached(attached);
    }

    public Limelight(String name, int pipeline) {
        this(name);
        cameraName = name;
        setLimelightPipeline(pipeline);
    }

    public Limelight(String name, int pipeline, LimelightConfig config) {
        this(name);
        cameraName = name;
        this.config = config;
        setLimelightPipeline(pipeline);
    }

    public String getName() {
        return config.getName();
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    /* ::: Basic Information Retrieval ::: */
    /**
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
     * @return Vertical Offset From Crosshair To Target in degrees (LL1: -20.5 degrees to 20.5
     *     degrees / LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTY(config.getName());
    }

    /** @return Whether the LL has any valid targets (April tags or other vision targets) */
    public boolean targetInView() {
        if (!isAttached()) {
            return false;
        }
        return LimelightHelpers.getTV(config.getName());
    }

    /** @return whether the LL sees multiple tags or not */
    public boolean multipleTagsInView() {
        if (!isAttached()) {
            return false;
        }
        return getTagCountInView() > 1;
    }

    public double getTagCountInView() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName()).tagCount;

        // if (retrieveJSON() == null) return 0;

        // return retrieveJSON().targetingResults.targets_Fiducials.length;
    }

    /**
     * @return the tag ID of the apriltag most centered in the LL's view (or based on different
     *     criteria set in LL dasbhoard)
     */
    public double getClosestTagID() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getFiducialID(config.getName());
    }

    public double getTargetSize() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTA(config.getName());
    }

    /* ::: Pose Retrieval ::: */

    /** @return the corresponding LL Pose3d (MEGATAG1) for the alliance in DriverStation.java */
    public Pose3d getMegaTag1_Pose3d() {
        if (!isAttached()) {
            return new Pose3d();
        }
        Pose3d pose3d = LimelightHelpers.getBotPose3d_wpiBlue(config.name);
        if (pose3d == null) {
            return new Pose3d();
        }
        return pose3d;
    }

    /** @return the corresponding LL Pose3d (MEGATAG2) for the alliance in DriverStation.java */
    public Pose2d getMegaTag2_Pose2d() {
        if (!isAttached()) {
            return new Pose2d();
        }
        PoseEstimate poseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
        if (poseEstimate == null) {
            return new Pose2d();
        }
        return poseEstimate.pose;
    }

    public PoseEstimate getMegaTag1_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }

        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
        if (poseEstimate == null) {
            return new PoseEstimate();
        }
        return poseEstimate;
    }

    public PoseEstimate getMegaTag2_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }

        PoseEstimate poseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
        if (poseEstimate == null) {
            return new PoseEstimate();
        }
        return poseEstimate;
    }

    public boolean hasAccuratePose() {
        if (!isAttached()) {
            return false;
        }
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    /** @return the distance of the 2d vector from the camera to closest apriltag */
    public double getDistanceToTagFromCamera() {
        if (!isAttached()) {
            return 0;
        }
        double x = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getX();
        double y = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getZ();
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public RawFiducial[] getRawFiducial() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name).rawFiducials;
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
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName()).timestampSeconds;
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
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.getName())
                .timestampSeconds;
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
     * @return
     */
    public double getDistanceToTarget(double targetHeight) {
        if (!isAttached()) {
            return 0;
        }
        return (targetHeight - config.up)
                / Math.tan(Units.degreesToRadians(config.roll + getVerticalOffset()));
    }

    public void sendValidStatus(String message) {
        config.isIntegrating = true;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    public void sendInvalidStatus(String message) {
        config.isIntegrating = false;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    /*
     * Utility Wrappers
     */

    /** @return The latest LL results as a LimelightResults object. */
    @SuppressWarnings("unused")
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(config.name);
    }

    /** @param pipelineIndex use pipeline indexes in {@link VisionConfig} */
    public void setLimelightPipeline(int pipelineIndex) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setPipelineIndex(config.name, pipelineIndex);
    }

    /** */
    public void setRobotOrientation(double degrees) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, 0, 0, 0, 0, 0);
    }

    public void setRobotOrientation(double degrees, double angularRate) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, angularRate, 0, 0, 0, 0);
    }

    public void setIMUmode(int mode) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetIMUMode(config.name, mode);
    }

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

    public double getTagTA() {
        if (!isAttached()) {
            return -99999;
        }
        if (!targetInView()) {
            return -99999;
        }

        double ta = LimelightHelpers.getTA(cameraName);

        return ta;
    }

    public double getTagRotationDegrees() {
        if (!isAttached()) {
            return -99999;
        }
        if (!targetInView()) {
            return -99999;
        }

        double rotation =
                LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getRotation().getZ();

        return rotation;
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

    /**
     * Set LL LED's to blink
     *
     * @return
     */
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
            var rawPoseArray =
                    LimelightHelpers.getLimelightNTTableEntry(config.getName(), "botpose_wpiblue")
                            .getDoubleArray(new double[0]);
            if (rawPoseArray.length < 6) {
                return false;
            }
            return true;
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
