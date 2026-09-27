package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Logs telemetry data from a {@link Limelight} camera to the robot's data-logging system (DogLog)
 * under the {@code Vision/<name>/} namespace.
 *
 * <p>Each method reads the corresponding value from the camera, forwards it to {@link
 * frc.spectrumLib.telemetry.Telemetry#log}, and also returns the value so callers can use it
 * directly without a second camera query.
 *
 * <p>Log keys are built once in the constructor so per-loop logging allocates no strings.
 */
public class VisionLogger {
    /** The Limelight camera whose data is being logged. */
    private final Limelight limelight;

    /** Namespace prefix used in all telemetry keys ({@code Vision/<name>/...}). */
    @Getter private final String name;

    private final String connectionKey;
    private final String integratingKey;
    private final String logStatusKey;
    private final String tagStatusKey;
    private final String mt1PoseKey;
    private final String mt2PoseKey;
    private final String tagCountKey;
    private final String targetSizeKey;
    private final String estimateAgeKey;
    private final String integratedKey;
    private final String mountTiltKey;
    private final String mountPitchKey;
    private final String mountRollKey;
    private final String mountHeightKey;

    /**
     * Constructs a logger for the given Limelight camera.
     *
     * @param name the namespace prefix used in telemetry keys
     * @param limelight the camera to read from
     */
    public VisionLogger(String name, Limelight limelight) {
        this.limelight = limelight;
        this.name = name;
        String prefix = "Vision/" + name + "/";
        connectionKey = prefix + "ConnectionStatus";
        integratingKey = prefix + "IntegratingStatus";
        logStatusKey = prefix + "LogStatus";
        tagStatusKey = prefix + "TagStatus";
        mt1PoseKey = prefix + "MT1Pose";
        mt2PoseKey = prefix + "MT2Pose";
        tagCountKey = prefix + "TagCount";
        targetSizeKey = prefix + "TargetSize";
        estimateAgeKey = prefix + "EstimateAgeSeconds";
        integratedKey = prefix + "IntegratedThisLoop";
        mountTiltKey = prefix + "MountCheck/TiltDeg";
        mountPitchKey = prefix + "MountCheck/PitchDeg";
        mountRollKey = prefix + "MountCheck/RollDeg";
        mountHeightKey = prefix + "MountCheck/HeightMeters";
    }

    /**
     * Logs and returns the age of the estimate this camera produced this loop, in seconds, or NaN
     * if it produced none.
     *
     * @return estimate age in seconds
     */
    public double getEstimateAge() {
        double age = limelight.getLastEstimateAgeSeconds();
        // On the dashboard (Elastic watches the turret camera's age).
        Telemetry.logDash(estimateAgeKey, age, "seconds");
        return age;
    }

    /**
     * Logs and returns whether an estimate from this camera was actually fused into the pose
     * estimator this loop, as opposed to merely passing the acceptance tiers.
     *
     * @return {@code true} if fused this loop
     */
    public boolean getIntegratedThisLoop() {
        boolean integrated = limelight.isIntegratedThisLoop();
        Telemetry.log(integratedKey, integrated);
        return integrated;
    }

    /**
     * Logs and returns whether the camera is currently connected.
     *
     * @return {@code true} if the camera is reachable over the network
     */
    public boolean getCameraConnection() {
        boolean connected = limelight.isCameraConnected();
        // On the dashboard: one indicator per camera.
        Telemetry.logDash(connectionKey, connected);
        return connected;
    }

    /**
     * Logs and returns whether pose measurements are currently being fused into the estimator.
     *
     * @return {@code true} if the camera is actively integrating
     */
    public boolean getIntegratingStatus() {
        boolean integrating = limelight.isIntegrating();
        Telemetry.log(integratingKey, integrating);
        return integrating;
    }

    /**
     * Logs and returns the camera's human-readable integration status message.
     *
     * @return the current log status string from the camera
     */
    public String getLogStatus() {
        String status = limelight.getLogStatus();
        Telemetry.log(logStatusKey, status);
        return status;
    }

    /**
     * Logs and returns the camera's human-readable tag-detection status message.
     *
     * @return the current tag status string from the camera
     */
    public String getTagStatus() {
        String status = limelight.getTagStatus();
        Telemetry.log(tagStatusKey, status);
        return status;
    }

    /**
     * Logs and returns the robot's 2-D pose derived from the MegaTag1 estimate.
     *
     * @return the MegaTag1 pose projected to 2-D in the WPILib Blue origin frame
     */
    public Pose2d getPose() {
        Pose2d pose = limelight.getMegaTag1_Pose3d().toPose2d();
        Telemetry.log(mt1PoseKey, pose);
        return pose;
    }

    /**
     * Logs and returns the robot's 2-D pose from the MegaTag2 (heading-fused) estimate.
     *
     * @return the MegaTag2 {@link Pose2d} in the WPILib Blue origin frame
     */
    public Pose2d getMegaPose() {
        Pose2d pose = limelight.getMegaTag2_Pose2d();
        Telemetry.log(mt2PoseKey, pose);
        return pose;
    }

    /**
     * Logs the out-of-plane part of the MegaTag1 estimate, for checking the camera's mount angles
     * against the values entered in its web UI.
     *
     * <p>MegaTag1 reports the <b>robot's</b> full 3-D pose: it solves the camera's pose from the
     * tag, then applies the camera-to-robot transform from the camera's entered offsets. With the
     * robot sitting flat on the carpet the truth is height 0, pitch 0, roll 0, so whatever these
     * read is the error in the entered mount rotation. {@code TiltDeg} is the total angle between
     * the reported robot Z axis and field up: it equals the magnitude of the mount rotation error
     * regardless of the camera's yaw, so it is the one number to read. A camera entered at 60 deg
     * that is really mounted at 55 shows about 5 deg of tilt.
     *
     * <p>{@code PitchDeg} and {@code RollDeg} say which way, but they are in the <b>robot</b>
     * frame, not the camera's: a pitch error of d on a camera yawed at psi lands as roll
     * -d*sin(psi) and pitch d*cos(psi). For the rear cameras at yaw +/-135 that splits evenly
     * between the two, and for the turret camera the split rotates with the turret — another reason
     * to read TiltDeg.
     *
     * <p>Read these with the robot stationary on a flat floor and at least one tag in view; real
     * chassis tilt (an obstacle, or weight transfer under acceleration) shows up here too. Nothing
     * is logged when no tag is in view, so the values do not fall to zero between sightings.
     *
     * <p>Free to call: the MegaTag1 sample is read from NetworkTables once per loop and cached, so
     * this adds no camera query beyond {@link #getPose()}.
     */
    public void logMountCheck() {
        if (limelight.getTagCountInView() < 1) {
            return;
        }
        Pose3d pose = limelight.getMegaTag1_Pose3d();
        double pitch = pose.getRotation().getY();
        double roll = pose.getRotation().getX();
        Telemetry.log(mountTiltKey, Units.radiansToDegrees(Math.hypot(pitch, roll)), "deg");
        Telemetry.log(mountPitchKey, Units.radiansToDegrees(pitch), "deg");
        Telemetry.log(mountRollKey, Units.radiansToDegrees(roll), "deg");
        Telemetry.log(mountHeightKey, pose.getZ(), "meters");
    }

    /**
     * Logs and returns the number of AprilTags contributing to the current pose estimate.
     *
     * @return the tag count from the MegaTag1 estimate
     */
    public double getTagCount() {
        double count = limelight.getTagCountInView();
        Telemetry.log(tagCountKey, count);
        return count;
    }

    /**
     * Logs and returns the area of the primary target as a percentage of the camera image.
     *
     * @return target area (0–100 %)
     */
    public double getTargetSize() {
        double size = limelight.getTargetSize();
        Telemetry.log(targetSizeKey, size);
        return size;
    }
}
