package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.List;
import java.util.Optional;

/**
 * Robot pose measured against one reef tag, for the last half metre of a reef alignment.
 *
 * <p>Why not the fused field pose: that pose carries every error in the tag map, the camera mounts
 * and the other tags the cameras happened to see. Scoring only needs to be right relative to the
 * branch, and the branch is fixed relative to its own tag, so this measures the robot from that one
 * tag and nothing else. Errors in where the tag "is" on the field cancel, because the goal pose is
 * built from the same tag pose.
 *
 * <p>How (6328's tx/ty observation, issue 8c): heading comes from the gyro, which is steadier than
 * any single-tag solve; the direction to the tag comes from the tag's angles in the image; the
 * range comes from the Limelight's own distance to the tag. That avoids the single-tag PnP
 * ambiguity entirely. The estimate is taken at the frame's capture time and carried to now with
 * odometry, and survives short dropouts (a bumper on the reef hiding the tag) for {@link
 * #MAX_AGE_SECONDS}.
 *
 * <p>Both cameras feed it; whichever sees the tag counts, weighted by range and by how close the
 * tag is to the image edge (issue 8b: trust edge tags less, don't drop them).
 */
public class ReefTagTracker {

    /** One tag in one frame. Angles in degrees from the image centre, LL convention. */
    public record Sighting(
            int id, double txDeg, double tyDeg, double distanceMeters, double fpgaTimestamp) {}

    /** A camera and where it is on the robot. */
    public record Camera(String name, Transform3d robotToCamera) {}

    /** How long a sighting keeps the estimate alive through a dropout. */
    public static final double MAX_AGE_SECONDS = 0.3;

    /** Sightings past this range are not used; angle error grows with range. */
    private static final double MAX_RANGE_METERS = 3.0;

    /** Half the Limelight 4 field of view, degrees (82 x 56.2). */
    private static final double HALF_HFOV = 41.0;

    private static final double HALF_VFOV = 28.1;

    /** Edge falloff band and the std-dev multiplier at the very edge (issue 8b dials). */
    private static final double EDGE_BAND_DEG = 6.0;

    private static final double EDGE_MAX_SCALE = 4.0;

    private static final class CameraEstimate {
        Pose2d atCapture;
        Pose2d odometryAtCapture;
        double captureFpga = Double.NEGATIVE_INFINITY;
        double weight;
    }

    private final Camera[] cameras;
    private final CameraEstimate[] estimates;
    private int targetTag = -1;

    public ReefTagTracker(Camera... cameras) {
        this.cameras = cameras;
        estimates = new CameraEstimate[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            estimates[i] = new CameraEstimate();
        }
    }

    /** Selects the tag to track; a change forgets everything seen for the old one. */
    public void setTargetTag(int tagId) {
        if (tagId != targetTag) {
            targetTag = tagId;
            for (CameraEstimate e : estimates) {
                e.captureFpga = Double.NEGATIVE_INFINITY;
            }
        }
    }

    public int getTargetTag() {
        return targetTag;
    }

    /** Feeds one camera's sightings for this loop. */
    public void addSightings(int cameraIndex, List<Sighting> sightings) {
        if (targetTag < 0) {
            return;
        }
        for (Sighting s : sightings) {
            if (s.id() != targetTag || s.fpgaTimestamp() <= estimates[cameraIndex].captureFpga) {
                continue;
            }
            Translation2d robotToTag = robotToTag(cameras[cameraIndex].robotToCamera(), s);
            if (robotToTag.getNorm() > MAX_RANGE_METERS) {
                continue;
            }
            Pose2d odometry = Robot.getSwerve().getPoseAtTimestamp(s.fpgaTimestamp());
            Rotation2d heading = odometry.getRotation();
            Translation2d robot =
                    Field.tagPose(targetTag).getTranslation().minus(robotToTag.rotateBy(heading));
            CameraEstimate e = estimates[cameraIndex];
            e.atCapture = new Pose2d(robot, heading);
            e.odometryAtCapture = odometry;
            e.captureFpga = s.fpgaTimestamp();
            double scale = edgeScale(s.txDeg(), s.tyDeg());
            e.weight =
                    1.0 / Math.max(1e-3, s.distanceMeters() * s.distanceMeters() * scale * scale);
            Telemetry.log("Vision/ReefTag/" + cameras[cameraIndex].name() + "/EdgeScale", scale);
        }
    }

    /**
     * Where the tag is from the robot centre, on the floor plane, in robot axes.
     *
     * <p>The ray to the tag in the camera frame is (1, -tan tx, tan ty) scaled to the measured
     * range: tx is positive to the right, which is -y. The mount transform takes it to the robot.
     */
    static Translation2d robotToTag(Transform3d robotToCamera, Sighting s) {
        Translation3d ray =
                new Translation3d(
                        1,
                        -Math.tan(Math.toRadians(s.txDeg())),
                        Math.tan(Math.toRadians(s.tyDeg())));
        Translation3d cameraToTag = ray.times(s.distanceMeters() / ray.getNorm());
        Translation3d robotToTag =
                robotToCamera
                        .getTranslation()
                        .plus(cameraToTag.rotateBy(robotToCamera.getRotation()));
        return robotToTag.toTranslation2d();
    }

    /** 1 away from the image edge, rising smoothly to {@link #EDGE_MAX_SCALE} at it. */
    static double edgeScale(double txDeg, double tyDeg) {
        double edgeDeg = Math.min(HALF_HFOV - Math.abs(txDeg), HALF_VFOV - Math.abs(tyDeg));
        double u = Math.max(0.0, 1.0 - edgeDeg / EDGE_BAND_DEG);
        return 1.0 + (EDGE_MAX_SCALE - 1.0) * u * u;
    }

    public boolean isFresh() {
        return getPose().isPresent();
    }

    /**
     * The tag-relative robot pose now, or empty when no camera has seen the target tag within
     * {@link #MAX_AGE_SECONDS}.
     */
    public Optional<Pose2d> getPose() {
        double now = Timer.getFPGATimestamp();
        Pose2d odometryNow = Robot.getSwerve().getRobotPose();
        double x = 0;
        double y = 0;
        double weightSum = 0;
        for (CameraEstimate e : estimates) {
            if (now - e.captureFpga > MAX_AGE_SECONDS) {
                continue;
            }
            // Carry the capture-time estimate forward by how far odometry says we moved since.
            Pose2d now2d = e.atCapture.plus(odometryNow.minus(e.odometryAtCapture));
            x += now2d.getX() * e.weight;
            y += now2d.getY() * e.weight;
            weightSum += e.weight;
        }
        if (weightSum == 0) {
            return Optional.empty();
        }
        return Optional.of(new Pose2d(x / weightSum, y / weightSum, odometryNow.getRotation()));
    }

    public void log() {
        Optional<Pose2d> pose = getPose();
        Telemetry.log("Vision/ReefTag/Target", targetTag);
        Telemetry.log("Vision/ReefTag/Fresh", pose.isPresent());
        pose.ifPresent(p -> Telemetry.log("Vision/ReefTag/Pose", p));
    }
}
