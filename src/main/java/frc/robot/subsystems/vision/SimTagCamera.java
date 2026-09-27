package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.robot.subsystems.vision.ReefTagTracker.Sighting;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Simulated Limelight tag sightings, built from the simulator's true robot pose so reef alignment
 * can be exercised with no robot. Adds angle and range noise and a fixed capture latency.
 */
final class SimTagCamera {
    private SimTagCamera() {}

    private static final double HALF_HFOV = 40.0;
    private static final double HALF_VFOV = 27.0;
    private static final double MAX_RANGE = 4.0;
    /** Tags seen more obliquely than this are not detected. */
    private static final double MAX_VIEW_ANGLE_DEG = 70.0;

    private static final double LATENCY_SECONDS = 0.03;
    private static final double ANGLE_NOISE_DEG = 0.15;
    private static final double RANGE_NOISE_FRACTION = 0.01;
    private static final Random random = new Random(3847);

    static List<Sighting> sightings(Transform3d robotToCamera) {
        List<Sighting> out = new ArrayList<>();
        var sim = Robot.getSwerve().getMapleSim();
        if (sim == null) {
            return out;
        }
        Pose3d camera =
                new Pose3d(sim.mapleSimDrive.getSimulatedDriveTrainPose())
                        .transformBy(robotToCamera);
        double timestamp = Timer.getFPGATimestamp() - LATENCY_SECONDS;
        for (AprilTag tag : Field.TAG_LAYOUT.getTags()) {
            if (!Field.isReefTag(tag.ID)) {
                continue;
            }
            Translation3d inCamera = tag.pose.relativeTo(camera).getTranslation();
            double x = inCamera.getX();
            if (x <= 0) {
                continue;
            }
            double txDeg = -Math.toDegrees(Math.atan2(inCamera.getY(), x));
            double tyDeg = Math.toDegrees(Math.atan2(inCamera.getZ(), x));
            double range = inCamera.getNorm();
            if (Math.abs(txDeg) > HALF_HFOV || Math.abs(tyDeg) > HALF_VFOV || range > MAX_RANGE) {
                continue;
            }
            // The tag's +x points out of its face; the camera must be in front of it.
            Translation3d tagToCamera = camera.getTranslation().minus(tag.pose.getTranslation());
            Translation3d normal = new Translation3d(1, 0, 0).rotateBy(tag.pose.getRotation());
            double cos =
                    (normal.getX() * tagToCamera.getX()
                                    + normal.getY() * tagToCamera.getY()
                                    + normal.getZ() * tagToCamera.getZ())
                            / tagToCamera.getNorm();
            if (cos < Math.cos(Math.toRadians(MAX_VIEW_ANGLE_DEG))) {
                continue;
            }
            out.add(
                    new Sighting(
                            tag.ID,
                            txDeg + random.nextGaussian() * ANGLE_NOISE_DEG,
                            tyDeg + random.nextGaussian() * ANGLE_NOISE_DEG,
                            range * (1 + random.nextGaussian() * RANGE_NOISE_FRACTION),
                            timestamp));
        }
        return out;
    }
}
