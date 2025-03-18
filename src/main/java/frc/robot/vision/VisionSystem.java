// See: https://docs.photonvision.org/en/latest/docs/simulation/simulation.html
package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSystem extends SubsystemBase {
    @SuppressWarnings("unused")
    private final PhotonCamera camera = new PhotonCamera("cameraName");
    // private final PhotonCamera frontCam = new PhotonCamera(VisionConfig.FRONT_LL);
    // private final PhotonCamera backCam = new PhotonCamera(VisionConfig.RIGHT_LL);
    private final VisionSystemSim visionSim = new VisionSystemSim("main");
    private final Pose2dSupplier getSimPose;

    Transform3d robotToFrontCamera =
            new Transform3d(
                    new Translation3d(0, 0, 0.5), // Centered on the robot, 0.5m up
                    new Rotation3d(0, Math.toRadians(-15), 0)); // Pitched 15 deg up
    Transform3d robotToBackCamera =
            new Transform3d(
                    new Translation3d(0, 0, 0.5), // Centered on the robot, 0.5m up
                    new Rotation3d(0, Math.toRadians(-15), 0)); // Pitched 15 deg up

    @FunctionalInterface
    public interface Pose2dSupplier {
        Pose2d getPose2d();
    }

    public VisionSystem(Pose2dSupplier getSimPose) {
        this.getSimPose = getSimPose;

        // Setup simulated camera properties
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibError(0.25, 0.08);
        props.setFPS(20.0);
        props.setAvgLatencyMs(35.0);
        props.setLatencyStdDevMs(5.0);

        // Setup simulated camera
        // PhotonCameraSim cameraSimFront = new PhotonCameraSim(frontCam, props);
        // PhotonCameraSim cameraSimBack = new PhotonCameraSim(backCam, props);
        // Draw field wireframe in simulated camera view
        // cameraSimFront.enableDrawWireframe(true);
        // cameraSimBack.enableDrawWireframe(false);

        // // Add simulated camera to vision sim
        // visionSim.addCamera(cameraSimFront, robotToFrontCamera);
        // visionSim.addCamera(cameraSimBack, robotToBackCamera);

        // Add AprilTags to vision sim
        try {
            AprilTagFieldLayout tagLayout =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
            visionSim.addAprilTags(tagLayout);
        } catch (IOException e) {
            System.err.println(e);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update the vision system with the simulated robot pose
        visionSim.update(getSimPose.getPose2d());
    }
}
