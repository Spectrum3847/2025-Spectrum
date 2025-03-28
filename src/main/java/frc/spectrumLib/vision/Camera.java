package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import frc.spectrumLib.vision.PhotonSimCamera.PhotonSimCameraConfig;

public class Camera {
    private Limelight limelight;
    private PhotonSimCamera photonSimCamera;
    private boolean usePhotonVision = Robot.isSimulation();

    public Camera(String name) {
        if (usePhotonVision) {
            photonSimCamera = new PhotonSimCamera(name);
        } else {
            limelight = new Limelight(name);
        }
    }

    public Camera(String name, boolean attached) {
        if (usePhotonVision) {
            photonSimCamera = new PhotonSimCamera(name, attached);
        } else {
            limelight = new Limelight(name, attached);
        }
    }

    public Camera(String name, int pipeline) {
        if (usePhotonVision) {
            photonSimCamera = new PhotonSimCamera(name, pipeline);
        } else {
            limelight = new Limelight(name, pipeline);
        }
    }

    public Camera(String name, int pipeline, LimelightConfig config) {
        limelight = new Limelight(name, pipeline, config);
    }

    public Camera(String name, int pipeline, PhotonSimCameraConfig config) {
        photonSimCamera = new PhotonSimCamera(name, pipeline, config);
    }

    public String getName() {
        return usePhotonVision ? photonSimCamera.getName() : limelight.getName();
    }

    public boolean isAttached() {
        return usePhotonVision ? photonSimCamera.isAttached() : limelight.isAttached();
    }

    public double getHorizontalOffset() {
        return usePhotonVision
                ? photonSimCamera.getHorizontalOffset()
                : limelight.getHorizontalOffset();
    }

    public double getVerticalOffset() {
        return usePhotonVision
                ? photonSimCamera.getVerticalOffset()
                : limelight.getVerticalOffset();
    }

    public boolean targetInView() {
        return usePhotonVision ? photonSimCamera.targetInView() : limelight.targetInView();
    }

    public boolean multipleTagsInView() {
        return usePhotonVision
                ? photonSimCamera.multipleTagsInView()
                : limelight.multipleTagsInView();
    }

    public double getTagCountInView() {
        return usePhotonVision
                ? photonSimCamera.getTagCountInView()
                : limelight.getTagCountInView();
    }

    public double getClosestTagID() {
        return usePhotonVision ? photonSimCamera.getClosestTagID() : limelight.getClosestTagID();
    }

    public double getTargetSize() {
        return usePhotonVision ? photonSimCamera.getTargetSize() : limelight.getTargetSize();
    }

    public Pose3d getMegaTag1_Pose3d() {
        return usePhotonVision
                ? photonSimCamera.getMegaTag1_Pose3d()
                : limelight.getMegaTag1_Pose3d();
    }

    public Pose2d getMegaTag2_Pose2d() {
        return usePhotonVision
                ? photonSimCamera.getMegaTag2_Pose2d()
                : limelight.getMegaTag2_Pose2d();
    }

    public PoseEstimate getMegaTag1_PoseEstimate() {
        return usePhotonVision
                ? photonSimCamera.getMegaTag1_PoseEstimate()
                : limelight.getMegaTag1_PoseEstimate();
    }

    public PoseEstimate getMegaTag2_PoseEstimate() {
        return usePhotonVision
                ? photonSimCamera.getMegaTag2_PoseEstimate()
                : limelight.getMegaTag2_PoseEstimate();
    }

    public boolean hasAccuratePose() {
        return usePhotonVision ? photonSimCamera.hasAccuratePose() : limelight.hasAccuratePose();
    }

    public double getDistanceToTagFromCamera() {
        return usePhotonVision
                ? photonSimCamera.getDistanceToTagFromCamera()
                : limelight.getDistanceToTagFromCamera();
    }

    public RawFiducial[] getRawFiducial() {
        return usePhotonVision ? photonSimCamera.getRawFiducial() : limelight.getRawFiducial();
    }

    public double getMegaTag1PoseTimestamp() {
        return usePhotonVision
                ? photonSimCamera.getMegaTag1PoseTimestamp()
                : limelight.getMegaTag1PoseTimestamp();
    }

    public double getMegaTag2PoseTimestamp() {
        return usePhotonVision
                ? photonSimCamera.getMegaTag2PoseTimestamp()
                : limelight.getMegaTag2PoseTimestamp();
    }

    public double getDistanceToTarget(double targetHeight) {
        return usePhotonVision
                ? photonSimCamera.getDistanceToTarget(targetHeight)
                : limelight.getDistanceToTarget(targetHeight);
    }

    public void sendValidStatus(String message) {
        if (!usePhotonVision) {
            limelight.sendValidStatus(message);
        }
    }

    public void sendInvalidStatus(String message) {
        if (!usePhotonVision) {
            limelight.sendInvalidStatus(message);
        }
    }

    public void setLimelightPipeline(int pipelineIndex) {
        if (!usePhotonVision) {
            limelight.setLimelightPipeline(pipelineIndex);
        }
    }

    public void setRobotOrientation(double degrees) {
        if (!usePhotonVision) {
            limelight.setRobotOrientation(degrees);
        }
    }

    public void setRobotOrientation(double degrees, double angularRate) {
        if (!usePhotonVision) {
            limelight.setRobotOrientation(degrees, angularRate);
        }
    }

    public void setIMUmode(int mode) {
        if (!usePhotonVision) {
            limelight.setIMUmode(mode);
        }
    }

    public double getTagTx() {
        return usePhotonVision ? photonSimCamera.getTagTx() : limelight.getTagTx();
    }

    public double getTagTA() {
        return usePhotonVision ? photonSimCamera.getTagTA() : limelight.getTagTA();
    }

    public double getTagRotationDegrees() {
        return usePhotonVision
                ? photonSimCamera.getTagRotationDegrees()
                : limelight.getTagRotationDegrees();
    }

    public void setLEDMode(boolean enabled) {
        if (!usePhotonVision) {
            limelight.setLEDMode(enabled);
        }
    }

    public void blinkLEDs() {
        if (!usePhotonVision) {
            limelight.blinkLEDs();
        }
    }

    public boolean isCameraConnected() {
        return usePhotonVision ? false : limelight.isCameraConnected();
    }

    public void printDebug() {
        if (!usePhotonVision) {
            limelight.printDebug();
        }
    }
}
