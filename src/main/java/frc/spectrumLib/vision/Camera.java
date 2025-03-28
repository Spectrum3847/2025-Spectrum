package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import frc.spectrumLib.vision.PhotonCamera.PhotonCameraConfig;

public class Camera {
    private Limelight limelight;
    private PhotonCamera photonCamera;
    private boolean usePhotonVision = RobotBase.isSimulation();

    public Camera(String name) {
        if (usePhotonVision) {
            photonCamera = new PhotonCamera(name);
        } else {
            limelight = new Limelight(name);
        }
    }

    public Camera(String name, boolean attached) {
        if (usePhotonVision) {
            photonCamera = new PhotonCamera(name, attached);
        } else {
            limelight = new Limelight(name, attached);
        }
    }

    public Camera(String name, int pipeline) {
        if (usePhotonVision) {
            photonCamera = new PhotonCamera(name, pipeline);
        } else {
            limelight = new Limelight(name, pipeline);
        }
    }

    public Camera(String name, int pipeline, LimelightConfig config) {
        limelight = new Limelight(name, pipeline, config);
    }

    public Camera(String name, int pipeline, PhotonCameraConfig config) {
        photonCamera = new PhotonCamera(name, pipeline, config);
    }

    public String getName() {
        return usePhotonVision ? photonCamera.getName() : limelight.getName();
    }

    public boolean isAttached() {
        return usePhotonVision ? photonCamera.isAttached() : limelight.isAttached();
    }

    public double getHorizontalOffset() {
        return usePhotonVision
                ? photonCamera.getHorizontalOffset()
                : limelight.getHorizontalOffset();
    }

    public double getVerticalOffset() {
        return usePhotonVision ? photonCamera.getVerticalOffset() : limelight.getVerticalOffset();
    }

    public boolean targetInView() {
        return usePhotonVision ? photonCamera.targetInView() : limelight.targetInView();
    }

    public boolean multipleTagsInView() {
        return usePhotonVision ? photonCamera.multipleTagsInView() : limelight.multipleTagsInView();
    }

    public double getTagCountInView() {
        return usePhotonVision ? photonCamera.getTagCountInView() : limelight.getTagCountInView();
    }

    public double getClosestTagID() {
        return usePhotonVision ? photonCamera.getClosestTagID() : limelight.getClosestTagID();
    }

    public double getTargetSize() {
        return usePhotonVision ? photonCamera.getTargetSize() : limelight.getTargetSize();
    }

    public Pose3d getMegaTag1_Pose3d() {
        return usePhotonVision ? photonCamera.getMegaTag1_Pose3d() : limelight.getMegaTag1_Pose3d();
    }

    public Pose2d getMegaTag2_Pose2d() {
        return usePhotonVision ? photonCamera.getMegaTag2_Pose2d() : limelight.getMegaTag2_Pose2d();
    }

    public PoseEstimate getMegaTag1_PoseEstimate() {
        return usePhotonVision
                ? photonCamera.getMegaTag1_PoseEstimate()
                : limelight.getMegaTag1_PoseEstimate();
    }

    public PoseEstimate getMegaTag2_PoseEstimate() {
        return usePhotonVision
                ? photonCamera.getMegaTag2_PoseEstimate()
                : limelight.getMegaTag2_PoseEstimate();
    }

    public boolean hasAccuratePose() {
        return usePhotonVision ? photonCamera.hasAccuratePose() : limelight.hasAccuratePose();
    }

    public double getDistanceToTagFromCamera() {
        return usePhotonVision
                ? photonCamera.getDistanceToTagFromCamera()
                : limelight.getDistanceToTagFromCamera();
    }

    public RawFiducial[] getRawFiducial() {
        return usePhotonVision ? photonCamera.getRawFiducial() : limelight.getRawFiducial();
    }

    public double getMegaTag1PoseTimestamp() {
        return usePhotonVision
                ? photonCamera.getMegaTag1PoseTimestamp()
                : limelight.getMegaTag1PoseTimestamp();
    }

    public double getMegaTag2PoseTimestamp() {
        return usePhotonVision
                ? photonCamera.getMegaTag2PoseTimestamp()
                : limelight.getMegaTag2PoseTimestamp();
    }

    public double getDistanceToTarget(double targetHeight) {
        return usePhotonVision
                ? photonCamera.getDistanceToTarget(targetHeight)
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
        return usePhotonVision ? photonCamera.getTagTx() : limelight.getTagTx();
    }

    public double getTagTA() {
        return usePhotonVision ? photonCamera.getTagTA() : limelight.getTagTA();
    }

    public double getTagRotationDegrees() {
        return usePhotonVision
                ? photonCamera.getTagRotationDegrees()
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
