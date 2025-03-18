package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.vision.Limelight;
import lombok.Getter;

public class VisionLogger {
    /** Tracks position with Limelight using current logger (DogLog) to record data */
    private final Limelight limelight;

    @Getter private String name;

    public VisionLogger(String name, Limelight limelight) {
        this.limelight = limelight;
        this.name = name;
    }

    public boolean getCameraConnection() {
        Telemetry.log("Vision " + name + " ConnectionStatus", limelight.isCameraConnected());
        return limelight.isCameraConnected();
    }

    public boolean getIntegratingStatus() { // Vision/Integrating
        Telemetry.log("Vision " + name + " IntegratingStatus", limelight.isIntegrating());
        return limelight.isIntegrating();
    }

    public String getLogStatus() {
        Telemetry.log("Vision " + name + " LogStatus", limelight.getLogStatus());
        return limelight.getLogStatus();
    }

    public String getTagStatus() {
        Telemetry.log("Vision " + name + " TagStatus", limelight.getTagStatus());
        return limelight.getLogStatus();
    }

    public Pose2d getPose() {
        Telemetry.log("Vision " + name + " Pose", limelight.getMegaTag1_Pose3d().toPose2d());
        return limelight.getMegaTag1_Pose3d().toPose2d();
    }

    public Pose2d getMegaPose() {
        Telemetry.log("Vision " + name + " MegaPose", limelight.getMegaTag2_Pose2d());
        return limelight.getMegaTag2_Pose2d();
    }

    public double getPoseX() {
        Telemetry.log("Vision " + name + " PoseX", getPose().getX());
        return getPose().getX();
    }

    public double getPoseY() {
        Telemetry.log("Vision " + name + " PoseY", getPose().getY());
        return getPose().getY();
    }

    public double getTagCount() {
        Telemetry.log("Vision " + name + " TagCount", limelight.getTagCountInView());
        return limelight.getTagCountInView();
    }

    public double getTargetSize() {
        Telemetry.log("Vision " + name + " TargetSize", limelight.getTargetSize());
        return limelight.getTargetSize();
    }
}
