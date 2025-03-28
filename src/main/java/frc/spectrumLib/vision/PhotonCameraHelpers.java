package frc.spectrumLib.vision;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;

/**
 * PhotonCameraHelpers provides static methods and classes for interfacing with PhotonCamera vision
 * cameras in FRC. This library supports all PhotonCamera features including AprilTag tracking,
 * Neural Networks, and standard color/retroreflective tracking.
 */
public class PhotonCameraHelpers {

    private static final Map<String, DoubleArrayEntry> doubleArrayEntries =
            new ConcurrentHashMap<>();

    /** Represents a Color/Retroreflective Target Result extracted from JSON Output */
    public static class PhotonCameraTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;

        public PhotonCameraTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    /** Represents an AprilTag/Fiducial Target Result extracted from JSON Output */
    public static class PhotonCameraTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;

        public PhotonCameraTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    /** Represents a Barcode Target Result extracted from JSON Output */
    public static class PhotonCameraTarget_Barcode {

        /** Barcode family type (e.g. "QR", "DataMatrix", etc.) */
        @JsonProperty("fam")
        public String family;

        /** Gets the decoded data content of the barcode */
        @JsonProperty("data")
        public String data;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("pts")
        public double[][] corners;

        public PhotonCameraTarget_Barcode() {}

        public String getFamily() {
            return family;
        }
    }

    /** Represents a Neural Classifier Pipeline Result extracted from JSON Output */
    public static class PhotonCameraTarget_Classifier {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("zone")
        public double zone;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public PhotonCameraTarget_Classifier() {}
    }

    /** Represents a Neural Detector Pipeline Result extracted from JSON Output */
    public static class PhotonCameraTarget_Detector {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        public PhotonCameraTarget_Detector() {}
    }

    /** PhotonCamera Results object, parsed from a PhotonCamera's JSON results output. */
    public static class PhotonCameraResults {

        public String error;

        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        public double latency_jsonParse;

        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("botpose_tagcount")
        public double botpose_tagcount;

        @JsonProperty("botpose_span")
        public double botpose_span;

        @JsonProperty("botpose_avgdist")
        public double botpose_avgdist;

        @JsonProperty("botpose_avgarea")
        public double botpose_avgarea;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }

        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }

        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }

        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }

        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @JsonProperty("Retro")
        public PhotonCameraTarget_Retro[] targets_Retro;

        @JsonProperty("Fiducial")
        public PhotonCameraTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("Classifier")
        public PhotonCameraTarget_Classifier[] targets_Classifier;

        @JsonProperty("Detector")
        public PhotonCameraTarget_Detector[] targets_Detector;

        @JsonProperty("Barcode")
        public PhotonCameraTarget_Barcode[] targets_Barcode;

        public PhotonCameraResults() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new PhotonCameraTarget_Retro[0];
            targets_Fiducials = new PhotonCameraTarget_Fiducial[0];
            targets_Classifier = new PhotonCameraTarget_Classifier[0];
            targets_Detector = new PhotonCameraTarget_Detector[0];
            targets_Barcode = new PhotonCameraTarget_Barcode[0];
        }
    }

    /**
     * Represents a PhotonCamera Raw Neural Detector result from PhotonCamera's NetworkTables
     * output.
     */
    public static class RawDetection {
        public int classId = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double corner0_X = 0;
        public double corner0_Y = 0;
        public double corner1_X = 0;
        public double corner1_Y = 0;
        public double corner2_X = 0;
        public double corner2_Y = 0;
        public double corner3_X = 0;
        public double corner3_Y = 0;

        public RawDetection(
                int classId,
                double txnc,
                double tync,
                double ta,
                double corner0_X,
                double corner0_Y,
                double corner1_X,
                double corner1_Y,
                double corner2_X,
                double corner2_Y,
                double corner3_X,
                double corner3_Y) {
            this.classId = classId;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.corner0_X = corner0_X;
            this.corner0_Y = corner0_Y;
            this.corner1_X = corner1_X;
            this.corner1_Y = corner1_Y;
            this.corner2_X = corner2_X;
            this.corner2_Y = corner2_Y;
            this.corner3_X = corner3_X;
            this.corner3_Y = corner3_Y;
        }
    }

    /** Encapsulates the state of an internal PhotonCamera IMU. */
    public static class IMUData {
        public double robotYaw = 0.0;
        public double Roll = 0.0;
        public double Pitch = 0.0;
        public double Yaw = 0.0;
        public double gyroX = 0.0;
        public double gyroY = 0.0;
        public double gyroZ = 0.0;
        public double accelX = 0.0;
        public double accelY = 0.0;
        public double accelZ = 0.0;

        public IMUData() {}

        public IMUData(double[] imuData) {
            if (imuData != null && imuData.length >= 10) {
                this.robotYaw = imuData[0];
                this.Roll = imuData[1];
                this.Pitch = imuData[2];
                this.Yaw = imuData[3];
                this.gyroX = imuData[4];
                this.gyroY = imuData[5];
                this.gyroZ = imuData[6];
                this.accelX = imuData[7];
                this.accelY = imuData[8];
                this.accelZ = imuData[9];
            }
        }
    }

    private static ObjectMapper mapper;

    /** Print JSON Parse time to the console in milliseconds */
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "photonCamera";
        }
        return name;
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose3d object. Array format: [x, y,
     * z, roll, pitch, yaw] where angles are in degrees.
     *
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose3d object representing the pose, or empty Pose3d if invalid data
     */
    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            // System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(
                        Units.degreesToRadians(inData[3]),
                        Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose2d object. Uses only x, y, and
     * yaw components, ignoring z, roll, and pitch. Array format: [x, y, z, roll, pitch, yaw] where
     * angles are in degrees.
     *
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose2d object representing the pose, or empty Pose2d if invalid data
     */
    public static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            // System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    /**
     * Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     *
     * @param pose The Pose3d object to convert
     * @return A 6-element array containing [x, y, z, roll, pitch, yaw]
     */
    public static double[] pose3dToArray(Pose3d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = pose.getTranslation().getZ();
        result[3] = Units.radiansToDegrees(pose.getRotation().getX());
        result[4] = Units.radiansToDegrees(pose.getRotation().getY());
        result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
        return result;
    }

    /**
     * Converts a Pose2d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees. Note: z, roll, and
     * pitch will be 0 since Pose2d only contains x, y, and yaw.
     *
     * @param pose The Pose2d object to convert
     * @return A 6-element array containing [x, y, 0, 0, 0, yaw]
     */
    public static double[] pose2dToArray(Pose2d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = 0;
        result[3] = Units.radiansToDegrees(0);
        result[4] = Units.radiansToDegrees(0);
        result[5] = Units.radiansToDegrees(pose.getRotation().getRadians());
        return result;
    }

    private static double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(
            String photonCameraName, String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry =
                PhotonCameraHelpers.getPhotonCameraDoubleArrayEntry(photonCameraName, entryName);

        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return null; // or some default PoseEstimate
        }

        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);

        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length != expectedTotalVals) {
            // Don't populate fiducials
        } else {
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] =
                        new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(
                pose,
                adjustedTimestamp,
                latency,
                tagCount,
                tagSpan,
                tagDist,
                tagArea,
                rawFiducials,
                isMegaTag2);
    }

    /**
     * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Array of RawFiducial objects containing detection details
     */
    public static RawFiducial[] getRawFiducials(String photonCameraName) {
        var entry =
                PhotonCameraHelpers.getPhotonCameraNTTableEntry(photonCameraName, "rawfiducials");
        var rawFiducialArray = entry.getDoubleArray(new double[0]);
        int valsPerEntry = 7;
        if (rawFiducialArray.length % valsPerEntry != 0) {
            return new RawFiducial[0];
        }

        int numFiducials = rawFiducialArray.length / valsPerEntry;
        RawFiducial[] rawFiducials = new RawFiducial[numFiducials];

        for (int i = 0; i < numFiducials; i++) {
            int baseIndex = i * valsPerEntry;
            int id = (int) extractArrayEntry(rawFiducialArray, baseIndex);
            double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
            double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
            double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
            double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
            double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
            double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);

            rawFiducials[i] =
                    new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
        }

        return rawFiducials;
    }

    /**
     * Gets the latest raw neural detector results from NetworkTables
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Array of RawDetection objects containing detection details
     */
    public static RawDetection[] getRawDetections(String photonCameraName) {
        var entry =
                PhotonCameraHelpers.getPhotonCameraNTTableEntry(photonCameraName, "rawdetections");
        var rawDetectionArray = entry.getDoubleArray(new double[0]);
        int valsPerEntry = 12;
        if (rawDetectionArray.length % valsPerEntry != 0) {
            return new RawDetection[0];
        }

        int numDetections = rawDetectionArray.length / valsPerEntry;
        RawDetection[] rawDetections = new RawDetection[numDetections];

        for (int i = 0; i < numDetections; i++) {
            int baseIndex = i * valsPerEntry; // Starting index for this detection's data
            int classId = (int) extractArrayEntry(rawDetectionArray, baseIndex);
            double txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1);
            double tync = extractArrayEntry(rawDetectionArray, baseIndex + 2);
            double ta = extractArrayEntry(rawDetectionArray, baseIndex + 3);
            double corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4);
            double corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5);
            double corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6);
            double corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7);
            double corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8);
            double corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9);
            double corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10);
            double corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11);

            rawDetections[i] =
                    new RawDetection(
                            classId, txnc, tync, ta, corner0_X, corner0_Y, corner1_X, corner1_Y,
                            corner2_X, corner2_Y, corner3_X, corner3_Y);
        }

        return rawDetections;
    }

    /**
     * Prints detailed information about a PoseEstimate to standard output. Includes timestamp,
     * latency, tag count, tag span, average tag distance, average tag area, and detailed
     * information about each detected fiducial.
     *
     * @param pose The PoseEstimate object to print. If null, prints "No PoseEstimate available."
     */
    public static void printPoseEstimate(PoseEstimate pose) {
        if (pose == null) {
            System.out.println("No PoseEstimate available.");
            return;
        }

        System.out.printf("Pose Estimate Information:%n");
        System.out.printf("Timestamp (Seconds): %.3f%n", pose.timestampSeconds);
        System.out.printf("Latency: %.3f ms%n", pose.latency);
        System.out.printf("Tag Count: %d%n", pose.tagCount);
        System.out.printf("Tag Span: %.2f meters%n", pose.tagSpan);
        System.out.printf("Average Tag Distance: %.2f meters%n", pose.avgTagDist);
        System.out.printf("Average Tag Area: %.2f%% of image%n", pose.avgTagArea);
        System.out.printf("Is MegaTag2: %b%n", pose.isMegaTag2);
        System.out.println();

        if (pose.rawFiducials == null || pose.rawFiducials.length == 0) {
            System.out.println("No RawFiducials data available.");
            return;
        }

        System.out.println("Raw Fiducials Details:");
        for (int i = 0; i < pose.rawFiducials.length; i++) {
            RawFiducial fiducial = pose.rawFiducials[i];
            System.out.printf(" Fiducial #%d:%n", i + 1);
            System.out.printf("  ID: %d%n", fiducial.id);
            System.out.printf("  TXNC: %.2f%n", fiducial.txnc);
            System.out.printf("  TYNC: %.2f%n", fiducial.tync);
            System.out.printf("  TA: %.2f%n", fiducial.ta);
            System.out.printf("  Distance to Camera: %.2f meters%n", fiducial.distToCamera);
            System.out.printf("  Distance to Robot: %.2f meters%n", fiducial.distToRobot);
            System.out.printf("  Ambiguity: %.2f%n", fiducial.ambiguity);
            System.out.println();
        }
    }

    public static Boolean validPoseEstimate(PoseEstimate pose) {
        return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
    }

    public static NetworkTable getPhotonCameraNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static void Flush() {
        NetworkTableInstance.getDefault().flush();
    }

    public static NetworkTableEntry getPhotonCameraNTTableEntry(
            String tableName, String entryName) {
        return getPhotonCameraNTTable(tableName).getEntry(entryName);
    }

    public static DoubleArrayEntry getPhotonCameraDoubleArrayEntry(
            String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(
                key,
                k -> {
                    NetworkTable table = getPhotonCameraNTTable(tableName);
                    return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
                });
    }

    public static double getPhotonCameraNTDouble(String tableName, String entryName) {
        return getPhotonCameraNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setPhotonCameraNTDouble(String tableName, String entryName, double val) {
        getPhotonCameraNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setPhotonCameraNTDoubleArray(
            String tableName, String entryName, double[] val) {
        getPhotonCameraNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getPhotonCameraNTDoubleArray(String tableName, String entryName) {
        return getPhotonCameraNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    public static String getPhotonCameraNTString(String tableName, String entryName) {
        return getPhotonCameraNTTableEntry(tableName, entryName).getString("");
    }

    public static String[] getPhotonCameraNTStringArray(String tableName, String entryName) {
        return getPhotonCameraNTTableEntry(tableName, entryName).getStringArray(new String[0]);
    }

    public static URL getPhotonCameraURLString(String tableName, String request) {
        String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
    /////

    /**
     * Does the PhotonCamera have a valid target?
     *
     * @param photonCameraName Name of the PhotonCamera camera ("" for default)
     * @return True if a valid target is present, false otherwise
     */
    public static boolean getTV(String photonCameraName) {
        return 1.0 == getPhotonCameraNTDouble(photonCameraName, "tv");
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     *
     * @param photonCameraName Name of the PhotonCamera camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    public static double getTX(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "tx");
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     *
     * @param photonCameraName Name of the PhotonCamera camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    public static double getTY(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "ty");
    }

    /**
     * Gets the horizontal offset from the principal pixel/point to the target in degrees. This is
     * the most accurate 2d metric if you are using a calibrated camera and you don't need
     * adjustable crosshair functionality.
     *
     * @param photonCameraName Name of the PhotonCamera camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    public static double getTXNC(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "txnc");
    }

    /**
     * Gets the vertical offset from the principal pixel/point to the target in degrees. This is the
     * most accurate 2d metric if you are using a calibrated camera and you don't need adjustable
     * crosshair functionality.
     *
     * @param photonCameraName Name of the PhotonCamera camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    public static double getTYNC(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "tync");
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     *
     * @param photonCameraName Name of the PhotonCamera camera ("" for default)
     * @return Target area percentage (0-100)
     */
    public static double getTA(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "ta");
    }

    /**
     * T2D is an array that contains several targeting metrics
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Array containing [targetValid, targetCount, targetLatency, captureLatency, tx, ty,
     *     txnc, tync, ta, tid, targetClassIndexDetector, targetClassIndexClassifier,
     *     targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels,
     *     targetVerticalExtentPixels, targetSkewDegrees]
     */
    public static double[] getT2DArray(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "t2d");
    }

    /**
     * Gets the number of targets currently detected.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Number of detected targets
     */
    public static int getTargetCount(String photonCameraName) {
        double[] t2d = getT2DArray(photonCameraName);
        if (t2d.length == 17) {
            return (int) t2d[1];
        }
        return 0;
    }

    /**
     * Gets the classifier class index from the currently running neural classifier pipeline
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Class index from classifier pipeline
     */
    public static int getClassifierClassIndex(String photonCameraName) {
        double[] t2d = getT2DArray(photonCameraName);
        if (t2d.length == 17) {
            return (int) t2d[10];
        }
        return 0;
    }

    /**
     * Gets the detector class index from the primary result of the currently running neural
     * detector pipeline.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Class index from detector pipeline
     */
    public static int getDetectorClassIndex(String photonCameraName) {
        double[] t2d = getT2DArray(photonCameraName);
        if (t2d.length == 17) {
            return (int) t2d[11];
        }
        return 0;
    }

    /**
     * Gets the current neural classifier result class name.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Class name string from classifier pipeline
     */
    public static String getClassifierClass(String photonCameraName) {
        return getPhotonCameraNTString(photonCameraName, "tcclass");
    }

    /**
     * Gets the primary neural detector result class name.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Class name string from detector pipeline
     */
    public static String getDetectorClass(String photonCameraName) {
        return getPhotonCameraNTString(photonCameraName, "tdclass");
    }

    /**
     * Gets the pipeline's processing latency contribution.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Pipeline latency in milliseconds
     */
    public static double getLatency_Pipeline(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "tl");
    }

    /**
     * Gets the capture latency.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Capture latency in milliseconds
     */
    public static double getLatency_Capture(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "cl");
    }

    /**
     * Gets the active pipeline index.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Current pipeline index (0-9)
     */
    public static double getCurrentPipelineIndex(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "getpipe");
    }

    /**
     * Gets the current pipeline type.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return Pipeline type string (e.g. "retro", "apriltag", etc.)
     */
    public static String getCurrentPipelineType(String photonCameraName) {
        return getPhotonCameraNTString(photonCameraName, "getpipetype");
    }

    /**
     * Gets the full JSON results dump.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return JSON string containing all current results
     */
    public static String getJSONDump(String photonCameraName) {
        return getPhotonCameraNTString(photonCameraName, "json");
    }

    /**
     * Switch to getBotPose
     *
     * @param photonCameraName
     * @return
     */
    @Deprecated
    public static double[] getBotpose(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     *
     * @param photonCameraName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     *
     * @param photonCameraName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "botpose_wpiblue");
    }

    public static double[] getBotPose(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "botpose_wpiblue");
    }

    public static double[] getBotPose_TargetSpace(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "botpose_targetspace");
    }

    public static double[] getCameraPose_TargetSpace(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "camerapose_targetspace");
    }

    public static double[] getTargetPose_CameraSpace(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "targetpose_cameraspace");
    }

    public static double[] getTargetPose_RobotSpace(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "targetpose_robotspace");
    }

    public static double[] getTargetColor(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "tc");
    }

    public static double getFiducialID(String photonCameraName) {
        return getPhotonCameraNTDouble(photonCameraName, "tid");
    }

    public static String getNeuralClassID(String photonCameraName) {
        return getPhotonCameraNTString(photonCameraName, "tclass");
    }

    public static String[] getRawBarcodeData(String photonCameraName) {
        return getPhotonCameraNTStringArray(photonCameraName, "rawbarcodes");
    }

    /////
    /////

    public static Pose3d getBotPose3d(String photonCameraName) {
        double[] poseArray = getPhotonCameraNTDoubleArray(photonCameraName, "botpose");
        return toPose3D(poseArray);
    }

    /**
     * (Not Recommended) Gets the robot's 3D pose in the WPILib Red Alliance Coordinate System.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Pose3d object representing the robot's position and orientation in Red Alliance field
     *     space
     */
    public static Pose3d getBotPose3d_wpiRed(String photonCameraName) {
        double[] poseArray = getPhotonCameraNTDoubleArray(photonCameraName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    /**
     * (Recommended) Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Pose3d object representing the robot's position and orientation in Blue Alliance
     *     field space
     */
    public static Pose3d getBotPose3d_wpiBlue(String photonCameraName) {
        double[] poseArray = getPhotonCameraNTDoubleArray(photonCameraName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    /**
     * Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Pose3d object representing the robot's position and orientation relative to the
     *     target
     */
    public static Pose3d getBotPose3d_TargetSpace(String photonCameraName) {
        double[] poseArray = getPhotonCameraNTDoubleArray(photonCameraName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Pose3d object representing the camera's position and orientation relative to the
     *     target
     */
    public static Pose3d getCameraPose3d_TargetSpace(String photonCameraName) {
        double[] poseArray =
                getPhotonCameraNTDoubleArray(photonCameraName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the target's 3D pose with respect to the camera's coordinate system.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Pose3d object representing the target's position and orientation relative to the
     *     camera
     */
    public static Pose3d getTargetPose3d_CameraSpace(String photonCameraName) {
        double[] poseArray =
                getPhotonCameraNTDoubleArray(photonCameraName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the target's 3D pose with respect to the robot's coordinate system.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Pose3d object representing the target's position and orientation relative to the
     *     robot
     */
    public static Pose3d getTargetPose3d_RobotSpace(String photonCameraName) {
        double[] poseArray =
                getPhotonCameraNTDoubleArray(photonCameraName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the camera's 3D pose with respect to the robot's coordinate system.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return Pose3d object representing the camera's position and orientation relative to the
     *     robot
     */
    public static Pose3d getCameraPose3d_RobotSpace(String photonCameraName) {
        double[] poseArray =
                getPhotonCameraNTDoubleArray(photonCameraName, "camerapose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
     *
     * @param photonCameraName
     * @return
     */
    public static Pose2d getBotPose2d_wpiBlue(String photonCameraName) {

        double[] result = getBotPose_wpiBlue(photonCameraName);
        return toPose2D(result);
    }

    /**
     * Gets the MegaTag1 Pose2d and timestamp for use with WPILib pose estimator
     * (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     *
     * @param photonCameraName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String photonCameraName) {
        return getBotPoseEstimate(photonCameraName, "botpose_wpiblue", false);
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator
     * (addVisionMeasurement) in the WPILib Blue alliance coordinate system. Make sure you are
     * calling setRobotOrientation() before calling this method.
     *
     * @param photonCameraName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String photonCameraName) {
        return getBotPoseEstimate(photonCameraName, "botpose_orb_wpiblue", true);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
     *
     * @param photonCameraName
     * @return
     */
    public static Pose2d getBotPose2d_wpiRed(String photonCameraName) {

        double[] result = getBotPose_wpiRed(photonCameraName);
        return toPose2D(result);
    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when
     * you are on the RED alliance
     *
     * @param photonCameraName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String photonCameraName) {
        return getBotPoseEstimate(photonCameraName, "botpose_wpired", false);
    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when
     * you are on the RED alliance
     *
     * @param photonCameraName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String photonCameraName) {
        return getBotPoseEstimate(photonCameraName, "botpose_orb_wpired", true);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
     *
     * @param photonCameraName
     * @return
     */
    public static Pose2d getBotPose2d(String photonCameraName) {

        double[] result = getBotPose(photonCameraName);
        return toPose2D(result);
    }

    /**
     * Gets the current IMU data from NetworkTables. IMU data is formatted as [robotYaw, Roll,
     * Pitch, Yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ]. Returns all zeros if data is
     * invalid or unavailable.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @return IMUData object containing all current IMU data
     */
    public static IMUData getIMUData(String photonCameraName) {
        double[] imuData = getPhotonCameraNTDoubleArray(photonCameraName, "imu");
        if (imuData == null || imuData.length < 10) {
            return new IMUData(); // Returns object with all zeros
        }
        return new IMUData(imuData);
    }

    /////
    /////

    public static void setPipelineIndex(String photonCameraName, int pipelineIndex) {
        setPhotonCameraNTDouble(photonCameraName, "pipeline", pipelineIndex);
    }

    public static void setPriorityTagID(String photonCameraName, int ID) {
        setPhotonCameraNTDouble(photonCameraName, "priorityid", ID);
    }

    /**
     * Sets LED mode to be controlled by the current pipeline.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     */
    public static void setLEDMode_PipelineControl(String photonCameraName) {
        setPhotonCameraNTDouble(photonCameraName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String photonCameraName) {
        setPhotonCameraNTDouble(photonCameraName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String photonCameraName) {
        setPhotonCameraNTDouble(photonCameraName, "ledMode", 2);
    }

    public static void setLEDMode_ForceOn(String photonCameraName) {
        setPhotonCameraNTDouble(photonCameraName, "ledMode", 3);
    }

    /**
     * Enables standard side-by-side stream mode.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     */
    public static void setStreamMode_Standard(String photonCameraName) {
        setPhotonCameraNTDouble(photonCameraName, "stream", 0);
    }

    /**
     * Enables Picture-in-Picture mode with secondary stream in the corner.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     */
    public static void setStreamMode_PiPMain(String photonCameraName) {
        setPhotonCameraNTDouble(photonCameraName, "stream", 1);
    }

    /**
     * Enables Picture-in-Picture mode with primary stream in the corner.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     */
    public static void setStreamMode_PiPSecondary(String photonCameraName) {
        setPhotonCameraNTDouble(photonCameraName, "stream", 2);
    }

    /**
     * Sets the crop window for the camera. The crop window in the UI must be completely open.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @param cropXMin Minimum X value (-1 to 1)
     * @param cropXMax Maximum X value (-1 to 1)
     * @param cropYMin Minimum Y value (-1 to 1)
     * @param cropYMax Maximum Y value (-1 to 1)
     */
    public static void setCropWindow(
            String photonCameraName,
            double cropXMin,
            double cropXMax,
            double cropYMin,
            double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropXMin;
        entries[1] = cropXMax;
        entries[2] = cropYMin;
        entries[3] = cropYMax;
        setPhotonCameraNTDoubleArray(photonCameraName, "crop", entries);
    }

    /** Sets 3D offset point for easy 3D targeting. */
    public static void setFiducial3DOffset(
            String photonCameraName, double offsetX, double offsetY, double offsetZ) {
        double[] entries = new double[3];
        entries[0] = offsetX;
        entries[1] = offsetY;
        entries[2] = offsetZ;
        setPhotonCameraNTDoubleArray(photonCameraName, "fiducial_offset_set", entries);
    }

    /**
     * Sets robot orientation values used by MegaTag2 localization algorithm.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
     * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
     * @param pitch (Unnecessary) Robot pitch in degrees
     * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
     * @param roll (Unnecessary) Robot roll in degrees
     * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
     */
    public static void SetRobotOrientation(
            String photonCameraName,
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        SetRobotOrientation_INTERNAL(
                photonCameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
    }

    public static void SetRobotOrientation_NoFlush(
            String photonCameraName,
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        SetRobotOrientation_INTERNAL(
                photonCameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate, false);
    }

    private static void SetRobotOrientation_INTERNAL(
            String photonCameraName,
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            boolean flush) {

        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = yawRate;
        entries[2] = pitch;
        entries[3] = pitchRate;
        entries[4] = roll;
        entries[5] = rollRate;
        setPhotonCameraNTDoubleArray(photonCameraName, "robot_orientation_set", entries);
        if (flush) {
            Flush();
        }
    }

    /**
     * Configures the IMU mode for MegaTag2 Localization
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @param mode IMU mode.
     */
    public static void SetIMUMode(String photonCameraName, int mode) {
        setPhotonCameraNTDouble(photonCameraName, "imumode_set", mode);
    }

    /**
     * Sets the 3D point-of-interest offset for the current fiducial pipeline.
     * https://docs.photonCameravision.io/docs/docs-photonCamera/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @param x X offset in meters
     * @param y Y offset in meters
     * @param z Z offset in meters
     */
    public static void SetFidcuial3DOffset(String photonCameraName, double x, double y, double z) {

        double[] entries = new double[3];
        entries[0] = x;
        entries[1] = y;
        entries[2] = z;
        setPhotonCameraNTDoubleArray(photonCameraName, "fiducial_offset_set", entries);
    }

    /**
     * Overrides the valid AprilTag IDs that will be used for localization. Tags not in this list
     * will be ignored for robot pose estimation.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @param validIDs Array of valid AprilTag IDs to track
     */
    public static void SetFiducialIDFiltersOverride(String photonCameraName, int[] validIDs) {
        double[] validIDsDouble = new double[validIDs.length];
        for (int i = 0; i < validIDs.length; i++) {
            validIDsDouble[i] = validIDs[i];
        }
        setPhotonCameraNTDoubleArray(photonCameraName, "fiducial_id_filters_set", validIDsDouble);
    }

    /**
     * Sets the downscaling factor for AprilTag detection. Increasing downscale can improve
     * performance at the cost of potentially reduced detection range.
     *
     * @param photonCameraName Name/identifier of the PhotonCamera
     * @param downscale Downscale factor. Valid values: 1.0 (no downscale), 1.5, 2.0, 3.0, 4.0. Set
     *     to 0 for pipeline control.
     */
    public static void SetFiducialDownscalingOverride(String photonCameraName, float downscale) {
        int d = 0; // pipeline
        if (downscale == 1.0) {
            d = 1;
        }
        if (downscale == 1.5) {
            d = 2;
        }
        if (downscale == 2) {
            d = 3;
        }
        if (downscale == 3) {
            d = 4;
        }
        if (downscale == 4) {
            d = 5;
        }
        setPhotonCameraNTDouble(photonCameraName, "fiducial_downscale_set", d);
    }

    /**
     * Sets the camera pose relative to the robot.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @param forward Forward offset in meters
     * @param side Side offset in meters
     * @param up Up offset in meters
     * @param roll Roll angle in degrees
     * @param pitch Pitch angle in degrees
     * @param yaw Yaw angle in degrees
     */
    public static void setCameraPose_RobotSpace(
            String photonCameraName,
            double forward,
            double side,
            double up,
            double roll,
            double pitch,
            double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setPhotonCameraNTDoubleArray(photonCameraName, "camerapose_robotspace_set", entries);
    }

    /////
    /////

    public static void setPythonScriptData(String photonCameraName, double[] outgoingPythonData) {
        setPhotonCameraNTDoubleArray(photonCameraName, "llrobot", outgoingPythonData);
    }

    public static double[] getPythonScriptData(String photonCameraName) {
        return getPhotonCameraNTDoubleArray(photonCameraName, "llpython");
    }

    /////
    /////

    /** Asynchronously take snapshot. */
    public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
        return CompletableFuture.supplyAsync(
                () -> {
                    return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
                });
    }

    private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
        URL url = getPhotonCameraURLString(tableName, "capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

    /**
     * Gets the latest JSON results output and returns a PhotonCameraResults object.
     *
     * @param photonCameraName Name of the PhotonCamera camera
     * @return PhotonCameraResults object containing all current target data
     */
    public static PhotonCameraResults getLatestResults(String photonCameraName) {

        long start = System.nanoTime();
        PhotonCameraHelpers.PhotonCameraResults results =
                new PhotonCameraHelpers.PhotonCameraResults();
        if (mapper == null) {
            mapper =
                    new ObjectMapper()
                            .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(photonCameraName), PhotonCameraResults.class);
        } catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.latency_jsonParse = millis;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return results;
    }
}
