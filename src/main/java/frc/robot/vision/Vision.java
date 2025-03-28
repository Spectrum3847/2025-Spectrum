package frc.robot.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Camera;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import frc.spectrumLib.vision.PhotonSimCamera.PhotonSimCameraConfig;
import java.text.DecimalFormat;
import java.util.Arrays;
import lombok.Getter;
import lombok.Setter;

public class Vision implements NTSendable, Subsystem {

    public static class VisionConfig {
        @Getter final String name = "Vision";
        /* Limelight Configuration */
        @Getter final String frontLL = "limelight-front";

        @Getter
        final LimelightConfig frontConfig =
                new LimelightConfig(frontLL)
                        .withTranslation(0.215, 0, 0.188)
                        .withRotation(0, Math.toRadians(28), 0);

        @Getter
        final PhotonSimCameraConfig frontSimConfig =
                new PhotonSimCameraConfig(frontLL)
                        .withTranslation(0.215, 0, 0.188)
                        .withRotation(0, Math.toRadians(0), 0);

        @Getter final String backLL = "limelight-back";

        @Getter
        final LimelightConfig backConfig =
                new LimelightConfig(backLL)
                        .withTranslation(-0.215, 0.0, 0.188)
                        .withRotation(0, Math.toRadians(28), Math.toRadians(180));

        @Getter
        final PhotonSimCameraConfig backSimConfig =
                new PhotonSimCameraConfig(backLL)
                        .withTranslation(-0.215, 0.0, 0.188)
                        .withRotation(0, Math.toRadians(0), Math.toRadians(180));

        /* Pipeline configs */
        @Getter final int frontTagPipeline = 0;
        @Getter final int backTagPipeline = 0;

        /* Pose Estimation Constants */

        @Getter double visionStdDevX = 0.5;
        @Getter double visionStdDevY = 0.5;
        @Getter double visionStdDevTheta = 0.5;

        @Getter
        final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta);
    }

    /** Limelights */
    @Getter public final Camera frontLL;

    public final Camera backLL;

    public final Camera[] allLimelights;

    private final DecimalFormat df = new DecimalFormat();

    @Getter @Setter private boolean isIntegrating = false;

    @Getter private boolean isAiming = false;

    int[] blueTags = {17, 18, 19, 20, 21, 22};
    int[] redTags = {6, 7, 8, 9, 10, 11};

    private VisionConfig config;

    public Vision(VisionConfig config) {
        this.config = config;

        if (Robot.isSimulation()) {
            frontLL = new Camera(config.frontLL, config.frontTagPipeline, config.frontSimConfig);
        } else {
            frontLL = new Camera(config.frontLL, config.frontTagPipeline, config.frontConfig);
        }

        if (Robot.isSimulation()) {
            backLL = new Camera(config.backLL, config.backTagPipeline, config.backSimConfig);
        } else {
            backLL = new Camera(config.backLL, config.backTagPipeline, config.backConfig);
        }

        allLimelights = new Camera[] {frontLL, backLL};

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Camera camera : allLimelights) {
            camera.setLEDMode(false);
            camera.setIMUmode(1);
        }

        this.register();
        telemetryInit();
    }

    @Override
    public String getName() {
        return config.getName();
    }

    // Setup the telemetry values, has to be called at the end of the implemented mechanism
    // constructor
    public void telemetryInit() {
        SendableRegistry.add(this, getName());
        SmartDashboard.putData(this);

        Robot.getField2d().getObject(frontLL.getName());
        Robot.getField2d().getObject(backLL.getName());
    }

    @Override
    public void periodic() {
        setLimeLightOrientation();
        disabledLimelightUpdates();
        enabledLimelightUpdates();

        Robot.getField2d().getObject(frontLL.getName()).setPose(getFrontMegaTag2Pose());
        Robot.getField2d().getObject(backLL.getName()).setPose(getBackMegaTag2Pose());
    }

    public Pose2d getFrontMegaTag2Pose() {
        Pose2d pose = frontLL.getMegaTag2_Pose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    public Pose2d getBackMegaTag2Pose() {
        Pose2d pose = backLL.getMegaTag2_Pose2d();
        if (pose != null) {
            return pose;
        }
        return new Pose2d();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.addDoubleProperty("FrontTX", frontLL::getTagTx, null);
        builder.addDoubleProperty("FrontTY", frontLL::getTagTA, null);
        builder.addDoubleProperty("FrontTagID", frontLL::getClosestTagID, null);
        builder.addDoubleProperty("BackTX", backLL::getTagTx, null);
        builder.addDoubleProperty("BackTY", backLL::getTagTA, null);
        builder.addDoubleProperty("BackTagID", backLL::getClosestTagID, null);
    }

    private void setLimeLightOrientation() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();

        for (Camera camera : allLimelights) {
            camera.setRobotOrientation(yaw);
        }
    }

    private void disabledLimelightUpdates() {
        if (Util.disabled.getAsBoolean()) {
            for (Camera limelight : allLimelights) {
                limelight.setIMUmode(1);
            }
            try {
                addMegaTag1_VisionInput(backLL, true);
            } catch (Exception e) {
                Telemetry.print("REAR MT1: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInput(frontLL, true);
            } catch (Exception e) {
                Telemetry.print("FRONT MT1: Vision pose not present but tried to access it");
            }
        }
    }

    private void enabledLimelightUpdates() {
        if (Util.teleop.getAsBoolean()) {
            for (Camera limelight : allLimelights) {
                limelight.setIMUmode(3);
            }
            try {
                addMegaTag2_VisionInput(backLL);
            } catch (Exception e) {
                Telemetry.print("REAR MT2: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag2_VisionInput(frontLL);
            } catch (Exception e) {
                Telemetry.print("FRONT MT2: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInput(backLL, false);
            } catch (Exception e) {
                Telemetry.print("REAR MT1: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInput(frontLL, false);
            } catch (Exception e) {
                Telemetry.print("FRONT MT1: Vision pose not present but tried to access it");
            }
        }
    }

    @SuppressWarnings("all")
    private void addMegaTag1_VisionInput(Camera ll, boolean integrateXY) {
        double xyStds;
        double degStds;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double targetSize = ll.getTargetSize();
            Pose3d megaTag1Pose3d = ll.getMegaTag1_Pose3d();
            Pose2d megaTag1Pose2d = megaTag1Pose3d.toPose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();

            // distance from current pose to vision estimated MT2 pose
            double mt1PoseDifference =
                    Robot.getSwerve()
                            .getRobotPose()
                            .getTranslation()
                            .getDistance(megaTag1Pose2d.getTranslation());

            /* rejections */
            // reject mt1 pose if individual tag ambiguity is too high
            // ll.setTagStatus("");
            for (RawFiducial tag : tags) {
                // search for highest ambiguity tag for later checks
                if (highestAmbiguity == 2 || tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                // ambiguity rejection check
                if (tag.ambiguity > 0.9) {
                    return;
                }
            }

            /* rejections */
            if (rejectionCheck(megaTag1Pose2d, targetSize)) {
                return;
            }

            if (Math.abs(megaTag1Pose3d.getRotation().getX()) > 5
                    || Math.abs(megaTag1Pose3d.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                ll.sendInvalidStatus("roll/pitch rejection");
                return;
            }

            /* integrations */
            // if almost stationary and extremely close to tag
            if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 2) {
                ll.sendValidStatus("Strong Multi integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 0.1) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
                degStds = 8;
            } else if (targetSize > 0.8
                    && (mt1PoseDifference < 0.5 || DriverStation.isDisabled())) {
                // Integrate if the target is very big and we are close to pose or disabled
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
                degStds = 999999;
            } else if (targetSize > 0.1
                    && (mt1PoseDifference < 0.25 || DriverStation.isDisabled())) {
                // Integrate if we are very close to pose or disabled and target is large enough
                ll.sendValidStatus("Proximity integration");
                xyStds = 1.0;
                degStds = 999999;
            } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                ll.sendValidStatus("Stable integration");
                xyStds = 1.5;
                degStds = 999999;
            } else {
                // Shouldn't integrate
                return;
            }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = 15;
            }

            if (robotSpeed.omegaRadiansPerSecond >= 0.5) {
                degStds = 50;
            }

            if (!integrateXY) {
                xyStds = 999999;
            }

            if (integrateXY) {
                xyStds = 0.01;
                degStds = 0.01;
            }

            Pose2d integratedPose =
                    new Pose2d(megaTag1Pose2d.getTranslation(), megaTag1Pose2d.getRotation());
            Robot.getSwerve()
                    .addVisionMeasurement(
                            integratedPose,
                            Utils.fpgaToCurrentTime(ll.getMegaTag1PoseTimestamp()),
                            VecBuilder.fill(xyStds, xyStds, degStds));
        } else {
            // ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    @SuppressWarnings("all")
    private void addMegaTag2_VisionInput(Camera ll) {
        double xyStds;
        double degStds = 99999;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double targetSize = ll.getTargetSize();
            Pose2d megaTag2Pose2d = ll.getMegaTag2_Pose2d();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();

            // distance from current pose to vision estimated MT2 pose
            double mt2PoseDifference =
                    Robot.getSwerve()
                            .getRobotPose()
                            .getTranslation()
                            .getDistance(megaTag2Pose2d.getTranslation());

            /* rejections */
            if (rejectionCheck(megaTag2Pose2d, targetSize)) {
                return;
            }

            /* integrations */
            // if almost stationary and extremely close to tag
            if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
            } else if (multiTags && targetSize > 2) {
                ll.sendValidStatus("Strong Multi integration");
                xyStds = 0.1;
            } else if (multiTags && targetSize > 0.1) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
            } else if (multiTags && targetSize > 2) {
                ll.sendValidStatus("Strong Multi integration");
                xyStds = 0.1;
            } else if (targetSize > 0.8
                    && (mt2PoseDifference < 0.5 || DriverStation.isDisabled())) {
                // Integrate if the target is very big and we are close to pose or disabled
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
            } else if (targetSize > 0.1
                    && (mt2PoseDifference < 0.25 || DriverStation.isDisabled())) {
                // Integrate if we are very close to pose or disabled and target is large enough
                ll.sendValidStatus("Proximity integration");
                xyStds = 0.0;
            } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                ll.sendValidStatus("Stable integration");
                xyStds = 0.5;
            } else {
                // Shouldn't integrate
                return;
            }

            Pose2d integratedPose =
                    new Pose2d(megaTag2Pose2d.getTranslation(), megaTag2Pose2d.getRotation());
            Robot.getSwerve()
                    .addVisionMeasurement(
                            integratedPose,
                            Utils.fpgaToCurrentTime(ll.getMegaTag2PoseTimestamp()),
                            VecBuilder.fill(xyStds, xyStds, degStds));
        } else {
            // ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    private boolean rejectionCheck(Pose2d pose, double targetSize) {
        /* rejections */
        if (Field.poseOutOfField(pose)) {
            return true;
        }

        if (Math.abs(Robot.getSwerve().getCurrentRobotChassisSpeeds().omegaRadiansPerSecond)
                >= 1.6) {
            return true;
        }

        // Final check, if it's small reject, else return false and integrate
        return targetSize <= 0.025;
    }

    /**
     * Choose the limelight with the best view of multiple tags
     *
     * @return
     */
    public Camera getBestLimelight() {
        Camera bestLimelight = frontLL;
        double bestScore = 0;
        for (Camera limelight : allLimelights) {
            double score = 0;
            // prefer LL with most tags, when equal tag count, prefer LL closer to tags
            score += limelight.getTagCountInView();
            score += limelight.getTargetSize();

            if (score > bestScore) {
                bestScore = score;
                bestLimelight = limelight;
            }
        }
        return bestLimelight;
    }

    /** reset pose to the best limelight's vision pose */
    public void resetPoseToVision() {
        Camera ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(),
                ll.getMegaTag1_Pose3d(),
                ll.getMegaTag2_Pose2d(),
                ll.getMegaTag1PoseTimestamp());
    }

    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {

        boolean reject = false;
        if (targetInView) {
            // replace botpose with this.pose
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d pose;

            // Check if the vision pose is bad and don't trust it
            if (Field.poseOutOfField(botpose3D)) { // pose out of field
                Telemetry.log("Pose out of field", reject);
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) { // when in air
                Telemetry.log("Pose in air", reject);
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) { // when tilted

                Telemetry.log("Pose tilted", reject);
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // Posts Current X,Y, and Angle (Theta) values
            double[] visionPose = {
                botpose.getX(), botpose.getY(), botpose.getRotation().getDegrees()
            };
            Telemetry.log("Current Vision Pose: ", visionPose);

            Robot.getSwerve()
                    .setVisionMeasurementStdDevs(VecBuilder.fill(0.00001, 0.00001, 0.00001));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Robot.getSwerve().addVisionMeasurement(integratedPose, poseTimestamp);
            pose = Robot.getSwerve().getRobotPose();
            // Gets updated pose of x, y, and theta values
            visionPose = new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
            Telemetry.log("Vision Pose Reset To: ", visionPose);

            // print "success"
            return true;
        }
        return false; // target not in view
    }

    /**
     * If at least one LL has an accurate pose
     *
     * @return
     */
    public boolean hasAccuratePose() {
        for (Camera limelight : allLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Camera limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    public int getClosestTagID() {
        int closestTagIDFront = (int) frontLL.getClosestTagID();
        int closestTagIDBack = (int) backLL.getClosestTagID();

        if (closestTagIDFront == -1) {
            return (int) closestTagIDBack;
        }
        return (int) closestTagIDFront;
    }

    public boolean isRearTagClosest() {
        int closestTagIDBack = (int) backLL.getClosestTagID();
        return getClosestTagID() != -1 && closestTagIDBack == getClosestTagID();
    }

    // ------------------------------------------------------------------------------
    // Calculation Functions
    // ------------------------------------------------------------------------------

    /**
     * Get the angle the robot should turn to based on the id the limelight is seeing.
     *
     * @return
     */
    public double getReefTagAngle() {
        double[][] reefFrontAngles = {
            {17, 60}, {18, 0}, {19, -60}, {20, -120}, {21, 180}, {22, 120},
            {6, 120}, {7, 180}, {8, -120}, {9, -60}, {10, 0}, {11, 60}
        };

        // int closetFrontTag = (int) frontLL.getClosestTagID();
        // int closetRearTag = (int) backLL.getClosestTagID();
        // int closetTag = closetFrontTag;
        // boolean rearTag = false;

        // if (closetTag == -1) {
        //     closetTag = closetRearTag;
        //     rearTag = true;
        // }

        int closetTag = getClosestTagID();
        boolean rearTag = isRearTagClosest();

        if (closetTag == -1) {
            // Return current angle if no tag seen before going through the array
            return Robot.getSwerve().getRobotPose().getRotation().getRadians();
        }

        for (int i = 0; i < reefFrontAngles.length; i++) {
            if (closetTag == reefFrontAngles[i][0]) {
                if (rearTag) {
                    return Math.toRadians(reefFrontAngles[i][1] + 180);
                }
                return Math.toRadians(reefFrontAngles[i][1]);
            }
        }

        // Return current angle if no tag is found
        return Robot.getSwerve().getRobotPose().getRotation().getRadians();
    }

    public boolean tagsInView() {

        DriverStation.Alliance alliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);

        if (alliance == DriverStation.Alliance.Blue) {
            double closestTagIDFront = frontLL.getClosestTagID();
            double closestTagIDBack = backLL.getClosestTagID();

            boolean isFrontTagInBlue =
                    Arrays.stream(blueTags).anyMatch(tag -> tag == closestTagIDFront);
            boolean isBackTagInBlue =
                    Arrays.stream(blueTags).anyMatch(tag -> tag == closestTagIDBack);

            return isFrontTagInBlue || isBackTagInBlue;
        } else if (alliance == DriverStation.Alliance.Red) {
            double closestTagIDFront = frontLL.getClosestTagID();
            double closestTagIDBack = backLL.getClosestTagID();

            boolean isFrontTagInRed =
                    Arrays.stream(redTags).anyMatch(tag -> tag == closestTagIDFront);
            boolean isBackTagInRed =
                    Arrays.stream(redTags).anyMatch(tag -> tag == closestTagIDBack);

            return isFrontTagInRed || isBackTagInRed;
        } else {
            return false;
        }
    }

    public double getTagTA() {
        if (frontLL.targetInView()) {
            return frontLL.getTagTA();
        } else if (backLL.targetInView()) {
            return backLL.getTagTA();
        } else {
            return 0;
        }
    }

    public double getTagTX() {
        if (frontLL.targetInView()) {
            return frontLL.getTagTx();
        } else if (backLL.targetInView()) {
            return backLL.getTagTx();
        } else {
            return 0;
        }
    }

    public String getCageToClimb() {
        Pose2d robotPose = frontLL.getMegaTag2_Pose2d();
        double[] cageDiffs = new double[3];

        if (Field.isBlue()) {
            cageDiffs[0] = Math.abs(robotPose.getY() - Units.inchesToMeters(286.779));
            cageDiffs[1] = Math.abs(robotPose.getY() - Units.inchesToMeters(242.855));
            cageDiffs[2] = Math.abs(robotPose.getY() - Units.inchesToMeters(199.947));

            if (indexOfSmallest(cageDiffs) == 0) {
                return "B1";
            } else if (indexOfSmallest(cageDiffs) == 1) {
                return "B2";
            } else if (indexOfSmallest(cageDiffs) == 2) {
                return "B3";
            } else {
                return "Nothing";
            }
        } else {
            cageDiffs[0] =
                    Math.abs(
                            Field.flipYifRed(robotPose.getY())
                                    - Field.flipYifRed(Units.inchesToMeters(286.779)));
            cageDiffs[1] =
                    Math.abs(
                            Field.flipYifRed(robotPose.getY())
                                    - Field.flipYifRed(Units.inchesToMeters(242.855)));
            cageDiffs[2] =
                    Math.abs(
                            Field.flipYifRed(robotPose.getY())
                                    - Field.flipYifRed(Units.inchesToMeters(199.947)));

            if (indexOfSmallest(cageDiffs) == 0) {
                return "R1";
            } else if (indexOfSmallest(cageDiffs) == 1) {
                return "R2";
            } else if (indexOfSmallest(cageDiffs) == 2) {
                return "R3";
            } else {
                return "Nothing";
            }
        }
    }

    public static double indexOfSmallest(double[] array) {
        int indexOfSmallest = 0;
        double smallestIndex = array[indexOfSmallest];
        for (int i = 0; i < array.length; i++) {
            if (array[i] <= smallestIndex) {
                smallestIndex = array[i];
                indexOfSmallest = i;
            }
        }
        return indexOfSmallest;
    }

    /**
     * Gets a field-relative position for the score to the reef the robot should align, adjusted for
     * the robot's movement.
     *
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    // public Translation2d getAdjustedReefPos() {

    //     int reefID = closestReefFace(); // must call closestReefFace before this method gets
    // passed
    //     Pose2d[] reefFaces = Field.Reef.getCenterFaces();
    //     double NORM_FUDGE = 0.075;
    //     // double tunableNoteVelocity = 1;
    //     // double tunableNormFudge = 0;
    //     // double tunableStrafeFudge = 1;
    //     // TODO: fudges may be subject to removal
    //     double tunableReefYFudge = 0.0;
    //     double tunableReefXFudge = 0.0;

    //     Translation2d robotPos = Robot.getSwerve().getRobotPose().getTranslation();
    //     Translation2d targetPose =
    //             Field.flipXifRed(reefFaces[reefID].getTranslation()); // given reef face
    //     double xDifference = Math.abs(robotPos.getX() - targetPose.getX());
    //     double spinYFudge =
    //             (xDifference < 5.8)
    //                     ? 0.05
    //                     : 0.8; // change spin fudge for score distances vs. feed distances

    //     ChassisSpeeds robotVel =
    //             Robot.getSwerve().getCurrentRobotChassisSpeeds(); // get current robot velocity

    //     double distance = robotPos.getDistance(reefFaces[fieldReefID].getTranslation());
    //     double normFactor =
    //             Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < NORM_FUDGE
    //                     ? 0.0
    //                     : Math.abs(
    //                             MathUtil.angleModulus(
    //
    // robotPos.minus(targetPose).getAngle().getRadians()
    //                                                     - Math.atan2(
    //                                                             robotVel.vyMetersPerSecond,
    //                                                             robotVel.vxMetersPerSecond))
    //                                     / Math.PI);

    //     double x =
    //             reefFaces[fieldReefID].getX()
    //                     + (Field.isBlue() ? tunableReefXFudge : -tunableReefXFudge);
    //     // - (robotVel.vxMetersPerSecond * (distance / tunableNoteVelocity));
    //     //      * (1.0 - (tunableNormFudge * normFactor)));
    //     double y =
    //             reefFaces[fieldReefID].getY()
    //                     + (Field.isBlue() ? -spinYFudge : spinYFudge)
    //                     + tunableReefYFudge;
    //     // - (robotVel.vyMetersPerSecond * (distance / tunableNoteVelocity));
    //     //       * tunableStrafeFudge);
    //     return new Translation2d(x, y);
    // }

    // ------------------------------------------------------------------------------
    // VisionStates Commands
    // ------------------------------------------------------------------------------

    /** Set all Limelights to blink */
    public Command blinkLimelights() {
        Telemetry.print("Vision.blinkLimelights", PrintPriority.HIGH);
        return startEnd(
                        () -> {
                            for (Camera limelight : allLimelights) {
                                limelight.blinkLEDs();
                            }
                        },
                        () -> {
                            for (Camera limelight : allLimelights) {
                                limelight.setLEDMode(false);
                            }
                        })
                .withName("Vision.blinkLimelights");
    }

    /** Only blinks left limelight */
    public Command solidLimelight() {
        return startEnd(() -> frontLL.setLEDMode(true), () -> frontLL.setLEDMode(false))
                .withName("Vision.solidLimelight");
    }
}
