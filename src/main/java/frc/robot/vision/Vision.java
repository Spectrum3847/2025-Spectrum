package frc.robot.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.reefscape.FieldHelpers;
import frc.reefscape.offsets.HomeOffsets;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.io.IOException;
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

        @Getter final String backLL = "limelight-back";

        @Getter
        final LimelightConfig backConfig =
                new LimelightConfig(backLL)
                        .withTranslation(-0.215, 0.0, 0.188)
                        .withRotation(0, Math.toRadians(28), Math.toRadians(180));

        /* Pipeline configs */
        @Getter final int frontTagPipeline = 0;
        @Getter final int backTagPipeline = 0;

        /* Pose Estimation Constants */

        @Getter double visionStdDevX = 0.5;
        @Getter double visionStdDevY = 0.5;
        @Getter double visionStdDevTheta = 0.2;

        @Getter
        final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta);
    }

    /** Limelights */
    @Getter public final Limelight frontLL;

    private static HomeOffsets offsets;

    public final Limelight backLL;

    public final Limelight[] allLimelights;

    private final DecimalFormat df = new DecimalFormat();

    @Getter @Setter private boolean isIntegrating = false;

    @Getter private boolean isAiming = false;

    int[] blueTags = {17, 18, 19, 20, 21, 22};
    int[] redTags = {6, 7, 8, 9, 10, 11};

    @Getter private static AprilTagFieldLayout tagLayout;

    private static final HomeOffsets homeOffsets = new HomeOffsets();

    private VisionConfig config;

    public Vision(VisionConfig config) {
        this.config = config;

        frontLL = new Limelight(config.frontLL, config.frontTagPipeline, config.frontConfig);

        backLL = new Limelight(config.backLL, config.backTagPipeline, config.backConfig);

        allLimelights = new Limelight[] {frontLL, backLL};

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
            limelight.setIMUmode(1);
        }

        /* Get the April Tag Field Layout */
        try {
            tagLayout =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
        } catch (IOException e) {
            System.err.println(e);
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

        Robot.getField2d().getObject(frontLL.getCameraName());
        Robot.getField2d().getObject(backLL.getCameraName());
    }

    @Override
    public void periodic() {
        setLimeLightOrientation();
        disabledLimelightUpdates();
        enabledLimelightUpdates();
        autonLimelightUpdates();

        Robot.getField2d().getObject(frontLL.getCameraName()).setPose(getFrontMegaTag2Pose());
        Robot.getField2d().getObject(backLL.getCameraName()).setPose(getBackMegaTag2Pose());
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
        builder.addDoubleProperty("FrontTA", frontLL::getTagTA, null);
        builder.addDoubleProperty("FrontTagID", frontLL::getClosestTagID, null);
        builder.addDoubleProperty("BackTX", backLL::getTagTx, null);
        builder.addDoubleProperty("BackTA", backLL::getTagTA, null);
        builder.addDoubleProperty("BackTagID", backLL::getClosestTagID, null);
    }

    private void setLimeLightOrientation() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();

        for (Limelight limelight : allLimelights) {
            limelight.setRobotOrientation(yaw);
        }
    }

    private void disabledLimelightUpdates() {
        if (Util.disabled.getAsBoolean()) {
            for (Limelight limelight : allLimelights) {
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
            for (Limelight limelight : allLimelights) {
                limelight.setIMUmode(1);
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

    private void autonLimelightUpdates() {
        if (Util.autoMode.getAsBoolean() && RobotStates.poseUpdate.getAsBoolean()) {
            for (Limelight limelight : allLimelights) {
                limelight.setIMUmode(1);
            }
            try {
                addMegaTag2_VisionInputAuton(backLL);
            } catch (Exception e) {
                Telemetry.print("REAR MT2: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag2_VisionInputAuton(frontLL);
            } catch (Exception e) {
                Telemetry.print("FRONT MT2: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInputAuton(backLL, false);
            } catch (Exception e) {
                Telemetry.print("REAR MT1: Vision pose not present but tried to access it");
            }

            try {
                addMegaTag1_VisionInputAuton(frontLL, false);
            } catch (Exception e) {
                Telemetry.print("FRONT MT1: Vision pose not present but tried to access it");
            }
        }
    }

    @SuppressWarnings("all")
    private void addMegaTag1_VisionInput(Limelight ll, boolean integrateXY) {
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
            ll.setTagStatus("");
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
                    && targetSize > 4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 2) {
                ll.sendValidStatus("Strong Multi integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 0.2) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
                degStds = 8;
            } else if (targetSize > 2 && (mt1PoseDifference < 0.5)) {
                // Integrate if the target is very big and we are close to pose
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
                degStds = 999999;
            } else if (targetSize > 1 && (mt1PoseDifference < 0.25)) {
                // Integrate if we are very close to pose and target is large enough
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

            if (integrateXY) { // If we are disabled just use this pose
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
            ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    @SuppressWarnings("all")
    private void addMegaTag1_VisionInputAuton(Limelight ll, boolean integrateXY) {
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
            ll.setTagStatus("");
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
            if (targetSize > 4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 2) {
                ll.sendValidStatus("Strong Multi integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 0.2) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
                degStds = 8;
            } else if (targetSize > 2 && (mt1PoseDifference < 0.5)) {
                // Integrate if the target is very big and we are close to pose
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
                degStds = 999999;
            } else if (targetSize > 1 && (mt1PoseDifference < 0.25)) {
                // Integrate if we are very close to pose and target is large enough
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

            if (integrateXY) { // If we are disabled just use this pose
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
            ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    @SuppressWarnings("all")
    private void addMegaTag2_VisionInput(Limelight ll) {
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
                    && targetSize > 4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
            } else if (multiTags && targetSize > 2) {
                ll.sendValidStatus("Strong Multi integration");
                xyStds = 0.1;
            } else if (multiTags && targetSize > 0.2) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
            } else if (targetSize > 2 && (mt2PoseDifference < 0.5 || DriverStation.isDisabled())) {
                // Integrate if the target is very big and we are close to pose or disabled
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
            } else if (targetSize > 1 && (mt2PoseDifference < 0.25 || DriverStation.isDisabled())) {
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
            ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    @SuppressWarnings("all")
    private void addMegaTag2_VisionInputAuton(Limelight ll) {
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
            if (targetSize > 4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
            } else if (multiTags && targetSize > 2) {
                ll.sendValidStatus("Strong Multi integration");
                xyStds = 0.1;
            } else if (multiTags && targetSize > 0.1) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.25;
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
            ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    private boolean rejectionCheck(Pose2d pose, double targetSize) {
        /* rejections */
        if (FieldHelpers.poseOutOfField(pose)) {
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
    public Limelight getBestLimelight() {
        Limelight bestLimelight = frontLL;
        double bestScore = 0;
        for (Limelight limelight : allLimelights) {
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
        Limelight ll = getBestLimelight();
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
            if (FieldHelpers.poseOutOfField(botpose3D)) { // pose out of field
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
        for (Limelight limelight : allLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    public int getClosestTagID() {
        int closestTagIDFront = (int) frontLL.getClosestTagID();
        int closestTagIDBack = (int) backLL.getClosestTagID();

        if (closestTagIDFront == -1) {
            return closestTagIDBack;
        }
        return closestTagIDFront;
    }

    public boolean isRearTagClosest() {
        int closetTag = getClosestTagID();
        int closestTagIDBack = (int) backLL.getClosestTagID();
        return closetTag != -1 && closestTagIDBack == closetTag;
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

    public Pose2d getReefOffsetFromTag() {
        int closestTagID = Robot.getVision().getClosestTagID();

        if (closestTagID < 6 || closestTagID == 16 || closestTagID > 22) {
            closestTagID = FieldHelpers.getReefZoneTagID(Robot.getSwerve().getRobotPose());
            if (closestTagID < 0) {
                return Robot.getSwerve().getRobotPose();
            }
        }

        double reefTagDistanceOffset = offsets.getReefTagDistanceOffset(closestTagID);
        double reefTagCenterOffset = offsets.getReefTagCenterOffset(closestTagID);

        return FieldHelpers.getXYOffsetFromTag(
                closestTagID, reefTagDistanceOffset, reefTagCenterOffset);
    }

    // ------------------------------------------------------------------------------
    // VisionStates Commands
    // ------------------------------------------------------------------------------

    /** Set all Limelights to blink */
    public Command blinkLimelights() {
        Telemetry.print("Vision.blinkLimelights", PrintPriority.HIGH);
        return startEnd(
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.blinkLEDs();
                            }
                        },
                        () -> {
                            for (Limelight limelight : allLimelights) {
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
