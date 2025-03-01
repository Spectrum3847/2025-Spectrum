package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.reefscape.Field;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.Trio;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import java.util.ArrayList;
import lombok.Getter;
import lombok.Setter;

public class Vision extends SubsystemBase {

    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String FRONT_LL = "limelight-front";
        public static final LimelightConfig FRONT_CONFIG =
                new LimelightConfig(FRONT_LL)
                        .withTranslation(0, -27.36, -8.04)
                        .withRotation(0, Math.toRadians(.5), Math.toRadians(-7.96));

        public static final String BACK_LL = "limelight-back";
        public static final LimelightConfig BACK_CONFIG =
                new LimelightConfig(BACK_LL)
                        .withTranslation(0, 0.5, 0.5)
                        .withRotation(0, Math.toRadians(15), 0);

        // TODO: Limelight config needs to be updated to actual position on robot

        /* Pipeline configs */
        public static final int frontTagPipeline = 0;
        public static final int backTagPipeline = 1;

        /* Pose Estimation Constants */

        // Increase these numbers to trust global measurements from vision less. (uses a matrix)
        public static final double VISION_REJECT_Distance = 1.8;

        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

        /* Vision Command Configs */
        // TODO: alignToTag vision etc.
    }

    /** Limelights */
    @Getter
    public final Limelight frontLL =
            new Limelight(
                    VisionConfig.FRONT_LL,
                    VisionConfig.frontTagPipeline,
                    VisionConfig.FRONT_CONFIG);

    public final LimelightLogger frontLogger = new LimelightLogger("front", frontLL);

    public final Limelight backLL =
            new Limelight(
                    VisionConfig.BACK_LL, VisionConfig.backTagPipeline, VisionConfig.BACK_CONFIG);

    public final LimeLightLogger backLogger = new LimelightLogger("back", backLL);

    public final Limelight[] allLimelights = {frontLL, backLL};

    private final DecimalFormat df = new DecimalFormat();

    @Getter @Setter boolean isIntegrating = false;

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    @Getter boolean isAiming = false;

    public Vision() {
        setName("vision");

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
        }
    }

    @Override
    public void periodic() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        for (Limelight limelight : allLimelights) {
            limelight.setRobotOrientation(yaw);

            if (DriverStation.isAutonomousEnabled() && limelight.targetInView()) {
                Pose3d botpose3D = limelight.getRawPose3d();
                Pose2d megaPose2d = limelight.getMegaPose2d();
                double timeStamp = limelight.getRawPoseTimestamp();
                Pose2d integratablePose =
                        new Pose2d(megaPose2d.getTranslation(), botpose3D.toPose2d().getRotation());
                autonPoses.add(Trio.of(botpose3D, integratablePose, timeStamp));
            }
        }
        try {
            isIntegrating = false;
            // Will NOT run in auto
            if (DriverStation.isTeleopEnabled()) {

                // choose LL with best view of tags and integrate from only that camera
                Limelight bestLimelight = getBestLimelight();
                for (Limelight limelight : allLimelights) {
                    LimeLightLogger logger = new LimeLightLogger(limelight.name, limelight);
                    if (limelight.getCameraName() == bestLimelight.getCameraName()) {
                        addFilteredVisionInput(bestLimelight);
                    } else {
                        limelight.sendInvalidStatus("not best rejection");
                    }
                    logger.getMegaPose();
                    logger.getPose();
                    logger.getLogStatus();
                    isIntegrating |= limelight.isIntegrating();
                }
                getDistanceToReefFromRobot();
            }

        } catch (Exception e) {
            Telemetry.print("Vision pose not present but tried to access it");
        }
    }

    private void addFilteredVisionInput(Limelight ll) {
        double xyStds = 1000;
        double degStds = 1000;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getRawPoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose3d botpose3D = ll.getRawPose3d();
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d megaPose2d = ll.getMegaPose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();

            // distance from current pose to vision estimated pose
            double poseDifference =
                    Robot.getSwerve()
                            .getRobotPose()
                            .getTranslation()
                            .getDistance(botpose.getTranslation());

            // /* rejections */
            // // reject pose if individual tag ambiguity is too high
            // ll.setTagStatus("");
            // for (RawFiducial tag : tags) {
            //     // search for highest ambiguity tag for later checks
            //     if (highestAmbiguity == 2) {
            //         highestAmbiguity = tag.ambiguity;
            //     } else if (tag.ambiguity > highestAmbiguity) {
            //         highestAmbiguity = tag.ambiguity;
            //     }
            //     // log ambiguities
            //     ll.setTagStatus("Tag " + tag.id + ": " + tag.ambiguity);
            //     // ambiguity rejection check
            //     if (tag.ambiguity > 0.9) {
            //         ll.sendInvalidStatus("ambiguity rejection");
            //         return;
            //     }
            // }
            // if (Field.poseOutOfField(botpose3D)) {
            //     // reject if pose is out of the field
            //     ll.sendInvalidStatus("bound rejection");
            //     return;
            // } else if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 1.6) {
            //     // reject if we are rotating more than 0.5 rad/s
            //     ll.sendInvalidStatus("rotation rejection");
            //     return;
            // } else if (Math.abs(botpose3D.getZ()) > 0.25) {
            //     // reject if pose is .25 meters in the air
            //     ll.sendInvalidStatus("height rejection");
            //     return;
            // } else if (Math.abs(botpose3D.getRotation().getX()) > 5
            //         || Math.abs(botpose3D.getRotation().getY()) > 5) {
            //     // reject if pose is 5 degrees titled in roll or pitch
            //     ll.sendInvalidStatus("roll/pitch rejection");
            //     return;
            // } else if (targetSize <= 0.025) {
            //     ll.sendInvalidStatus("size rejection");
            //     return;
            // }
            // /* integrations */
            // // if almost stationary and extremely close to tag
            // else if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
            //         && targetSize > 0.4) {
            //     ll.sendValidStatus("Stationary close integration");
            //     xyStds = 0.1;
            //     degStds = 0.1;
            // } else if (multiTags && targetSize > 0.05) {
            //     ll.sendValidStatus("Multi integration");
            //     xyStds = 0.25;
            //     degStds = 8;
            //     if (targetSize > 0.09) {
            //         ll.sendValidStatus("Strong Multi integration");
            //         xyStds = 0.1;
            //         degStds = 0.1;
            //     }
            // } else if (targetSize > 0.8 && poseDifference < 0.5) {
            //     ll.sendValidStatus("Close integration");
            //     xyStds = 0.5;
            //     degStds = 16;
            // } else if (targetSize > 0.1 && poseDifference < 0.3) {
            //     ll.sendValidStatus("Proximity integration");
            //     xyStds = 2.0;
            //     degStds = 999999;
            // } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
            //     ll.sendValidStatus("Stable integration");
            //     xyStds = 0.5;
            //     degStds = 999999;
            // } else {
            //     ll.sendInvalidStatus("catch rejection: " + df.format(poseDifference) + " poseDiff");
            //     return;
            // }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = 15;
            }

            if (robotSpeed.omegaRadiansPerSecond >= 0.5) {
                degStds = 15;
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            Robot.getSwerve()
                    .setVisionMeasurementStdDevs(
                            VecBuilder.fill(
                                    VisionConfig.VISION_STD_DEV_X,
                                    VisionConfig.VISION_STD_DEV_Y,
                                    VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose2d.getTranslation(), botpose.getRotation());
            Robot.getSwerve().addVisionMeasurement(integratedPose, timeStamp);
        } else {
            ll.setTagStatus("no tags");
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    // TODO:Auton Reset pose to Vision method

    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

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
            Pose2d pose = Robot.getSwerve().getRobotPose();
            if (Field.poseOutOfField(botpose3D)
                    || Math.abs(botpose3D.getZ()) > 0.25
                    || (Math.abs(botpose3D.getRotation().getX()) > 5
                            || Math.abs(botpose3D.getRotation().getY()) > 5)) { // when has bad
                Telemetry.log("Pose bad", reject);
                reject = true;
            }
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

            // track STDs
            VisionConfig.VISION_STD_DEV_X = 0.001;
            VisionConfig.VISION_STD_DEV_Y = 0.001;
            VisionConfig.VISION_STD_DEV_THETA = 0.001;

            // Posts Current X,Y, and Angle (Theta) values
            double[] visionPose = {
                botpose.getX(), botpose.getY(), botpose.getRotation().getDegrees()
            };
            Telemetry.log("Vision Pose", visionPose);
            // Telemetry.log("Vision X: ", botpose.getX());
            // Telemetry.log("Vision Y: ", botpose.getY());
            // Telemetry.log("Vision Theta: ", botpose.getRotation().getDegrees());

            Robot.getSwerve()
                    .setVisionMeasurementStdDevs(
                            VecBuilder.fill(
                                    VisionConfig.VISION_STD_DEV_X,
                                    VisionConfig.VISION_STD_DEV_Y,
                                    VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Robot.getSwerve().addVisionMeasurement(integratedPose, poseTimestamp);
            pose = Robot.getSwerve().getRobotPose();
            // Gets updated pose of x, y, and theta values
            visionPose = new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
            Telemetry.log("Vision Pose", visionPose);

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

    // ------------------------------------------------------------------------------
    // Logging
    // ------------------------------------------------------------------------------

    /** Tracks position with Limelight using current logger (DogLog) to record data */
    public static class LimelightLogger {
        private final Limelight limelight;
        private String name;

        public LimelightLogger(String name, Limelight limelight) {
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
            Telemetry.log("Vision " + name + " Pose", limelight.getRawPose3d().toPose2d());
            return limelight.getRawPose3d().toPose2d();
        }

        public Pose2d getMegaPose() {
            Telemetry.log("Vision " + name + " MegaPose", limelight.getMegaPose2d());
            return limelight.getMegaPose2d();
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

    // ------------------------------------------------------------------------------
    // Config
    // ------------------------------------------------------------------------------

    /**
     * CommandConfig used to create a PIDController and for vision commands using other subsystems
     * (i.e. swerve)
     */
    public static class CommandConfig {
        public double kp;
        public double tolerance;
        public double maxOutput;
        public double error;
        public int pipelineIndex;
        public Limelight limelight;
        /* For Drive-To commands */
        public CommandConfig alignCommand;
        public double verticalSetpoint; // numbers get small as the cone gets closer
        public double verticalMaxView;

        public void configKp(double kp) {
            this.kp = kp;
        }

        public void configTolerance(double tolerance) {
            this.tolerance = tolerance;
        }

        public void configMaxOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void configError(double error) {
            this.error = error;
        }

        public void configPipelineIndex(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        public void configLimelight(Limelight limelight) {
            this.limelight = limelight;
        }

        public void configVerticalSetpoint(double verticalSetpoint) {
            this.verticalSetpoint = verticalSetpoint;
        }

        public void configVerticalMaxView(double verticalMaxView) {
            this.verticalMaxView = verticalMaxView;
        }

        public void configAlignCommand(CommandConfig alignCommand) {
            this.alignCommand = alignCommand;
        }

        public CommandConfig() {}
    }

    // ------------------------------------------------------------------------------
    // Calculation Functions
    // ------------------------------------------------------------------------------

    /**
     * REQUIRES ACCURATE POSE ESTIMATION. Uses trigonometric functions to calculate the angle
     * between the robot heading and the angle required to face coral faces.
     *
     * @return angle between robot heading and the closest reef face in degrees
     */
    private int fieldReefID;

    // public double getThetaToReefFace() {
    //     double closestReefFace = closestReefFace();
    //     Translation2d robot2d = Robot.getSwerve().getRobotPose().getTranslation();
    //     fieldReefID = -1;
    //     for(int i = 18; i<23; i++){
    //         if(closestReefFace == i){
    //             fieldReefID = i;
    //         } else if(closestReefFace == 17 ){
    //             fieldReefID = 5;
    //         }
    //     }
    //     if (fieldReefID == -1) {
    //         return -1;
    //     }

    //     Translation2d reefFace = Field.Reef.centerFaces[fieldReefID].getTranslation();

    //     double angleBetweenRobotandReefFace =
    //         MathUtil.angleModulus(
    //             reefFace.minus(robot2d).getAngle().getRadians());

    //     return angleBetweenRobotandReefFace;

    //     //runs closestReefFace to get the closest reef face id
    // }

    public double getAdjustedThetaToReefFace() {
        Translation2d reefFace = getAdjustedReefPos();
        Translation2d robot2d = Robot.getSwerve().getRobotPose().getTranslation();
        double angleBetweenRobotandReefFace =
                MathUtil.angleModulus(reefFace.minus(robot2d).getAngle().getRadians());

        return angleBetweenRobotandReefFace;
    }

    public int closestReefFace() {
        double[] reefdistance = getDistanceToReefFromRobot();
        if (reefdistance[0] > -1) {
            RawFiducial[] tags = frontLL.getRawFiducial();
            if (Field.isRed()) {
                int closestReef = tags[0].id;
                for (RawFiducial tag : tags) {
                    if (tag.distToRobot < tags[closestReef].distToRobot) {
                        closestReef = tag.id;
                    }
                }
                Telemetry.print("Closest Reef Face: " + closestReef, PrintPriority.HIGH);
                return closestReef; // red reef tag
            } else {
                int closestReef = tags[0].id;
                for (RawFiducial tag : tags) {
                    if (tag.distToRobot < tags[closestReef].distToRobot) {
                        closestReef = tag.id;
                    }
                }
                Telemetry.print("Closest Reef Face: " + closestReef, PrintPriority.HIGH);
                return closestReef; // blue reef tag
            }
        }
        Telemetry.print("No reef tag found", PrintPriority.HIGH);
        return -1; // no reef tag found
    }

    /** Returns the distance from the reef in meters, adjusted for the robot's movement. */
    public double[] getDistanceToReefFromRobot() {
        RawFiducial[] frontTags = frontLL.getRawFiducial();
        RawFiducial[] rightTags = backLL.getRawFiducial();

        ArrayList<Integer> ValidReefFaceIDsRed = new ArrayList<Integer>();
        for (int i = 6; i < 12; i++) {
            ValidReefFaceIDsRed.add(i);
        }
        ArrayList<Integer> ValidReefFaceIDsBlue = new ArrayList<Integer>();
        for (int i = 17; i < 22; i++) {
            ValidReefFaceIDsBlue.add(i);
        }

        // ArrayList<Double> seenReefFaces = new ArrayList<Double>();
        double[] seenReefFaces = new double[12];
        for (RawFiducial tag : frontTags) {
            if (ValidReefFaceIDsRed.contains(tag.id) || ValidReefFaceIDsBlue.contains(tag.id)) {
                seenReefFaces[0] = tag.distToCamera;
            }
        }

        for (RawFiducial tag : rightTags) {
            if (ValidReefFaceIDsRed.contains(tag.id) || ValidReefFaceIDsBlue.contains(tag.id)) {
                // seenReefFaces.add(tag.distToCamera);
            }
        }

        SmartDashboard.putNumber("GetDistanceSeenReefFace", seenReefFaces[0]);
        return seenReefFaces;
    }

    /**
     * Gets a field-relative position for the score to the reef the robot should align, adjusted for
     * the robot's movement.
     *
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getAdjustedReefPos() {

        int reefID = closestReefFace(); // must call closestReefFace before this method gets passed
        Pose2d[] reefFaces = Field.Reef.getCenterFaces();
        double NORM_FUDGE = 0.075;
        double tunableNoteVelocity = 1;
        double tunableNormFudge = 0;
        double tunableStrafeFudge = 1;
        double tunableReefYFudge = 0.0;
        double tunableReefXFudge = 0.0;

        Translation2d robotPos = Robot.getSwerve().getRobotPose().getTranslation();
        Translation2d targetPose =
                Field.flipXifRed(reefFaces[reefID].getTranslation()); // given reefface
        double xDifference = Math.abs(robotPos.getX() - targetPose.getX());
        double spinYFudge =
                (xDifference < 5.8)
                        ? 0.05
                        : 0.8; // change spin fudge for score distances vs. feed distances

        ChassisSpeeds robotVel =
                Robot.getSwerve().getCurrentRobotChassisSpeeds(); // get current robot velocity

        double distance = robotPos.getDistance(reefFaces[fieldReefID].getTranslation());
        double normFactor =
                Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < NORM_FUDGE
                        ? 0.0
                        : Math.abs(
                                MathUtil.angleModulus(
                                                robotPos.minus(targetPose).getAngle().getRadians()
                                                        - Math.atan2(
                                                                robotVel.vyMetersPerSecond,
                                                                robotVel.vxMetersPerSecond))
                                        / Math.PI);

        double x =
                reefFaces[fieldReefID].getX()
                        + (Field.isBlue() ? tunableReefXFudge : -tunableReefXFudge);
        // - (robotVel.vxMetersPerSecond * (distance / tunableNoteVelocity));
        //      * (1.0 - (tunableNormFudge * normFactor)));
        double y =
                reefFaces[fieldReefID].getY()
                        + (Field.isBlue() ? -spinYFudge : spinYFudge)
                        + tunableReefYFudge;
        // - (robotVel.vyMetersPerSecond * (distance / tunableNoteVelocity));
        //       * tunableStrafeFudge);
        return new Translation2d(x, y);
    }

    // Executes the alignToVisionTarget command

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
        return startEnd(
                        () -> {
                            frontLL.setLEDMode(true);
                        },
                        () -> {
                            frontLL.setLEDMode(false);
                        })
                .withName("Vision.solidLimelight");
    }
}
