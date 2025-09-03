package frc.reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.offsets.WorldChampsOffsets;
import frc.robot.Robot;
import frc.robot.swerve.Swerve;
import lombok.Getter;

public class Zones {
    @Getter private static final double atReefXYTolerance = Units.inchesToMeters(0.7); // 0.55

    @Getter
    private static final double atReefRotationTolerance = Units.degreesToRadians(0.35); // rads

    @Getter private static final double netAlgaeX = 9.618; // red coordinates
    @Getter private static final double netAlgaeZoneTolerance = 0.3;

    private static final Swerve swerve = Robot.getSwerve();
    // private static final HomeOffsets offsets = new HomeOffsets();
    private static final WorldChampsOffsets offsets = new WorldChampsOffsets();
    private static final double reefRangeRadius =
            Field.Reef.apothem + Field.Reef.faceToZoneLine + Units.inchesToMeters(30);

    public static final Trigger blueFieldSide = swerve.inXzone(0, Field.getHalfLength());
    public static final Trigger opponentFieldSide =
            new Trigger(() -> blueFieldSide.getAsBoolean() != Field.isBlue());

    public static final Trigger topLeftZone =
            swerve.inXzoneAlliance(Field.Reef.getCenter().getX(), Field.getHalfLength())
                    .and(
                            swerve.inYzoneAlliance(
                                    Field.Reef.getCenter().getY(), Field.getFieldWidth()));
    public static final Trigger topRightZone =
            swerve.inXzoneAlliance(Field.Reef.getCenter().getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.getCenter().getY()));
    public static final Trigger bottomLeftZone =
            swerve.inXzoneAlliance(0, Field.Reef.getCenter().getX())
                    .and(
                            swerve.inYzoneAlliance(
                                    Field.Reef.getCenter().getY(), Field.getFieldWidth()));
    public static final Trigger bottomRightZone =
            swerve.inXzoneAlliance(0, Field.Reef.getCenter().getX())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.getCenter().getY()));

    public static final Trigger netAlgaeZone =
            // swerve.inXzoneAlliance(
            //                 3 * Field.getHalfLength() / 4,
            //                 Field.getHalfLength()
            //                         - Units.inchesToMeters(24)
            //                         - swerve.getConfig().getRobotLength() / 2)
            //         .and(topLeftZone);
            swerve.inXzone(netAlgaeX - netAlgaeZoneTolerance, netAlgaeX + netAlgaeZoneTolerance)
                    .or(
                            swerve.inXzone(
                                    (Field.getFieldLength() - netAlgaeX) - netAlgaeZoneTolerance,
                                    (Field.getFieldLength() - netAlgaeX) + netAlgaeZoneTolerance));

    public static final Trigger isCloseToReef =
            new Trigger(() -> Zones.withinReefRange(reefRangeRadius));

    // -------------------------------------------------------------
    // Reef Offsets Helper
    // -------------------------------------------------------------

    /**
     * Depending on the offsets file used, change the tag offsets here.
     *
     * <p>getTagOffset automatically checks if the offset taken is < 0 or greater than the given tag
     * limit. If the tag is blue (> 16), it subtracts 17 from the given tag
     *
     * @param tag
     * @return
     */
    public double getTagOffset(int tag) {
        return offsets.getReefTagDistanceOffset(tag);
    }

    public double getTagAngleOffset(int tag) {
        return offsets.getReefTagAngleOffset(tag);
    }

    /**
     * Calls on getReefZoneTagID with the given robot swerve position. Automatically has a check on
     * reefTagID and returns robot pose if the robot is not in a target reef zone
     *
     * @return Target Reef Pose
     */
    Pose2d getScoreReefPose() {
        int reefTagID = FieldHelpers.getReefZoneTagID(Robot.getSwerve().getRobotPose());
        if (reefTagID < 0) {
            return Robot.getSwerve().getRobotPose();
        }

        SmartDashboard.putNumber("Target Reef ID: ", reefTagID);
        return FieldHelpers.getScorePoseFromTagID(reefTagID);
    }

    /**
     * Calls ScoreReefPose to get Pose of Target Reef
     *
     * @return X of Target Reef
     */
    public double getScoreReefPoseX() {
        double reefX = getScoreReefPose().getX();

        SmartDashboard.putNumber("TargetReefX", reefX);
        return reefX;
    }

    /**
     * Calls ScoreReefPose to get Pose of Target Reef
     *
     * @return Y of Target Reef
     */
    public double getScoreReefPoseY() {
        double reefY = getScoreReefPose().getY();

        SmartDashboard.putNumber("TargetReefY", reefY);
        return reefY;
    }

    /**
     * Calls ScoreReefPose to get Pose of Target Reef
     *
     * @return Returns Target Reef Pose Angle value based on robot heading hence adding PI is needed
     *     since the target reef face is pi radians from robot heading
     */
    public double getScoreReefPoseAngle() {
        double reefAngle = getScoreReefPose().getRotation().getRadians();

        SmartDashboard.putNumber("TargetReefAngle", reefAngle);
        return reefAngle;
    }

    public static boolean atReef() {
        // SwerveConfig config = Robot.getSwerve().getConfig();
        Pose2d robotPose = Robot.getSwerve().getRobotPose();
        double heading = robotPose.getRotation().getDegrees();
        double flippedHeading;
        if (heading > 0) {
            flippedHeading = heading - 180;
        } else {
            flippedHeading = heading + 180;
        }

        double goalX = FieldHelpers.getReefOffsetFromTagX();
        double goalY = FieldHelpers.getReefOffsetFromTagY();
        double goalAngle = Math.toDegrees(FieldHelpers.getReefTagAngle());

        // System.out.println("Pose Angle: " + heading);
        // System.out.println("Target Angle: " + goalAngle);
        // System.out.println(
        //         "Rotation diff: " + Robot.getSwerve().getRotationDifference(heading, goalAngle));
        if (Robot.getSwerve().getRotationDifference(heading, goalAngle)
                        > Math.toDegrees(getAtReefRotationTolerance())
                && Robot.getSwerve().getRotationDifference(flippedHeading, goalAngle)
                        > Math.toDegrees(getAtReefRotationTolerance())) {
            return false;
        }
        // System.out.println("Rotation good");

        // System.out.println("X diff: " + Math.abs(robotPose.getX() - goalX));
        if (Math.abs(robotPose.getX() - goalX) > getAtReefXYTolerance()) {
            return false;
        }
        // System.out.println("X good");

        // System.out.println("Y diff: " + Math.abs(robotPose.getY() - goalY));
        if (Math.abs(robotPose.getY() - goalY) > getAtReefXYTolerance()) {
            return false;
        }
        // System.out.println("Y good");

        return true;
    }

    public static boolean withinReefRange(double range) {
        Translation2d reefCenter = FieldHelpers.flipIfRedSide(Field.Reef.getCenter());
        Pose2d robotPose = Robot.getSwerve().getRobotPose();

        double distance = reefCenter.getDistance(robotPose.getTranslation());

        if (distance < range) {
            return true;
        }
        return false;
    }
}
