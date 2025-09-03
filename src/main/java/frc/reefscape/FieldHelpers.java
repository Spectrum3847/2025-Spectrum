package frc.reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.reefscape.Field.Reef;
import frc.reefscape.offsets.HomeOffsets;
import frc.robot.Robot;

public class FieldHelpers {

    private static Zones zones = new Zones();
    // private static final HomeOffsets offsets = new HomeOffsets();
    private static final HomeOffsets offsets = new HomeOffsets();

    // -----------------------------------------------------------------------
    // Field Helper Methods
    // -----------------------------------------------------------------------

    /* Methods to flip robot pose */

    public static double flipAngle(double angle) {
        return (angle + 180) % 360;
    }

    public static Rotation2d flipAngle(Rotation2d angle) {
        return angle.rotateBy(Rotation2d.fromDegrees(180));
    }

    public static double flipAngleIfRed(double blue) {
        if (Field.isRed()) {
            return (blue + 180) % 360;
        }
        return blue;
    }

    public static Rotation2d flipAngleIfRed(Rotation2d blue) {
        if (Field.isRed()) {
            return blue.rotateBy(Rotation2d.fromDegrees(180));
        }
        return blue;
    }

    public static Translation2d flipIfRed(Translation2d blue) {
        return new Translation2d(flipXifRed(blue.getX()), flipYifRed(blue.getY()));
    }

    public static Translation3d flipIfRed(Translation3d blue) {
        return new Translation3d(flipXifRed(blue.getX()), flipYifRed(blue.getY()), blue.getZ());
    }

    public static Pose2d flipIfRed(Pose2d red) {
        return new Pose2d(flipIfRed(red.getTranslation()), flipAngleIfRed(red.getRotation()));
    }

    public static Translation2d flipIfRedSide(Translation2d red) {
        if (Zones.blueFieldSide.getAsBoolean()) {
            return red;
        }
        return new Translation2d(flipX(red.getX()), flipY(red.getY()));
    }

    public static Pose2d flipIfRedSide(Pose2d red) {
        if (Zones.blueFieldSide.getAsBoolean()) {
            return red;
        }
        return new Pose2d(
                flipIfRedSide(new Translation2d(red.getX(), red.getY())),
                flipAngle(red.getRotation()));
    }

    public static double flipX(double xCoordinate) {
        return Field.fieldLength - xCoordinate;
    }

    public static double flipY(double yCoordinate) {
        return Field.fieldWidth - yCoordinate;
    }

    // If we are red flip the x pose to the other side of the field
    public static double flipXifRed(double xCoordinate) {
        if (Field.isRed()) {
            return Field.fieldLength - xCoordinate;
        }
        return xCoordinate;
    }

    // If we are red flip the y pose to the other side of the field
    public static double flipYifRed(double yCoordinate) {
        if (Field.isRed()) {
            return Field.fieldWidth - yCoordinate;
        }
        return yCoordinate;
    }

    /**
     * Normalizes an angle to the range [-π, π).
     *
     * @param angle The angle in radians.
     * @return The normalized angle.
     */
    private static double normalizeAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle >= Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static boolean poseOutOfField(Pose2d pose2D) {
        double x = pose2D.getX();
        double y = pose2D.getY();
        return (x <= 0 || x >= Field.fieldLength) || (y <= 0 || y >= Field.fieldWidth);
    }

    public static boolean poseOutOfField(Pose3d pose3D) {
        return poseOutOfField(pose3D.toPose2d());
    }

    // -----------------------------------------------------------------------
    // Cage Helper Methods
    // -----------------------------------------------------------------------

    public static int indexOfSmallest(double[] array) {
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

    // -----------------------------------------------------------------------
    // Reef Helper Methods
    // -----------------------------------------------------------------------

    /* Tag ID methods */

    /**
     * Converts an index to a reef tag ID
     *
     * @param index
     * @return
     */
    public static int indexToReefTagID(int index) {
        return index + 17;
    }

    /**
     * Converts a given Reef Tag Id into index form for center faces to pull from
     *
     * @param tagID
     * @return
     */
    public static int blueReefTagIDToIndex(int tagID) {

        // blue reef indexer
        if (tagID < 17 || tagID > 22 || tagID < 0) {
            return -1;
        }

        return tagID - 17;
    }

    /**
     * Converts a blue reef tag ID to a red reef tag ID
     *
     * @param blueTagID
     * @return
     */
    public static int blueToRedTagID(int blueTagID) {
        switch (blueTagID) {
            case 17:
                return 8;
            case 18:
                return 7;
            case 19:
                return 6;
            case 20:
                return 11;
            case 21:
                return 10;
            case 22:
                return 9;
            default:
                return blueTagID;
        }
    }

    public static int redToBlueTagID(int redTagID) {
        switch (redTagID) {
            case 8:
                return 17;
            case 7:
                return 18;
            case 6:
                return 19;
            case 11:
                return 20;
            case 10:
                return 21;
            case 9:
                return 22;
            default:
                return redTagID;
        }
    }

    /**
     * Returns the reef tag ID based on the robot's pose
     *
     * @param pose
     * @return
     */
    public static int getReefZoneTagID(Pose2d pose) {
        pose = flipIfRedSide(pose);
        int tag = indexToReefTagID(getReefZone(pose));

        if (!Zones.blueFieldSide.getAsBoolean()) {
            tag = blueToRedTagID(tag);
        }

        SmartDashboard.putNumber("Target ID getReefZone: ", tag);
        return tag;
    }

    /* Reef pose methods */

    /**
     * Returns the reef index zone based on the robot's pose changed to blue pose including the
     * center is consistently blue center
     *
     * @param pose
     * @return
     */
    public static int getReefZone(Pose2d pose) {
        Translation2d point = pose.getTranslation();
        Translation2d relativePoint = point.minus(Reef.getCenter());
        double angle = Math.atan2(relativePoint.getX(), relativePoint.getY()); // Standard atan2
        double distance = relativePoint.getNorm();

        // Normalize angle to be between 0 and 2*PI
        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        // Check if the point is within the 4.5 meters radius
        if (distance > 4.5) {
            // System.out.println("Distance Error");
            return -1; // Outside the zones
        }

        // Determine the zone based on the angle
        double zoneAngle = Math.PI / 3; // 60 degrees per zone
        int index = (int) ((angle + Math.PI) / zoneAngle); // Convert angle to zone index

        return index % 6; // Modular for safety, definitely works without the modular just don't
        // remove it
    }

    public static Pose2d getOffsetPosition(int tagID, double offsetMeters, double offsetRadians) {

        // reefTagIDToIndex
        int faceIndex = blueReefTagIDToIndex(redToBlueTagID(tagID));

        if (faceIndex < 0 || faceIndex >= Reef.centerFaces.length) {
            System.out.println("Bad Face Index: " + faceIndex);
            return Robot.getSwerve().getRobotPose();
        }

        // System.out.println("Tag ID getOffsetPosition" + tagID);
        // System.out.println("Index: " + faceIndex);
        Pose2d face = flipIfRed(Reef.centerFaces[faceIndex]);
        // System.out.println("FaceX: " + Units.metersToInches(face.getX()));
        // System.out.println("FaceY: " + Units.metersToInches(face.getY()));
        // System.out.println("FaceAngle: " + face.getRotation().getDegrees());

        // currently, only heading is set to front for facing the reef face
        double rotation = normalizeAngle(offsetRadians); // for angle values
        boolean reverseChecker = reverseRotationBlue();

        double offsetChecker = 1; // for translation values

        if (Field.isRed()) {
            offsetChecker = -1;
        }

        if (reverseChecker && Field.isBlue()) {
            offsetChecker *= -1;
            rotation = normalizeAngle(rotation - Math.PI);
        }
        // Red checkers for when reverse: both rotation and translation
        if (reverseChecker && Field.isRed()) {
            offsetChecker *= -1;
            rotation = normalizeAngle(rotation);
        }
        if (!reverseChecker && Field.isRed()) {
            offsetChecker = -1;
            rotation = normalizeAngle(rotation - Math.PI);
        }

        Rotation2d rotationOffset = face.getRotation().rotateBy(new Rotation2d(rotation));

        // checks if the rotation is 0 since that means back is closer
        // if (rotation == 0) {
        //     offsetChecker = -1;
        // }

        // Calculate the perpendicular offset
        Translation2d offsetTranslation =
                (new Translation2d(-offsetMeters * offsetChecker, rotationOffset));

        // Apply the offset to the face's position
        Translation2d newTranslation = face.getTranslation().plus(offsetTranslation);
        // System.out.println("NEWX: " + newTranslation.getX());
        // System.out.println("NEWY: " + newTranslation.getY());
        // System.out.println("NewAngle: " + rotationOffset);
        return new Pose2d(newTranslation, rotationOffset);
    }

    /**
     * Converts a target angle into a reverse rotation if the back is closer; otherwise, returns the
     * original target angle for front heading.
     *
     * <p>variable robotAngle The current angle of the robot in radians. variable reefRotation The
     * rotation adjustment factor in radians. targetAngle The desired target angle in radians.
     *
     * @return true/false if robot heading is reversed to reef face
     */
    public static boolean reverseRotationBlue() {
        Pose2d robotPose = Robot.getSwerve().getRobotPose();

        int tagID = getReefZoneTagID(robotPose);
        int tagIndex;

        if (Zones.blueFieldSide.getAsBoolean()) { // (Field.isBlue()) {
            tagIndex = blueReefTagIDToIndex(tagID);
        } else {
            tagIndex = blueReefTagIDToIndex(redToBlueTagID(tagID));
        }

        if (tagID < 0 || tagID == 16 || tagIndex < 0) {
            return false;
        }

        double reefRotation = Reef.centerFaces[tagIndex].getRotation().getRadians();
        double targetAngle = getTagAngleOffset(tagID);

        // Adjust target angle based on reef rotation and normalize
        double adjustedTargetAngle = normalizeAngle(reefRotation + targetAngle);

        // Calculate front and back heading differences
        double robotAngle = robotPose.getRotation().getRadians();
        double robotTargetAngleToFront = Math.abs(normalizeAngle(adjustedTargetAngle - robotAngle));
        double robotTargetAngleToBack =
                Math.abs(normalizeAngle(adjustedTargetAngle - (robotAngle + Math.PI)));

        // Return the optimal rotation
        if (robotTargetAngleToBack < robotTargetAngleToFront) {
            return true;
        }
        return false;
    }

    /**
     * Returns the reef face pose based on the tag ID sent from either red or blue
     *
     * @param tagID
     * @return Pose2d of reef side
     */
    public static Pose2d getReefSideFromTagID(int faceIndex) {
        if (faceIndex < 0) {
            return Robot.getSwerve().getRobotPose();
        }
        Pose2d reefFacePose = Reef.centerFaces[blueReefTagIDToIndex(faceIndex)];

        if (Field.isRed()) {
            reefFacePose = flipIfRed(reefFacePose);
            return flipIfRed(reefFacePose);
        }

        return reefFacePose;
    }

    /**
     * Returns the score pose based on the reef tag ID received by the limelight
     *
     * @param blueReefTagID may also be a red reef tag ID that will later be converted from a blue
     *     reef tagID
     * @return
     */
    public static Pose2d getScorePoseFromTagID(int blueReefTagID) {
        if (blueReefTagID < 0 || blueReefTagID > 22 || blueReefTagID == 16) {
            return Robot.getSwerve().getRobotPose();
        }

        double offSetMeters = zones.getTagOffset(blueReefTagID);
        double offsetRadians = zones.getTagAngleOffset(blueReefTagID);

        return getOffsetPosition(blueReefTagID, offSetMeters, offsetRadians);
    }

    public static double offSetMeters(int tagID) {
        return zones.getTagOffset(tagID);
    }

    public static double getTagAngleOffset(int tagID) {
        return zones.getTagAngleOffset(tagID);
    }

    /**
     * Method gets
     *
     * @param tagID
     * @param distanceAway
     * @param centerOffset
     * @return
     */
    public static Pose2d getXYOffsetFromTag(int tagID, double distanceAway, double centerOffset) {
        Pose2d tagPose;

        if (Robot.getVision().getTagLayout() == null || tagID == -1) {
            return Robot.getSwerve().getRobotPose(); // Pose to where we are if the tag is invalid
        }
        tagPose = Robot.getVision().getTagLayout().getTagPose(tagID).get().toPose2d();

        Rotation2d rotationOffsetParallel =
                tagPose.getRotation().plus(new Rotation2d(offsets.getReefTagAngleOffset(tagID)));
        Rotation2d rotationOffsetPerpendicular = tagPose.getRotation().plus(new Rotation2d(90));

        Translation2d offsetPose =
                tagPose.getTranslation()
                        .minus(new Translation2d(centerOffset, rotationOffsetPerpendicular));

        offsetPose = offsetPose.minus(new Translation2d(distanceAway, rotationOffsetParallel));
        return new Pose2d(offsetPose, rotationOffsetParallel);
    }

    public static double getReefOffsetFromTagX() {
        return Robot.getVision().getReefOffsetFromTag().getX();
    }

    public static double getReefOffsetFromTagY() {
        return Robot.getVision().getReefOffsetFromTag().getY();
    }

    // ------------------------------------------------------------------------------
    // Calculation Functions
    // ------------------------------------------------------------------------------

    /**
     * Get the angle the robot should turn to based on the id the limelight is seeing.
     *
     * @return
     */
    public static double getReefTagAngle() {
        double[][] reefFrontAngles = {
            {17, 60}, {18, 0}, {19, -60}, {20, -120}, {21, 180}, {22, 120},
            {6, 120}, {7, 180}, {8, -120}, {9, -60}, {10, 0}, {11, 60}
        };

        int closestTag = Robot.getVision().getClosestTagID();
        boolean rearTag = Robot.getVision().isRearTagClosest();

        if (closestTag <= 0) {
            Pose2d currentPose = Robot.getSwerve().getRobotPose();
            int tagID = FieldHelpers.getReefZoneTagID(currentPose);
            closestTag = tagID;
            rearTag = false;
        }

        for (int i = 0; i < reefFrontAngles.length; i++) {
            if (closestTag == reefFrontAngles[i][0]) {
                if (rearTag || !Robot.getSwerve().frontClosestToAngle(reefFrontAngles[i][1])) {
                    return Math.toRadians(reefFrontAngles[i][1] + 180);
                }
                return Math.toRadians(reefFrontAngles[i][1]);
            }
        }

        // Return current angle if no tag is found
        return Robot.getSwerve().getRobotPose().getRotation().getRadians();
    }
}
