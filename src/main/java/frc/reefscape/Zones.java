package frc.reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.swerve.Swerve;

public class Zones {

    private static final Swerve swerve = Robot.getSwerve();
    private static final HomeOffsets homeOffsets = new HomeOffsets();
    private static final StateChampsOffsets stateChampsOffsets = new StateChampsOffsets();
    private static final WorldsChampsOffsets worldsChampsOffsets = new WorldsChampsOffsets();

    public static final Trigger blueFieldSide = swerve.inXzone(0, Field.getHalfLength());

    public static final Trigger topLeftZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger topRightZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));
    public static final Trigger bottomLeftZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger bottomRightZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));

    public static final Trigger bargeZone =
            swerve.inXzoneAlliance(
                            3 * Field.getHalfLength() / 4,
                            Field.getHalfLength()
                                    - Units.inchesToMeters(24)
                                    - swerve.getConfig().getRobotLength() / 2)
                    .and(topLeftZone);

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
        double[][] tagOffsetsArray = homeOffsets.getReefTagOffsets();
        int indexOfTag = tag;
        if (tag < 0 || tag > 22) {
            return 0;
        }

        if (tag >= 17) {
            indexOfTag = indexOfTag - 17;
        }

        if (indexOfTag < 0 || indexOfTag > 16) {
            return 0;
        }

        // System.out.println("Tag Index: " + indexOfTag);
        return tagOffsetsArray[indexOfTag][1];
    }

    public double getTagAngleOffset(int tag) {
        double[][] tagOffsetsArray = homeOffsets.getReefTagOffsets();
        int indexOfTag = tag;
        if (tag < 0 || tag > 22) {
            return 0;
        }

        if (tag >= 17) {
            indexOfTag -= 17;
        }

        if (indexOfTag < 0) {
            return 0;
        }

        return tagOffsetsArray[indexOfTag][2];
    }

    /**
     * Calls on getReefZoneTagID with the given robot swerve position. Automatically has a check on
     * reefTagID and returns robot pose if the robot is not in a target reef zone
     *
     * @return Target Reef Pose
     */
    Pose2d getScoreReefPose() {
        int reefTagID = Field.Reef.getReefZoneTagID(Robot.getSwerve().getRobotPose());
        if (reefTagID < 0) {
            return Robot.getSwerve().getRobotPose();
        }

        SmartDashboard.putNumber("Target Reef ID: ", reefTagID);
        return Field.Reef.getScorePoseFromTagID(reefTagID);
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
}
