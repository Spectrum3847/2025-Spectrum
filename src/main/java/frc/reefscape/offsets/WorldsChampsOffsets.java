package frc.reefscape.offsets;

import frc.reefscape.TagProperties;
import frc.robot.RobotStates;
import lombok.Getter;

public class WorldsChampsOffsets {

    // TODO: Upload new world champs offsets when at worlds

    // TagProperty: offsetLeft, offsetRight, centerLeft, centerRight, taGoal, AngleOffset
    // Blue Tags (17–22)
    private static final TagProperties tag17Offset =
            new TagProperties(11.5, 11.0, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag18Offset =
            new TagProperties(12.0, 12.5, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag19Offset =
            new TagProperties(12.0, 13.0, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag20Offset =
            new TagProperties(12.0, 12.5, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag21Offset =
            new TagProperties(11.5, 10.5, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag22Offset =
            new TagProperties(12.0, 11.0, 0.0, 0.0, 7.9, 180);

    // Red Tags (6–11)
    private static final TagProperties tag6Offset =
            new TagProperties(11.0, 11.0, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag7Offset =
            new TagProperties(12.5, 12.5, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag8Offset =
            new TagProperties(13.0, 13.0, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag9Offset =
            new TagProperties(12.5, 12.5, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag10Offset =
            new TagProperties(10.5, 10.5, 0.0, 0.0, 7.9, 180);

    private static final TagProperties tag11Offset =
            new TagProperties(11.0, 11.0, 0.0, 0.0, 7.9, 180);

    // tag offsets ordered from blue tags to red tags due to centerFaces index values

    @Getter
    private static final TagProperties[] reefTagOffsets = {
        tag17Offset,
        tag18Offset,
        tag19Offset,
        tag20Offset,
        tag21Offset,
        tag22Offset,
        tag6Offset,
        tag7Offset,
        tag8Offset,
        tag9Offset,
        tag10Offset,
        tag11Offset
    };

    // ---------------------------------------------------------------
    // Helper Methods
    // ---------------------------------------------------------------

    public static int getReefTagIDToOffsetIndex(int tagID) {
        if (tagID < 6 || tagID == 16 || tagID > 22) {
            return -1;
        }

        int offsetIndex = tagID;

        if (offsetIndex >= 17) {
            offsetIndex -= 17;
        }

        return offsetIndex;
    }

    public static double getReefTagDistanceOffset(int tagID) {
        int offsetIndex = getReefTagIDToOffsetIndex(tagID);
        if (offsetIndex == -1) {
            return 0.0;
        }

        if (RobotStates.rightScore.getAsBoolean()) {
            return reefTagOffsets[offsetIndex].getOffset()[1];
        }
        return reefTagOffsets[offsetIndex].getOffset()[0];
    }

    public static double getReefTagCenterOffset(int tagID) {
        int offsetIndex = getReefTagIDToOffsetIndex(tagID);
        if (offsetIndex == -1) {
            return 0.0;
        }
        if (RobotStates.rightScore.getAsBoolean()) {
            return reefTagOffsets[offsetIndex].getCenterOffset()[1];
        }
        return reefTagOffsets[offsetIndex].getCenterOffset()[0];
    }

    public static double getReefTagAngleOffset(int tagID) {
        int offsetIndex = getReefTagIDToOffsetIndex(tagID);
        if (offsetIndex == -1) {
            return 0.0;
        }

        return reefTagOffsets[offsetIndex].getAngle();
    }
}
