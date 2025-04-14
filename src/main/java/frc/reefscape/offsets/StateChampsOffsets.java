package frc.reefscape.offsets;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import lombok.Getter;

public class StateChampsOffsets {

    // blue tag offsets
    @Getter private static final double Tag17Offset = meterConverter(11.0);
    @Getter private static final double Tag18Offset = meterConverter(12.5);
    @Getter private static final double Tag19Offset = meterConverter(13.0);
    @Getter private static final double Tag20Offset = meterConverter(12.5);
    @Getter private static final double Tag21Offset = meterConverter(10.5);
    @Getter private static final double Tag22Offset = meterConverter(11.0);
    // red tag offsets
    @Getter private static final double Tag6Offset = meterConverter(11.0);
    @Getter private static final double Tag7Offset = meterConverter(12.5);
    @Getter private static final double Tag8Offset = meterConverter(13.0);
    @Getter private static final double Tag9Offset = meterConverter(12.5);
    @Getter private static final double Tag10Offset = meterConverter(10.5);
    @Getter private static final double Tag11Offset = meterConverter(11.0);

    // red tag angles offsets
    // each angle is the opposite of the reef face angle when sent
    @Getter private static final double Tag17Angle = radianConverter(180); // 60);
    @Getter private static final double Tag18Angle = radianConverter(180);
    @Getter private static final double Tag19Angle = radianConverter(180); // -60
    @Getter private static final double Tag20Angle = radianConverter(180);
    @Getter private static final double Tag21Angle = radianConverter(180);
    @Getter private static final double Tag22Angle = radianConverter(180);

    // blue tag angles offsets
    // each angle is the opposite of the reef face angle when sent
    @Getter private static final double Tag6Angle = radianConverter(180);
    @Getter private static final double Tag7Angle = radianConverter(180);
    @Getter private static final double Tag8Angle = radianConverter(180);
    @Getter private static final double Tag9Angle = radianConverter(180);
    @Getter private static final double Tag10Angle = radianConverter(180);
    @Getter private static final double Tag11Angle = radianConverter(180);

    // blue tag area goals
    @Getter private static final double Tag17TAGoal = 8.9;
    @Getter private static final double Tag18TAGoal = 8.9;
    @Getter private static final double Tag19TAGoal = 8.9; // 7.8
    @Getter private static final double Tag20TAGoal = 8.9; // 7.8
    @Getter private static final double Tag21TAGoal = 8.9; // 7.8
    @Getter private static final double Tag22TAGoal = 8.9; // 7.8

    // red tag area goals
    @Getter private static final double Tag6TAGoal = 8.9; // 7.8;
    @Getter private static final double Tag7TAGoal = 8.9; // 7.8;
    @Getter private static final double Tag8TAGoal = 8.9; // 7.8;
    @Getter private static final double Tag9TAGoal = 8.9; // 7.6;
    @Getter private static final double Tag10TAGoal = 8.9; // 7.7;
    @Getter private static final double Tag11TAGoal = 8.9; // 7.8;

    /**
     * Converts inches to meters and adds half the robot length to the offset
     *
     * @param offsetInches
     * @return
     */
    private static double meterConverter(double offsetInches) {
        double meterConversion = Units.inchesToMeters(offsetInches);
        double halfRobotLength = Robot.getSwerve().getConfig().getRobotLength() / 2;

        return meterConversion + halfRobotLength;
    }

    /** @param degrees */
    private static double radianConverter(double offsetDegrees) {
        double radianConversion = Units.degreesToRadians(offsetDegrees);

        return radianConversion;
    }

    // tag offsets ordered from blue tags to red tags due to centerFaces index values
    @Getter
    private static final double[][] reefTagOffsets = {
        {17, Tag17Offset, Tag17Angle},
        {18, Tag18Offset, Tag18Angle},
        {19, Tag19Offset, Tag19Angle},
        {20, Tag20Offset, Tag20Angle},
        {21, Tag21Offset, Tag21Angle},
        {22, Tag22Offset, Tag22Angle},
        {6, Tag6Offset, Tag6Angle},
        {7, Tag7Offset, Tag7Angle},
        {8, Tag8Offset, Tag8Angle},
        {9, Tag9Offset, Tag9Angle},
        {10, Tag10Offset, Tag10Angle},
        {11, Tag11Offset, Tag11Angle}
    };

    @Getter
    private static double[][] tagAreaOffsets = {
        {17, Tag17TAGoal},
        {18, Tag18TAGoal},
        {19, Tag19TAGoal},
        {20, Tag20TAGoal},
        {21, Tag21TAGoal},
        {22, Tag22TAGoal},
        {6, Tag6TAGoal},
        {7, Tag7TAGoal},
        {8, Tag8TAGoal},
        {9, Tag9TAGoal},
        {10, Tag10TAGoal},
        {11, Tag11TAGoal}
    };
}
