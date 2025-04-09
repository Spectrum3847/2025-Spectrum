package frc.reefscape;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import lombok.Getter;

public class WorldsChampsOffsets {

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
    // original values: 60, 0, 300, 240, 180, 120
    @Getter private static final double Tag17Angle = radianConverter(180); // 60);
    @Getter private static final double Tag18Angle = radianConverter(180);
    @Getter private static final double Tag19Angle = radianConverter(180); // -60
    @Getter private static final double Tag20Angle = radianConverter(180);
    @Getter private static final double Tag21Angle = radianConverter(180);
    @Getter private static final double Tag22Angle = radianConverter(180);

    // blue tag angles offsets
    // original values: 120, 180, 240, 300, 0, 60
    @Getter private static final double Tag6Angle = radianConverter(180);
    @Getter private static final double Tag7Angle = radianConverter(180);
    @Getter private static final double Tag8Angle = radianConverter(180);
    @Getter private static final double Tag9Angle = radianConverter(180);
    @Getter private static final double Tag10Angle = radianConverter(180);
    @Getter private static final double Tag11Angle = radianConverter(180);

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
}
