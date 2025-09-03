package frc.reefscape;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import lombok.Getter;

public class TagProperties {
    @Getter private final double[] frontOffset = new double[2];
    @Getter private final double[] rearOffset = new double[2];
    @Getter private final double[] frontCenterOffset = new double[2];
    @Getter private final double[] rearCenterOffset = new double[2];
    @Getter private final double taGoal;
    @Getter private final double angle;

    /**
     * @param frontOffsetInchesLeft
     * @param frontOffsetInchesRight
     * @param rearOffsetInchesLeft
     * @param rearOffsetInchesRight
     * @param frontCenterOffsetInchesLeft
     * @param frontCenterOffsetInchesRight
     * @param rearCenterOffsetInchesLeft
     * @param rearCenterOffsetInchesRight
     * @param taGoal
     * @param angle
     */
    public TagProperties(
            double frontOffsetInchesLeft,
            double frontOffsetInchesRight,
            double rearOffsetInchesLeft,
            double rearOffsetInchesRight,
            double frontCenterOffsetInchesLeft,
            double frontCenterOffsetInchesRight,
            double rearCenterOffsetInchesLeft,
            double rearCenterOffsetInchesRight,
            double taGoal,
            double angleDegrees) {

        frontOffset[0] = meterOffsetWithRobot(frontOffsetInchesLeft);
        frontOffset[1] = meterOffsetWithRobot(frontOffsetInchesRight);
        rearOffset[0] = meterOffsetWithRobot(rearOffsetInchesLeft);
        rearOffset[1] = meterOffsetWithRobot(rearOffsetInchesRight);
        frontCenterOffset[0] = Units.inchesToMeters(frontCenterOffsetInchesLeft);
        frontCenterOffset[1] = Units.inchesToMeters(frontCenterOffsetInchesRight);
        rearCenterOffset[0] = Units.inchesToMeters(rearCenterOffsetInchesLeft);
        rearCenterOffset[1] = Units.inchesToMeters(rearCenterOffsetInchesRight);
        this.taGoal = taGoal;
        this.angle = radianConverter(angleDegrees);
    }

    private static double meterOffsetWithRobot(double offsetInches) {
        double meterConversion = Units.inchesToMeters(offsetInches);

        return addRobotLength(meterConversion);
    }

    private static double addRobotLength(double meterOffset) {
        double halfRobotLength = Robot.getSwerve().getConfig().getRobotLength() / 2;

        return meterOffset + halfRobotLength;
    }

    private static double radianConverter(double offsetDegrees) {
        return Units.degreesToRadians(offsetDegrees);
    }
}
