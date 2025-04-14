package frc.reefscape;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import lombok.Getter;

public class TagProperties {
    @Getter private final double[] offset = new double[2];
    @Getter private final double[] centerOffset = new double[2];
    @Getter private final double taGoal;
    @Getter private final double angle;

    /**
     * @param offsetInchesLeft
     * @param offsetInchesRight
     * @param centerOffsetInchesRight
     * @param centerOffsetInchesLeft
     * @param taGoal
     * @param angle
     */
    public TagProperties(
            double offsetInchesLeft,
            double offsetInchesRight,
            double centerOffsetInchesLeft,
            double centerOffsetInchesRight,
            double taGoal,
            double angleDegrees) {

        offset[0] = meterOffsetWithRobot(offsetInchesLeft);
        offset[1] = meterOffsetWithRobot(offsetInchesRight);
        centerOffset[0] = Units.inchesToMeters(centerOffsetInchesLeft);
        centerOffset[1] = Units.inchesToMeters(centerOffsetInchesRight);
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
