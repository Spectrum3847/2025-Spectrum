package frc.reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Alliance and field-bounds helpers. Blue origin throughout. */
public final class FieldHelpers {
    private FieldHelpers() {}

    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public static double flipAngleIfRed(double blueDegrees) {
        return isRed() ? (blueDegrees + 180) % 360 : blueDegrees;
    }

    public static boolean poseOutOfField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x <= 0 || x >= Field.fieldLength || y <= 0 || y >= Field.fieldWidth;
    }
}
