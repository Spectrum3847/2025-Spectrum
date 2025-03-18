// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.reefscape;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class Field {
    @Getter public static final double fieldLength = Units.inchesToMeters(690.876);
    @Getter private static final double halfLength = fieldLength / 2.0;
    @Getter public static final double fieldWidth = Units.inchesToMeters(317);
    @Getter private static final double halfWidth = fieldWidth / 2.0;

    @Getter
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        @Getter
        public static final Pose2d[] centerFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order

        public static final List<Map<ReefHeight, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in
        // clockwise

        static {
            // Initialize faces
            centerFaces[0] = // reef id 18
                    new Pose2d(
                            Units.inchesToMeters(144.003),
                            Units.inchesToMeters(158.500),
                            Rotation2d.fromDegrees(180));
            centerFaces[1] = // reef id 19
                    new Pose2d(
                            Units.inchesToMeters(160.373),
                            Units.inchesToMeters(186.857),
                            Rotation2d.fromDegrees(120));
            centerFaces[2] = // reef id 20
                    new Pose2d(
                            Units.inchesToMeters(193.116),
                            Units.inchesToMeters(186.858),
                            Rotation2d.fromDegrees(60));
            centerFaces[3] = // reef id 21
                    new Pose2d(
                            Units.inchesToMeters(209.489),
                            Units.inchesToMeters(158.502),
                            Rotation2d.fromDegrees(0));
            centerFaces[4] = // reef id 22
                    new Pose2d(
                            Units.inchesToMeters(193.118),
                            Units.inchesToMeters(130.145),
                            Rotation2d.fromDegrees(-60));
            centerFaces[5] = // reef id 17
                    new Pose2d(
                            Units.inchesToMeters(160.375),
                            Units.inchesToMeters(130.144),
                            Rotation2d.fromDegrees(-120));

            // Initialize branch positions
            //     for (int face = 0; face < 6; face++) {
            //         Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
            //         Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
            //         for (var level : ReefHeight.values()) {
            //             Pose2d poseDirection =
            //                     new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
            //             double adjustX = Units.inchesToMeters(30.738);
            //             double adjustY = Units.inchesToMeters(6.469);

            //             fillRight.put(
            //                     level,
            //                     new Pose3d(
            //                             new Translation3d(
            //                                     poseDirection
            //                                             .transformBy(
            //                                                     new Transform2d(
            //                                                             adjustX,
            //                                                             adjustY,
            //                                                             new Rotation2d()))
            //                                             .getX(),
            //                                     poseDirection
            //                                             .transformBy(
            //                                                     new Transform2d(
            //                                                             adjustX,
            //                                                             adjustY,
            //                                                             new Rotation2d()))
            //                                             .getY(),
            //                                     level.height),
            //                             new Rotation3d(
            //                                     0,
            //                                     Units.degreesToRadians(level.pitch),
            //                                     poseDirection.getRotation().getRadians())));
            //             fillLeft.put(
            //                     level,
            //                     new Pose3d(
            //                             new Translation3d(
            //                                     poseDirection
            //                                             .transformBy(
            //                                                     new Transform2d(
            //                                                             adjustX,
            //                                                             -adjustY,
            //                                                             new Rotation2d()))
            //                                             .getX(),
            //                                     poseDirection
            //                                             .transformBy(
            //                                                     new Transform2d(
            //                                                             adjustX,
            //                                                             -adjustY,
            //                                                             new Rotation2d()))
            //                                             .getY(),
            //                                     level.height),
            //                             new Rotation3d(
            //                                     0,
            //                                     Units.degreesToRadians(level.pitch),
            //                                     poseDirection.getRotation().getRadians())));
            //         }
            //         branchPositions.add((face * 2) + 1, fillRight);
            //         branchPositions.add((face * 2) + 2, fillLeft);
            //     }
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final Pose2d leftIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
        public static final Pose2d middleIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
        public static final Pose2d rightIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public enum ReefHeight {
        L4(Units.inchesToMeters(72), -90),
        L3(Units.inchesToMeters(47.625), -35),
        L2(Units.inchesToMeters(31.875), -35),
        L1(Units.inchesToMeters(18), 0);

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }

        public final double height;
        public final double pitch;
    }

    @Getter private static final double aprilTagWidth = Units.inchesToMeters(6.50);

    /** Returns {@code true} if the robot is on the blue alliance. */
    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    /** Returns {@code true} if the robot is on the red alliance. */
    public static boolean isRed() {
        return !isBlue();
    }

    public static final Trigger red = new Trigger(() -> isRed());
    public static final Trigger blue = new Trigger(() -> isBlue());

    // Flip the angle if we are blue, as we are setting things for a red driver station angle
    // This flips the left and right side for aiming purposes
    public static double flipAimAngleIfBlue(double redAngleDegs) {
        if (Field.isBlue()) {
            return 180 - redAngleDegs;
        }
        return redAngleDegs;
    }

    // This flips the true angle of the robot if we are blue
    public static double flipTrueAngleIfBlue(double redAngleDegs) {
        if (Field.isBlue()) {
            return (180 + redAngleDegs) % 360;
        }
        return redAngleDegs;
    }

    public static double flipTrueAngleIfRed(double blueAngleDegs) {
        if (Field.isRed()) {
            return (180 + blueAngleDegs) % 360;
        }
        return blueAngleDegs;
    }

    public static Rotation2d flipAngleIfRed(Rotation2d blue) {
        if (Field.isRed()) {
            return new Rotation2d(-blue.getCos(), blue.getSin());
        } else {
            return blue;
        }
    }

    public static Pose2d flipXifRed(Pose2d blue) {
        return new Pose2d(
                flipXifRed(blue.getX()), blue.getTranslation().getY(), blue.getRotation());
    }

    public static Translation2d flipXifRed(Translation2d blue) {
        return new Translation2d(flipXifRed(blue.getX()), blue.getY());
    }

    public static Translation3d flipXifRed(Translation3d blue) {
        return new Translation3d(flipXifRed(blue.getX()), blue.getY(), blue.getZ());
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

    public static boolean poseOutOfField(Pose2d pose2D) {
        double x = pose2D.getX();
        double y = pose2D.getY();
        return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
    }

    public static boolean poseOutOfField(Pose3d pose3D) {
        return poseOutOfField(pose3D.toPose2d());
    }
}
