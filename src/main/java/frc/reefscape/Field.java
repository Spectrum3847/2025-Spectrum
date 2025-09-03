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
import frc.robot.Robot;
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
    private static final Pose2d centerField = new Pose2d(halfLength, halfWidth, new Rotation2d());

    @Getter
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final double netRobotPovDegrees = 0;

        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);

        /**
         * Returns the y value of a cage when closest to a given cage.
         *
         * @return double
         */
        public double getCageToClimb() {
            Pose2d robotPose = Robot.getSwerve().getRobotPose();
            int index = 0;
            // Default is blue cages
            double[] cageDiffs = {
                Math.abs(robotPose.getY() - farCage.getY()),
                Math.abs(robotPose.getY() - middleCage.getY()),
                Math.abs(robotPose.getY() - closeCage.getY())
            };

            if (isRed()) {
                double redRobotYPose = FieldHelpers.flipYifRed(robotPose.getY());

                cageDiffs[0] = Math.abs(redRobotYPose - FieldHelpers.flipYifRed(farCage.getY()));
                cageDiffs[1] =
                        Math.abs(
                                FieldHelpers.flipYifRed(robotPose.getY())
                                        - FieldHelpers.flipYifRed(middleCage.getY()));
                cageDiffs[2] =
                        Math.abs(
                                FieldHelpers.flipYifRed(robotPose.getY())
                                        - FieldHelpers.flipYifRed(closeCage.getY()));
            }
            index = FieldHelpers.indexOfSmallest(cageDiffs);

            // R1 or B1
            if (index == 0) {
                // R1 or B1
                return cageDiffs[index];
            } else if (index == 1) {
                // R2 or B2
                return cageDiffs[index];
            } else if (index == 2) {
                // R3 or B3
                return cageDiffs[index];
            } else {
                // Robot Y if no target zone found
                return Robot.getSwerve().getRobotPose().getY();
            }
        }
    }

    public static class CoralStation {
        public static final double leftFaceRobotPovDegrees = 144.011;
        public static final double rightFaceRobotPovDegrees = -144.011;

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
        @Getter static final double radius = Units.inchesToMeters(37.02);
        @Getter static final double apothem = Units.inchesToMeters(32.06);

        @Getter
        private static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Trigger poseReversal =
                new Trigger(() -> FieldHelpers.reverseRotationBlue());

        @SuppressWarnings("all")
        @Getter
        public static final Pose2d[] centerFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order

        @SuppressWarnings("all")
        public static final List<Map<ReefHeight, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in
        // clockwise

        static Zones zones = new Zones();

        static {
            // Initialize faces
            centerFaces[0] = // reef id 17
                    new Pose2d(
                            Units.inchesToMeters(160.375),
                            Units.inchesToMeters(130.144),
                            Rotation2d.fromDegrees(-120));
            centerFaces[1] = // reef id 18
                    new Pose2d(
                            Units.inchesToMeters(144.003),
                            Units.inchesToMeters(158.500),
                            Rotation2d.fromDegrees(180));
            centerFaces[2] = // reef id 19
                    new Pose2d(
                            Units.inchesToMeters(160.373),
                            Units.inchesToMeters(186.857),
                            Rotation2d.fromDegrees(120));
            centerFaces[3] = // reef id 20
                    new Pose2d(
                            Units.inchesToMeters(193.116),
                            Units.inchesToMeters(186.858),
                            Rotation2d.fromDegrees(60));
            centerFaces[4] = // reef id 21
                    new Pose2d(
                            Units.inchesToMeters(209.489),
                            Units.inchesToMeters(158.502),
                            Rotation2d.fromDegrees(0));
            centerFaces[5] = // reef id 22
                    new Pose2d(
                            Units.inchesToMeters(193.118),
                            Units.inchesToMeters(130.145),
                            Rotation2d.fromDegrees(-60));
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

    public static final Trigger red = new Trigger(Field::isRed);
    public static final Trigger blue = new Trigger(Field::isBlue);
}
