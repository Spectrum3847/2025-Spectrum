// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.reefscape;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Field dimensions and reference points for REEFSCAPE. Metres, blue-alliance origin.
 *
 * <p>Tag positions come from {@link #TAG_LAYOUT}. The robot and the Limelights must use the same
 * layout as the event field (welded or AndyMark); change it here and in each camera's web UI.
 */
public final class Field {
    private Field() {}

    public static final AprilTagFieldLayout TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final double fieldLength = TAG_LAYOUT.getFieldLength();
    public static final double fieldWidth = TAG_LAYOUT.getFieldWidth();

    /** Reef tag IDs, blue then red, each in the same order around the reef. */
    public static final int[] BLUE_REEF_TAGS = {17, 18, 19, 20, 21, 22};

    public static final int[] RED_REEF_TAGS = {8, 7, 6, 11, 10, 9};

    /**
     * Which algae height each face holds, indexed like {@link #BLUE_REEF_TAGS}: true where it sits
     * on the high (L3) shelf. Alternates around the reef; 18, 20 and 22 are high.
     */
    private static final boolean[] HIGH_ALGAE = {false, true, false, true, false, true};

    public static final double coralStationLeftDegrees = 144.011 - 90;

    public static final Pose2d leftCoralStation =
            new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(291.176),
                    Rotation2d.fromDegrees(90 - 144.011));

    public static final Pose2d rightCoralStation =
            new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(25.824),
                    Rotation2d.fromDegrees(144.011 - 90));

    /** Barge net centreline, x (blue side of the field centre line). */
    public static final double bargeX = fieldLength / 2.0;

    public enum ReefHeight {
        L1(Units.inchesToMeters(18), 0),
        L2(Units.inchesToMeters(31.875), -35),
        L3(Units.inchesToMeters(47.625), -35),
        L4(Units.inchesToMeters(72), -90);

        ReefHeight(double height, double pitchDegrees) {
            this.height = height;
            this.pitchDegrees = pitchDegrees;
        }

        public final double height;
        public final double pitchDegrees;
    }

    /** Branch spacing either side of a face centre. */
    private static final double BRANCH_HALF_SPACING = Units.inchesToMeters(6.469);

    /** How far behind the face (tag plane) the branch pipes stand. */
    private static final double BRANCH_INSET = Units.inchesToMeters(2.0);

    public static Pose2d tagPose(int id) {
        return TAG_LAYOUT.getTagPose(id).map(Pose3d::toPose2d).orElse(Pose2d.kZero);
    }

    public static boolean isReefTag(int id) {
        return (id >= 6 && id <= 11) || (id >= 17 && id <= 22);
    }

    /** True for the reef face whose algae sits on the high shelf. */
    public static boolean isHighAlgae(int reefTagId) {
        for (int i = 0; i < 6; i++) {
            if (BLUE_REEF_TAGS[i] == reefTagId || RED_REEF_TAGS[i] == reefTagId) {
                return HIGH_ALGAE[i];
            }
        }
        return false;
    }

    /** The reef tag of the given alliance nearest to a field position. */
    public static int nearestReefTag(Translation2d position, boolean red) {
        int best = -1;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int id : red ? RED_REEF_TAGS : BLUE_REEF_TAGS) {
            double d = tagPose(id).getTranslation().getDistance(position);
            if (d < bestDistance) {
                bestDistance = d;
                best = id;
            }
        }
        return best;
    }

    /**
     * Where a coral sits on a branch, for the simulation view. Left and right are as seen facing
     * the tag.
     */
    public static Pose3d branchPose(int reefTagId, boolean right, ReefHeight height) {
        Pose2d face = tagPose(reefTagId);
        // The tag faces out of the reef, so +y in its frame is the robot's right when facing it.
        Pose2d branch =
                face.transformBy(
                        new Transform2d(
                                -BRANCH_INSET,
                                right ? BRANCH_HALF_SPACING : -BRANCH_HALF_SPACING,
                                Rotation2d.kZero));
        return new Pose3d(
                branch.getX(),
                branch.getY(),
                height.height,
                new Rotation3d(
                        0,
                        Units.degreesToRadians(-height.pitchDegrees),
                        face.getRotation().getRadians() + Math.PI));
    }
}
