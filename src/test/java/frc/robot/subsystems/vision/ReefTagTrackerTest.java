package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.ReefTagTracker.Sighting;
import org.junit.jupiter.api.Test;

class ReefTagTrackerTest {
    /** Front camera: 0.215 m forward, 0.188 m up, pitched 28 degrees up. */
    private static final Transform3d FRONT =
            new Transform3d(
                    new Translation3d(0.215, 0, 0.188), new Rotation3d(0, -Math.toRadians(28), 0));

    private static Sighting sightingOf(Translation3d tagInRobot, Transform3d robotToCamera) {
        Translation3d inCamera =
                tagInRobot
                        .minus(robotToCamera.getTranslation())
                        .rotateBy(robotToCamera.getRotation().unaryMinus());
        double tx = -Math.toDegrees(Math.atan2(inCamera.getY(), inCamera.getX()));
        double ty = Math.toDegrees(Math.atan2(inCamera.getZ(), inCamera.getX()));
        return new Sighting(18, tx, ty, inCamera.getNorm(), 0);
    }

    @Test
    void recoversTheTagPositionFromAnglesAndRange() {
        Translation3d tag = new Translation3d(0.9, -0.25, 0.308);
        Translation2d measured = ReefTagTracker.robotToTag(FRONT, sightingOf(tag, FRONT));
        assertEquals(0.9, measured.getX(), 1e-6);
        assertEquals(-0.25, measured.getY(), 1e-6);
    }

    @Test
    void worksForTheRearCamera() {
        Transform3d back =
                new Transform3d(
                        new Translation3d(-0.215, 0, 0.188),
                        new Rotation3d(0, -Math.toRadians(28), Math.PI));
        Translation3d tag = new Translation3d(-1.1, 0.3, 0.308);
        Translation2d measured = ReefTagTracker.robotToTag(back, sightingOf(tag, back));
        assertEquals(-1.1, measured.getX(), 1e-6);
        assertEquals(0.3, measured.getY(), 1e-6);
    }

    @Test
    void trustsEdgeTagsLess() {
        assertEquals(1.0, ReefTagTracker.edgeScale(0, 0), 1e-9);
        assertEquals(4.0, ReefTagTracker.edgeScale(41, 0), 1e-9);
    }
}
