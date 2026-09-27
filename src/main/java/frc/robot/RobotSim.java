package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.reefscape.Field;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Branch;
import frc.robot.subsystems.SuperStructure.Piece;
import frc.robot.subsystems.vision.Vision;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.ArrayList;
import java.util.List;

/**
 * Simulation views: the Mechanism2d side view, and the poses AdvantageScope needs to draw the robot
 * model (advantagescope-custom-assets/Robot_GammaRay), the piece in the claw and every coral scored
 * this session.
 *
 * <p>The component poses follow that folder's README: each GLB is exported at the CAD pose, which
 * is the code's home (elevator 0, shoulder 0, elbow 180, twist 90), and each joint rotates about
 * its pivot measured from the CAD.
 */
public class RobotSim {
    public static final Mechanism2d leftView =
            new Mechanism2d(Units.inchesToMeters(75), Units.inchesToMeters(75));

    // Pivots at HOME, from the CAD (see the model folder's README).
    private static final Translation3d SHOULDER_PIVOT = new Translation3d(0, -0.200, 0.641);
    private static final Translation3d ELBOW_PIVOT = new Translation3d(0, -0.200, 0.387);
    private static final Translation3d TWIST_PIVOT = new Translation3d(0, 0, 0.387);
    private static final double ELEVATOR_TRAVEL_METERS = 0.775;

    /** Where a held piece sits, from the twist pivot, at home. */
    private static final Transform3d CLAW_TO_PIECE =
            new Transform3d(0, 0, 0.55, new Rotation3d(0, Math.PI / 2, 0));

    private final SuperStructure superStructure;
    private final List<Pose3d> scoredCoral = new ArrayList<>();
    private int lastScoreCount = 0;

    public RobotSim(SuperStructure superStructure) {
        this.superStructure = superStructure;
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        SmartDashboard.putData("Sim/LeftView", leftView);
    }

    /** Pose that rotates a part about a pivot: T(p) R T(-p). */
    private static Transform3d about(Translation3d pivot, Rotation3d rotation) {
        return new Transform3d(pivot, rotation)
                .plus(new Transform3d(pivot.unaryMinus(), Rotation3d.kZero));
    }

    public void periodic() {
        var elevator = superStructure.getElevator();
        var shoulder = superStructure.getShoulder();
        var elbow = superStructure.getElbow();
        var twist = superStructure.getTwist();

        // Shoulder: + swings the claw toward the front, which is -pitch about +Y.
        double a = -Math.toRadians(shoulder.getDegrees());
        // Elbow: chain-driven from the carriage, so its angle is absolute, not relative to the
        // upper arm; the forearm's rotation relative to the upper arm is b - a.
        double b = Math.toRadians(elbow.getDegrees() - 180);
        double t = Math.toRadians(twist.getPositionDegrees() - 90);
        double h = elevator.getPositionRotations() / 21.1 * ELEVATOR_TRAVEL_METERS;

        Pose3d carriage = new Pose3d(0, 0, h, Rotation3d.kZero);
        Pose3d upperArm = carriage.transformBy(about(SHOULDER_PIVOT, new Rotation3d(0, a, 0)));
        Pose3d forearm = upperArm.transformBy(about(ELBOW_PIVOT, new Rotation3d(0, b - a, 0)));
        Pose3d claw = forearm.transformBy(about(TWIST_PIVOT, new Rotation3d(0, 0, t)));
        Telemetry.log("Sim/Components", new Pose3d[] {carriage, upperArm, forearm, claw});

        Pose3d robot = new Pose3d(superStructure.getSwerve().getRobotPose());
        Piece held = superStructure.getHeldPiece();
        Pose3d heldPose =
                robot.transformBy(
                        new Transform3d(Pose3d.kZero, claw)
                                .plus(new Transform3d(TWIST_PIVOT, Rotation3d.kZero))
                                .plus(CLAW_TO_PIECE));
        Telemetry.log(
                "Sim/HeldCoral", held == Piece.CORAL ? new Pose3d[] {heldPose} : new Pose3d[0]);
        Telemetry.log(
                "Sim/HeldAlgae", held == Piece.ALGAE ? new Pose3d[] {heldPose} : new Pose3d[0]);

        if (superStructure.getCoralScoreCount() != lastScoreCount) {
            lastScoreCount = superStructure.getCoralScoreCount();
            scoredCoral.add(
                    Field.branchPose(
                            Vision.nearestReefTag(),
                            superStructure.getSelectedBranch() == Branch.RIGHT,
                            superStructure.getSelectedLevel()));
        }
        Telemetry.log("Sim/ScoredCoral", scoredCoral.toArray(new Pose3d[0]));
    }
}
