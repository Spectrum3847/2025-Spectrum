package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.Field;
import frc.reefscape.FieldHelpers;
import frc.robot.Robot;
import frc.robot.auton.Auton;
import frc.robot.subsystems.vision.ReefTagTracker.Camera;
import frc.robot.subsystems.vision.ReefTagTracker.Sighting;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import frc.spectrumLib.vision.VisionLogger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.Getter;

/**
 * Pose estimation from the front and back Limelight 4s, ported from the 2026 robot.
 *
 * <p>Roles are kept strictly apart:
 *
 * <ul>
 *   <li><b>Disabled</b>: the better camera's MegaTag1 seeds translation <i>and heading</i>. The
 *       seed only counts as confirmed after a second of multi-tag frames whose heading holds within
 *       3 degrees.
 *   <li><b>Enabled</b>: both cameras fuse translation only, MegaTag2 once the seed is confirmed.
 *       Heading belongs to the gyro. The 2025 code fused MegaTag1 heading every teleop loop (15
 *       degree two-tag noise tail) and once gave a pose a zero standard deviation.
 *   <li><b>Safety nets</b>: seed while enabled if nothing seeded before; confirm or refute the auto
 *       start pose from multi-tag frames; reset a gross (20 degree) heading error once the robot is
 *       still.
 * </ul>
 *
 * <p>Reef alignment does not use the fused pose for its last half metre; see {@link
 * ReefTagTracker}, which this class feeds.
 *
 * <p>Deliberately not a registered Subsystem: {@link Robot#robotPeriodic()} runs it before the
 * scheduler so this loop's correction reaches every mechanism in the same loop.
 */
public class Vision {

    public static class VisionConfig {
        /**
         * Robot-relative mounts. LL convention: forward, right, up (m); roll, pitch (up positive),
         * yaw (deg). These are the source of truth: they are pushed to each camera every couple of
         * seconds, overwriting its web UI. Measure them on the robot (issue 8d); a pitch that is
         * off by a degree shows up as range error at every tag.
         */
        @Getter
        final LimelightConfig frontConfig =
                new LimelightConfig("limelight-front")
                        .withTranslation(0.215, 0, 0.188)
                        .withRotation(0, 28, 0);

        @Getter
        final LimelightConfig backConfig =
                new LimelightConfig("limelight-back")
                        .withTranslation(-0.215, 0, 0.188)
                        .withRotation(0, 28, 180);

        /** Ignore a measurement dimension (heading while enabled). */
        @Getter final double largeVariance = 999999.0;

        @Getter final double maxEstimateAgeSeconds = 1.0;
        @Getter final double maxZErrorMeters = 0.75;
        @Getter final double maxTiltDegrees = 5;
        @Getter final double maxYawRateRadPerSec = 1.6;
        @Getter final double yawRateLookbackSeconds = 0.3;
        @Getter final double maxAmbiguity = 0.9;
        /** Tags further than this are not fused (AOS uses about 5 m). */
        @Getter final double maxTagDistanceMeters = 5.0;

        @Getter final double minTargetSize = 0.025;

        /**
         * 6328's std-dev model: xy = coefficient * d^2 / n^2, where d is the mean tag distance and
         * n the tag count. Retune on this robot from logs.
         */
        @Getter final double xyStdCoefficient = 0.01;

        @Getter final double minXyStd = 0.05;
        /** MegaTag1 translation moves with its noisy heading, so trust it less than MegaTag2. */
        @Getter final double mt1Factor = 2.0;

        @Getter final int seedConfirmLoops = 50;
        @Getter final double seedConfirmSpreadDeg = 3.0;
        @Getter final double grossHeadingErrorDeg = 20.0;
        @Getter final double grossHeadingHoldSeconds = 1.0;
        @Getter final double stillLinearSpeed = 0.2;
        @Getter final double stillOmega = 0.1;
        @Getter final double placementAgreeDeg = 8.0;
        @Getter final int placementDecideFrames = 10;
    }

    private final VisionConfig config;
    @Getter private final Limelight frontLL;
    @Getter private final Limelight backLL;
    private final Limelight[] limelights;
    private final VisionLogger[] loggers;
    private final Alert[] disconnectedAlerts;
    private final double[] lastConnectedSeconds;

    /** Tag-relative tracking for reef alignment, fed by both cameras. */
    @Getter private final ReefTagTracker reefTagTracker;

    private final Camera[] trackerCameras;

    @Getter private boolean poseHeadingSeeded = false;
    @Getter private boolean poseSeedConfirmed = false;
    private boolean headingFromPlacement = false;
    private int seedConfirmStreak = 0;
    private double seedRefDeg = 0;
    private double seedLow = 0;
    private double seedHigh = 0;
    private double lastAcceptedFpga = Double.NaN;
    private double grossStartFpga = Double.NaN;
    private int placementAgree = 0;
    private int placementDisagree = 0;
    private double placementLastFrame = Double.NaN;

    private static final int YAW_HISTORY = 64;
    private final double[] yawRates = new double[YAW_HISTORY];
    private final double[] yawTimes = new double[YAW_HISTORY];
    private int yawIndex = 0;

    /** Rejections by reason since boot, so a log shows why frames were dropped (issue 8b). */
    private final Map<String, Integer> rejections = new HashMap<>();

    private double lastSettingsSend = Double.NEGATIVE_INFINITY;

    private final Alert notSeededAlert =
            new Alert(
                    "Vision has NOT seeded the pose: get a camera on two or more tags before the"
                            + " match.",
                    AlertType.kWarning);

    public Vision(VisionConfig config) {
        this.config = config;
        frontLL = new Limelight(config.frontConfig);
        backLL = new Limelight(config.backConfig);
        limelights = new Limelight[] {frontLL, backLL};
        loggers =
                new VisionLogger[] {
                    new VisionLogger("Front", frontLL), new VisionLogger("Back", backLL)
                };
        disconnectedAlerts = new Alert[limelights.length];
        lastConnectedSeconds = new double[limelights.length];
        for (int i = 0; i < limelights.length; i++) {
            disconnectedAlerts[i] =
                    new Alert(limelights[i].getName() + " is not connected", AlertType.kError);
        }
        trackerCameras =
                new Camera[] {
                    new Camera("Front", robotToCamera(config.frontConfig)),
                    new Camera("Back", robotToCamera(config.backConfig))
                };
        reefTagTracker = new ReefTagTracker(trackerCameras);
        sendCameraSettings();
        Telemetry.print("Vision Initialized");
    }

    /** LL mount to a WPILib transform. LL pitch is up-positive; WPILib's is down-positive. */
    static Transform3d robotToCamera(LimelightConfig c) {
        return new Transform3d(
                new Translation3d(c.getForward(), -c.getRight(), c.getUp()),
                new Rotation3d(
                        Math.toRadians(c.getRoll()),
                        -Math.toRadians(c.getPitch()),
                        Math.toRadians(c.getYaw())));
    }

    public Transform3d getRobotToCamera(int index) {
        return trackerCameras[index].robotToCamera();
    }

    // ── Loop ───────────────────────────────────────────────────────────────────

    public void periodic() {
        for (Limelight ll : limelights) {
            ll.invalidate();
        }
        double heading = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        for (Limelight ll : limelights) {
            ll.setRobotOrientation(heading);
        }
        if (Timer.getFPGATimestamp() - lastSettingsSend >= 2.0) {
            sendCameraSettings();
        }
        NetworkTableInstance.getDefault().flush();

        recordYawRate();
        feedReefTagTracker();
        if (Util.disabled.getAsBoolean()) {
            disabledUpdates();
        } else if (Util.teleop.getAsBoolean() || Auton.autonPoseUpdate.getAsBoolean()) {
            enabledUpdates();
        }
        logTelemetry();
    }

    /**
     * Republishes what the robot owns on every camera: IMU mode 1 (internal IMU seeded by our
     * heading) and the mounts, so a camera that reboots mid-match picks both back up.
     */
    private void sendCameraSettings() {
        for (Limelight ll : limelights) {
            ll.setIMUmode(1);
            ll.pushConfiguredCameraPose();
        }
        lastSettingsSend = Timer.getFPGATimestamp();
    }

    private void feedReefTagTracker() {
        for (int i = 0; i < limelights.length; i++) {
            reefTagTracker.addSightings(
                    i,
                    RobotBase.isSimulation()
                            ? SimTagCamera.sightings(trackerCameras[i].robotToCamera())
                            : sightings(limelights[i]));
        }
    }

    private static List<Sighting> sightings(Limelight ll) {
        RawFiducial[] tags = ll.getRawFiducial();
        List<Sighting> out = new ArrayList<>(tags.length);
        double timestamp = ll.getMegaTag1PoseTimestamp();
        for (RawFiducial t : tags) {
            out.add(new Sighting(t.id, t.txnc, t.tync, t.distToCamera, timestamp));
        }
        return out;
    }

    // ── Disabled: seed and confirm ─────────────────────────────────────────────

    private void disabledUpdates() {
        Limelight best = getBestLimelight();
        Estimate seed = mt1Estimate(best, true);
        if (seed != null) {
            integrate(best, seed);
            poseHeadingSeeded = true;
            trackSeedConfirmation(best);
        } else {
            seedConfirmStreak = 0;
        }
        notSeededAlert.set(!poseHeadingSeeded);
    }

    private void trackSeedConfirmation(Limelight best) {
        if (poseSeedConfirmed || !best.multipleTagsInView()) {
            seedConfirmStreak = 0;
            return;
        }
        double deg = best.getMegaTag1_Pose3d().toPose2d().getRotation().getDegrees();
        if (seedConfirmStreak == 0) {
            seedRefDeg = deg;
            seedLow = 0;
            seedHigh = 0;
        }
        double d = MathUtil.inputModulus(deg - seedRefDeg, -180, 180);
        seedLow = Math.min(seedLow, d);
        seedHigh = Math.max(seedHigh, d);
        if (seedHigh - seedLow > config.seedConfirmSpreadDeg) {
            seedConfirmStreak = 0;
            return;
        }
        if (++seedConfirmStreak >= config.seedConfirmLoops) {
            poseSeedConfirmed = true;
            Telemetry.print(
                    "Vision: pose seed confirmed from " + best.getName(), PrintPriority.HIGH);
        }
    }

    /** Called by Robot when it places the pose on the selected auto's start. */
    public void notePlacedAtAutoStart() {
        headingFromPlacement = true;
    }

    // ── Enabled: fuse translation, run the heading safety nets ────────────────

    private void enabledUpdates() {
        Limelight best = getBestLimelight();
        boolean still = isStill();

        if (!poseHeadingSeeded && still && best.multipleTagsInView()) {
            Estimate seed = mt1Estimate(best, true);
            if (seed != null) {
                integrate(best, seed);
                poseHeadingSeeded = true;
                Telemetry.print("Vision: pose seeded while enabled", PrintPriority.HIGH);
            }
        }

        // Fuse in capture order across cameras (issue 8b).
        List<Estimate> estimates = new ArrayList<>(2);
        List<Limelight> sources = new ArrayList<>(2);
        for (Limelight ll : limelights) {
            Estimate e = poseSeedConfirmed ? mt2Estimate(ll) : mt1Estimate(ll, false);
            if (e != null) {
                estimates.add(e);
                sources.add(ll);
            }
        }
        if (estimates.size() == 2 && estimates.get(0).timestamp > estimates.get(1).timestamp) {
            integrate(sources.get(1), estimates.get(1));
            integrate(sources.get(0), estimates.get(0));
        } else {
            for (int i = 0; i < estimates.size(); i++) {
                integrate(sources.get(i), estimates.get(i));
            }
        }

        double headingErrorDeg = multiTagHeadingErrorDeg(best);
        checkGrossHeading(best, headingErrorDeg, still);
        checkPlacement(best, headingErrorDeg);
        Telemetry.log("Vision/ChassisSource", poseSeedConfirmed ? "MT2" : "MT1");
    }

    /** Best camera's fresh multi-tag MT1 heading minus the pose heading, or NaN. */
    private double multiTagHeadingErrorDeg(Limelight best) {
        if (!best.multipleTagsInView() || isStale(best, best.getMegaTag1PoseTimestamp())) {
            return Double.NaN;
        }
        Pose2d mt1 = best.getMegaTag1_Pose3d().toPose2d();
        if (FieldHelpers.poseOutOfField(mt1)) {
            return Double.NaN;
        }
        return mt1.getRotation().minus(Robot.getSwerve().getRobotPose().getRotation()).getDegrees();
    }

    private void checkGrossHeading(Limelight best, double errorDeg, boolean still) {
        boolean gross =
                !Double.isNaN(errorDeg)
                        && still
                        && Math.abs(errorDeg) >= config.grossHeadingErrorDeg;
        double now = Timer.getFPGATimestamp();
        if (!gross) {
            grossStartFpga = Double.NaN;
        } else if (Double.isNaN(grossStartFpga)) {
            grossStartFpga = now;
        } else if (now - grossStartFpga >= config.grossHeadingHoldSeconds) {
            Robot.getSwerve()
                    .addVisionMeasurement(
                            best.getMegaTag1_Pose3d().toPose2d(),
                            Utils.fpgaToCurrentTime(best.getMegaTag1PoseTimestamp()),
                            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01)));
            grossStartFpga = Double.NaN;
            poseSeedConfirmed = true;
            Telemetry.print(
                    String.format("Vision: gross heading error of %.1f deg corrected", errorDeg),
                    PrintPriority.HIGH);
        }
        Telemetry.log("Vision/HeadingCorrection/ErrorDeg", errorDeg, "deg");
        Telemetry.log("Vision/HeadingCorrection/Armed", gross);
    }

    /**
     * With the pose placed on the auto start and never camera-seeded, ten agreeing multi-tag frames
     * confirm the placed heading and ten steady disagreeing ones re-seed from the camera. Agreement
     * moves nothing, so it needs no stillness.
     */
    private void checkPlacement(Limelight best, double errorDeg) {
        if (poseHeadingSeeded || !headingFromPlacement || Double.isNaN(errorDeg)) {
            return;
        }
        double frame = best.getMegaTag1PoseTimestamp();
        if (frame == placementLastFrame) {
            return;
        }
        placementLastFrame = frame;
        if (Math.abs(errorDeg) <= config.placementAgreeDeg) {
            placementDisagree = 0;
            if (++placementAgree >= config.placementDecideFrames) {
                poseHeadingSeeded = true;
                Telemetry.print("Vision: auto start heading confirmed", PrintPriority.HIGH);
            }
            return;
        }
        placementAgree = 0;
        if (++placementDisagree >= config.placementDecideFrames) {
            Estimate seed = mt1Estimate(best, true);
            if (seed != null) {
                integrate(best, seed);
                poseHeadingSeeded = true;
                Telemetry.print(
                        String.format(
                                "Vision: auto start heading was WRONG by %.1f deg; re-seeded",
                                errorDeg),
                        PrintPriority.HIGH);
            }
        }
    }

    private boolean isStill() {
        ChassisSpeeds s = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond) <= config.stillLinearSpeed
                && Math.abs(s.omegaRadiansPerSecond) <= config.stillOmega;
    }

    // ── Estimates ──────────────────────────────────────────────────────────────

    private record Estimate(Pose2d pose, double timestamp, double xyStd, double headingStdDeg) {}

    /**
     * MegaTag1 estimate. {@code seed} makes it authoritative for translation and heading, which
     * only the disabled seed and the heading safety nets use.
     */
    private Estimate mt1Estimate(Limelight ll, boolean seed) {
        Pose3d pose3d = ll.getMegaTag1_Pose3d();
        if (!commonChecks(ll, pose3d.toPose2d(), pose3d)) {
            return null;
        }
        for (RawFiducial tag : ll.getRawFiducial()) {
            if (tag.ambiguity > config.maxAmbiguity) {
                rejected(ll, "High ambiguity");
                return null;
            }
        }
        double timestamp = ll.getMegaTag1PoseTimestamp();
        if (isStale(ll, timestamp)) {
            rejected(ll, "Stale");
            return null;
        }
        return seed
                ? new Estimate(pose3d.toPose2d(), timestamp, 0.01, 0.01)
                : new Estimate(
                        pose3d.toPose2d(),
                        timestamp,
                        xyStd(ll) * config.mt1Factor,
                        config.largeVariance);
    }

    /** MegaTag2 estimate: translation only, since its heading is the one we pushed. */
    private Estimate mt2Estimate(Limelight ll) {
        Pose2d pose = ll.getMegaTag2_Pose2d();
        if (!commonChecks(ll, pose, ll.getMegaTag1_Pose3d())) {
            return null;
        }
        double timestamp = ll.getMegaTag2PoseTimestamp();
        if (isStale(ll, timestamp)) {
            rejected(ll, "Stale");
            return null;
        }
        return new Estimate(pose, timestamp, xyStd(ll), config.largeVariance);
    }

    /**
     * Gates shared by both sources. Tilt and height come from the frame's MT1 3-D solve, because a
     * knocked camera or a bad mount shows there whichever pose is being fused.
     */
    private boolean commonChecks(Limelight ll, Pose2d pose, Pose3d solve3d) {
        if (!ll.targetInView()) {
            ll.sendInvalidStatus("No targets");
            return false;
        }
        if (FieldHelpers.poseOutOfField(pose)) {
            return rejected(ll, "Out of field");
        }
        if (peakYawRate() >= config.maxYawRateRadPerSec) {
            return rejected(ll, "Spinning");
        }
        if (ll.getTargetSize() <= config.minTargetSize) {
            return rejected(ll, "Target too small");
        }
        if (Math.abs(solve3d.getRotation().getX()) > Math.toRadians(config.maxTiltDegrees)
                || Math.abs(solve3d.getRotation().getY()) > Math.toRadians(config.maxTiltDegrees)) {
            return rejected(ll, "Tilted");
        }
        if (Math.abs(solve3d.getZ()) > config.maxZErrorMeters) {
            return rejected(ll, "Off the floor");
        }
        if (meanTagDistance(ll) > config.maxTagDistanceMeters) {
            return rejected(ll, "Too far");
        }
        return true;
    }

    /** 6328's model, scaled up for tags near the image edge (issue 8b). */
    private double xyStd(Limelight ll) {
        RawFiducial[] tags = ll.getRawFiducial();
        int n = Math.max(1, tags.length);
        double d = meanTagDistance(ll);
        double edge = 0;
        for (RawFiducial t : tags) {
            edge += ReefTagTracker.edgeScale(t.txnc, t.tync);
        }
        edge = tags.length == 0 ? 1 : edge / tags.length;
        return Math.max(config.minXyStd, config.xyStdCoefficient * d * d / (n * n) * edge);
    }

    private static double meanTagDistance(Limelight ll) {
        RawFiducial[] tags = ll.getRawFiducial();
        if (tags.length == 0) {
            return 0;
        }
        double sum = 0;
        for (RawFiducial t : tags) {
            sum += t.distToRobot;
        }
        return sum / tags.length;
    }

    private boolean isStale(Limelight ll, double fpgaTimestamp) {
        double age = Timer.getFPGATimestamp() - fpgaTimestamp;
        ll.setLastEstimateAgeSeconds(age);
        return age > config.maxEstimateAgeSeconds;
    }

    /** Counts and reports a rejection; always false, so gates can return it. */
    private boolean rejected(Limelight ll, String reason) {
        ll.sendInvalidStatus(reason);
        rejections.merge(reason, 1, Integer::sum);
        return false;
    }

    private void integrate(Limelight ll, Estimate e) {
        Robot.getSwerve()
                .addVisionMeasurement(
                        e.pose,
                        Utils.fpgaToCurrentTime(e.timestamp),
                        VecBuilder.fill(e.xyStd, e.xyStd, Units.degreesToRadians(e.headingStdDeg)));
        lastAcceptedFpga = Timer.getFPGATimestamp();
        ll.setIntegratedThisLoop(true);
        ll.sendValidStatus("Integrated");
    }

    /** The camera with the most tags, ties broken by target size. */
    public Limelight getBestLimelight() {
        Limelight best = frontLL;
        double bestScore = 0;
        for (Limelight ll : limelights) {
            double score = ll.getTagCountInView() * 100.0 + ll.getTargetSize();
            if (score > bestScore) {
                bestScore = score;
                best = ll;
            }
        }
        return best;
    }

    public double secondsSinceLastAcceptedEstimate() {
        return Double.isNaN(lastAcceptedFpga)
                ? Double.POSITIVE_INFINITY
                : Timer.getFPGATimestamp() - lastAcceptedFpga;
    }

    private void recordYawRate() {
        yawRates[yawIndex] =
                Math.abs(Robot.getSwerve().getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
        yawTimes[yawIndex] = Timer.getFPGATimestamp();
        yawIndex = (yawIndex + 1) % YAW_HISTORY;
    }

    /** Peak yaw rate over the lookback: frames arriving now were captured up to 0.3 s ago. */
    private double peakYawRate() {
        double cutoff = Timer.getFPGATimestamp() - config.yawRateLookbackSeconds;
        double peak = 0;
        for (int i = 0; i < YAW_HISTORY; i++) {
            if (yawTimes[i] >= cutoff) {
                peak = Math.max(peak, yawRates[i]);
            }
        }
        return peak;
    }

    // ── Telemetry, alerts, commands ────────────────────────────────────────────

    private void logTelemetry() {
        double now = Timer.getFPGATimestamp();
        for (int i = 0; i < limelights.length; i++) {
            if (limelights[i].isCameraConnected()) {
                lastConnectedSeconds[i] = now;
            }
            disconnectedAlerts[i].set(RobotBase.isReal() && now - lastConnectedSeconds[i] > 2.0);
            loggers[i].getIntegratingStatus();
            loggers[i].getIntegratedThisLoop();
            loggers[i].getLogStatus();
        }
        reefTagTracker.log();
        Telemetry.logDash("Vision/PoseHeadingSeeded", poseHeadingSeeded);
        Telemetry.logDash("Vision/PoseSeedConfirmed", poseSeedConfirmed);
        Telemetry.log("Vision/SeedConfirmProgress", seedConfirmStreak);
        if (!Telemetry.slowLogThisLoop()) {
            return;
        }
        for (VisionLogger logger : loggers) {
            logger.getCameraConnection();
            logger.getPose();
            logger.getTagCount();
            logger.getTargetSize();
            logger.getEstimateAge();
        }
        for (Map.Entry<String, Integer> r : rejections.entrySet()) {
            Telemetry.log("Vision/Rejections/" + r.getKey(), r.getValue());
        }
        Telemetry.logDash(
                "Vision/SecondsSinceAcceptedEstimate", secondsSinceLastAcceptedEstimate());
    }

    /** Keeps 165 s of camera video on each Limelight; called at the end of an FMS match. */
    public void triggerRewindCapture() {
        for (Limelight ll : limelights) {
            LimelightHelpers.triggerRewindCapture(ll.getName(), 165);
        }
    }

    /** Snaps the pose to the best camera's MegaTag1 solve (translation and heading). */
    public Command resetPoseToVisionCommand() {
        return Commands.runOnce(
                        () -> {
                            Limelight best = getBestLimelight();
                            Estimate e = mt1Estimate(best, true);
                            if (e != null) {
                                integrate(best, e);
                                poseHeadingSeeded = true;
                            }
                        })
                .ignoringDisable(true)
                .withName("Vision.resetPoseToVision");
    }

    /** Reef tag of our alliance nearest the robot. */
    public static int nearestReefTag() {
        return Field.nearestReefTag(
                Robot.getSwerve().getRobotPose().getTranslation(), FieldHelpers.isRed());
    }
}
