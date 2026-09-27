package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field.ReefHeight;
import frc.robot.auton.Auton;
import frc.robot.configs.FM2025;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Branch;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbConfig;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.Elbow.ElbowConfig;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorConfig;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeConfig;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.Shoulder.ShoulderConfig;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.twist.Twist;
import frc.robot.subsystems.twist.Twist.TwistConfig;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConfig;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.framework.SpectrumRobot;
import frc.spectrumLib.telemetry.BatteryLogger;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import frc.spectrumLib.util.CrashTracker;
import frc.spectrumLib.util.Util;
import lombok.Getter;

/**
 * Entry point. Builds the subsystems, binds the gamepads once, and runs each loop in a fixed order:
 * vision (pose), superstructure (decisions), then the scheduler (mechanism outputs).
 */
public class Robot extends SpectrumRobot {
    @Getter private static Config config;
    @Getter private static final Field2d field2d = new Field2d();
    public static Telemetry telemetry = new Telemetry();

    public static class Config {
        public final SwerveConfig swerve = new SwerveConfig();
        public final PilotConfig pilot = new PilotConfig();
        public final OperatorConfig operator = new OperatorConfig();
        public final ElevatorConfig elevator = new ElevatorConfig();
        public final ShoulderConfig shoulder = new ShoulderConfig();
        public final ElbowConfig elbow = new ElbowConfig();
        public final TwistConfig twist = new TwistConfig();
        public final IntakeConfig intake = new IntakeConfig();
        public final ClimbConfig climb = new ClimbConfig();
        public final VisionConfig vision = new VisionConfig();
    }

    @Getter private static Swerve swerve;
    @Getter private static Elevator elevator;
    @Getter private static Shoulder shoulder;
    @Getter private static Elbow elbow;
    @Getter private static Twist twist;
    @Getter private static Intake intake;
    @Getter private static Climb climb;
    @Getter private static Vision vision;
    @Getter private static Pilot pilot;
    @Getter private static Operator operator;
    @Getter private static SuperStructure superStructure;
    @Getter private static Auton auton;
    @Getter private static BatteryLogger batteryLogger;
    private static Leds leds;
    private static RobotSim robotSim;
    private static SimScenario scenario;

    private boolean autonWarmedUp = false;

    public Robot() {
        super();
        // Nobody replays .hoot logs; SignalLogger.start() still works for a deliberate capture.
        SignalLogger.enableAutoLogging(false);
        // Mirror logs to NetworkTables only in the simulator, for AdvantageScope's live view; on
        // the robot the whole-log mirror costs field bandwidth (see Telemetry.start).
        Telemetry.start(
                RobotBase.isSimulation(), true, false, true, false, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");
            // Only one 2025 robot is left; any rio (and the sim) gets its config.
            config = new FM2025();

            batteryLogger = new BatteryLogger();
            pilot = new Pilot(config.pilot);
            operator = new Operator(config.operator);
            swerve = new Swerve(config.swerve);
            elevator = new Elevator(config.elevator);
            shoulder = new Shoulder(config.shoulder);
            elbow = new Elbow(config.elbow);
            twist = new Twist(config.twist);
            intake = new Intake(config.intake);
            climb = new Climb(config.climb);
            vision = new Vision(config.vision);
            superStructure =
                    new SuperStructure(
                            swerve,
                            elevator,
                            shoulder,
                            elbow,
                            twist,
                            intake,
                            climb,
                            vision.getReefTagTracker());
            auton = new Auton(superStructure);
            leds = new Leds(superStructure);
            if (RobotBase.isSimulation()) {
                robotSim = new RobotSim(superStructure);
            }
            configureBindings();
            Auton.verifyAutoFiles();
            batteryLogger.setEnabled(true);
            Telemetry.print("--- Robot Init Complete ---");
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        RobotController.setBrownoutVoltage(Units.Volts.of(6.0));
        Telemetry.logDashAlways("BuildConstants/GitSHA", BuildConstants.GIT_SHA);
        Telemetry.logDashAlways("BuildConstants/GitBranch", BuildConstants.GIT_BRANCH);
        Telemetry.logDashAlways("BuildConstants/BuildDate", BuildConstants.BUILD_DATE);
    }

    // ── Bindings, made once ────────────────────────────────────────────────────

    /**
     * Held while the button is held: pressing requests the state, releasing goes back to idle, but
     * only if nothing else has taken over in the meantime.
     */
    private void hold(Trigger button, WantedSuperState state) {
        button.onTrue(superStructure.setStateCommand(state));
        button.onFalse(releaseTo(state, WantedSuperState.IDLE));
    }

    /** Hold to stage, release to score. */
    private void stageThenScore(Trigger button, WantedSuperState stage, WantedSuperState score) {
        button.onTrue(superStructure.setStateCommand(stage));
        button.onFalse(releaseTo(stage, score));
    }

    private Command releaseTo(WantedSuperState from, WantedSuperState to) {
        return Commands.runOnce(
                () -> {
                    if (superStructure.getWantedSuperState() == from) {
                        superStructure.setWantedSuperState(to);
                    }
                });
    }

    private Command select(ReefHeight level) {
        return Commands.runOnce(() -> superStructure.setSelectedLevel(level)).ignoringDisable(true);
    }

    private Command select(Branch branch) {
        return Commands.runOnce(() -> superStructure.setSelectedBranch(branch))
                .ignoringDisable(true);
    }

    public void configureBindings() {
        hold(pilot.groundCoral_LT, WantedSuperState.GROUND_CORAL);
        hold(pilot.stationCoral_LB, WantedSuperState.STATION_CORAL);
        hold(pilot.autoScore_RT, WantedSuperState.AUTO_SCORE_CORAL);
        hold(pilot.reefAlgae_A, WantedSuperState.REEF_ALGAE);
        hold(pilot.groundAlgae_X, WantedSuperState.GROUND_ALGAE);
        stageThenScore(
                pilot.stageCoral_RB, WantedSuperState.STAGE_CORAL, WantedSuperState.SCORE_CORAL);
        stageThenScore(pilot.net_B, WantedSuperState.STAGE_NET, WantedSuperState.SCORE_NET);
        stageThenScore(
                pilot.processor_Y,
                WantedSuperState.STAGE_PROCESSOR,
                WantedSuperState.SCORE_PROCESSOR);
        hold(pilot.home_select, WantedSuperState.FORCE_HOME);
        hold(operator.home_select, WantedSuperState.FORCE_HOME);

        pilot.upReorient.onTrue(swerve.reorientForward());
        pilot.leftReorient.onTrue(swerve.reorientLeft());
        pilot.downReorient.onTrue(swerve.reorientBack());
        pilot.rightReorient.onTrue(swerve.reorientRight());
        pilot.visionPoseReset_select.onTrue(vision.resetPoseToVisionCommand());

        // Rumble when a piece lands in the claw, and when the score gate opens.
        new Trigger(intake::hasPiece).onTrue(pilot.rumbleCommand(0.6, 0.3));
        new Trigger(superStructure::isScoreGateOpen).onTrue(pilot.rumbleCommand(1.0, 0.2));

        operator.l1_A.onTrue(select(ReefHeight.L1));
        operator.l2_B.onTrue(select(ReefHeight.L2));
        operator.l3_X.onTrue(select(ReefHeight.L3));
        operator.l4_Y.onTrue(select(ReefHeight.L4));
        operator.leftBranch.onTrue(select(Branch.LEFT));
        operator.rightBranch.onTrue(select(Branch.RIGHT));
        hold(operator.eject_LT, WantedSuperState.EJECT);
        hold(operator.climbHeld, WantedSuperState.CLIMB_PREP);
        operator.climbDrive.onTrue(superStructure.setStateCommand(WantedSuperState.CLIMB));
        climb.setManualInput(operator::getClimbPercent);

        Command coast =
                Commands.parallel(
                                elevator.coastMode(),
                                shoulder.coastMode(),
                                elbow.coastMode(),
                                twist.coastMode(),
                                climb.coastMode())
                        .ignoringDisable(true)
                        .withName("Robot.coast");
        Command brake =
                Commands.parallel(
                                elevator.ensureBrakeMode(),
                                shoulder.ensureBrakeMode(),
                                elbow.ensureBrakeMode(),
                                twist.ensureBrakeMode(),
                                climb.ensureBrakeMode())
                        .ignoringDisable(true)
                        .withName("Robot.brake");
        pilot.coast_A.or(operator.coast_A).onTrue(coast);
        pilot.brake_B.or(operator.brake_B).onTrue(brake);
        Util.disabled.onFalse(brake);
    }

    @Override
    public void robotInit() {
        SmartDashboard.putData("Field2d", field2d);
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    @Override
    public void robotPeriodic() {
        RobotLoop.next();
        try {
            vision.periodic();
            superStructure.periodic();
            CommandScheduler.getInstance().run();
            leds.periodic();
            batteryLogger.setBatteryVoltage(RobotController.getBatteryVoltage());
            batteryLogger.setRioCurrent(RobotController.getInputCurrent());
            batteryLogger.logPower();
            field2d.setRobotPose(swerve.getRobotPose());
            Telemetry.log("Match Data/MatchTime", DriverStation.getMatchTime(), "seconds");
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    // ── Modes ──────────────────────────────────────────────────────────────────

    @Override
    public void disabledInit() {
        System.gc(); // Take the pause here, where it costs nothing.
        if (!autonWarmedUp) {
            CommandScheduler.getInstance()
                    .schedule(
                            Commands.sequence(
                                            FollowPathCommand.warmupCommand(),
                                            PathfindingCommand.warmupCommand())
                                    .ignoringDisable(true));
            autonWarmedUp = true;
        }
    }

    @Override
    public void autonomousInit() {
        superStructure.setWantedSuperState(WantedSuperState.IDLE);
        Command auto = auton.getAutonomousCommand();
        Telemetry.print("Auto running: " + auto.getName(), PrintPriority.HIGH);
        CommandScheduler.getInstance().schedule(auto);
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
        superStructure.setWantedSuperState(WantedSuperState.IDLE);
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        superStructure.setWantedSuperState(WantedSuperState.IDLE);
    }

    @Override
    public void teleopExit() {
        if (DriverStation.isFMSAttached()) {
            vision.triggerRewindCapture();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        superStructure.setWantedSuperState(WantedSuperState.IDLE);
    }

    @Override
    public void simulationInit() {
        if (System.getenv("SIM_SCENARIO") != null) {
            scenario = new SimScenario();
        }
    }

    @Override
    public void simulationPeriodic() {
        robotSim.periodic();
        if (scenario != null) {
            scenario.periodic();
        }
    }
}
