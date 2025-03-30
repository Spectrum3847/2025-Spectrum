package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.Auton;
import frc.robot.climb.Climb;
import frc.robot.climb.Climb.ClimbConfig;
import frc.robot.configs.FM2025;
import frc.robot.configs.PHOTON2025;
import frc.robot.configs.PM2025;
import frc.robot.elbow.Elbow;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.intake.Intake;
import frc.robot.intake.Intake.IntakeConfig;
import frc.robot.leds.LedFull;
import frc.robot.leds.LedFull.LedFullConfig;
import frc.robot.operator.Operator;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.shoulder.Shoulder;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConfig;
import frc.robot.swerve.SwerveStates;
import frc.robot.twist.Twist;
import frc.robot.twist.Twist.TwistConfig;
import frc.robot.vision.Vision;
import frc.robot.vision.Vision.VisionConfig;
import frc.robot.vision.VisionSystem;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumRobot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import frc.spectrumLib.util.CrashTracker;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import lombok.Getter;
import org.json.simple.parser.ParseException;

public class Robot extends SpectrumRobot {
    @Getter private static RobotSim robotSim;
    @Getter private static Config config;
    static Telemetry telemetry = new Telemetry();
    @Getter private static final Field2d field2d = new Field2d();

    public enum RobotFault {
        OVERCURRENT,
    }

    public static class Config {
        public SwerveConfig swerve = new SwerveConfig();

        public PilotConfig pilot = new PilotConfig();
        public OperatorConfig operator = new OperatorConfig();
        public ElevatorConfig elevator = new ElevatorConfig();
        public ShoulderConfig shoulder = new ShoulderConfig();

        public IntakeConfig intake = new IntakeConfig();
        public LedFullConfig leds = new LedFullConfig();
        public ClimbConfig climb = new ClimbConfig();
        public ElbowConfig elbow = new ElbowConfig();
        public TwistConfig twist = new TwistConfig();
        public VisionConfig vision = new VisionConfig();
    }

    @Getter private static Swerve swerve;
    @Getter private static Elevator elevator;
    @Getter private static Intake intake;
    @Getter private static LedFull leds;
    @Getter private static Operator operator;
    @Getter private static Pilot pilot;
    @Getter private static VisionSystem visionSystem;
    @Getter private static Vision vision;
    @Getter private static Auton auton;
    @Getter private static Climb climb;
    @Getter private static Elbow elbow;
    @Getter private static Shoulder shoulder;
    @Getter private static Twist twist;
    public static boolean commandInit = false;

    public Robot() {
        super();
        Telemetry.start(true, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");
            robotSim = new RobotSim();

            /** Set up the config */
            switch (Rio.id) {
                case PHOTON_2025:
                    config = new PHOTON2025();
                    break;
                case PM_2025:
                    config = new PM2025();
                    break;
                case FM_2025:
                    config = new FM2025();
                    break;
                default: // SIM and UNKNOWN
                    config = new FM2025();
                    break;
            }

            /**
             * Initialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are also subsystems.
             */
            double canInitDelay = 0.1; // Delay between any mechanism with motor/can configs

            leds = new LedFull(config.leds);
            operator = new Operator(config.operator);
            pilot = new Pilot(config.pilot);
            swerve = new Swerve(config.swerve);
            Timer.delay(canInitDelay);
            elevator = new Elevator(config.elevator);
            Timer.delay(canInitDelay);
            climb = new Climb(config.climb);
            Timer.delay(canInitDelay);
            shoulder = new Shoulder(config.shoulder);
            Timer.delay(canInitDelay);
            elbow = new Elbow(config.elbow);
            Timer.delay(canInitDelay);
            intake = new Intake(config.intake);
            Timer.delay(canInitDelay);
            vision = new Vision(config.vision);
            visionSystem = new VisionSystem(swerve::getRobotPose);
            Timer.delay(canInitDelay);
            twist = new Twist(config.twist);
            auton = new Auton();

            // Setup Default Commands for all subsystems
            setupDefaultCommands();

            Telemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This method cancels all commands and returns subsystems to their default commands and the
     * gamepad configs are reset so that new bindings can be assigned based on mode This method
     * should be called when each mode is initialized
     */
    public void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Reset Config for all gamepads and other button bindings
        pilot.resetConfig();
        operator.resetConfig();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
        RobotStates.clearStates().schedule();
    }

    public void clearCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
        RobotStates.clearStates().schedule();
    }

    public void setupAutoVisualizer() {
        SmartDashboard.putData("Field2d", field2d);
    }

    @Override // Deprecated
    public void robotInit() {
        setupAutoVisualizer();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    /* ROBOT PERIODIC  */
    /**
     * This method is called periodically the entire time the robot is running. Periodic methods are
     * called every 20 ms (50 times per second) by default Since the robot software is always
     * looping you shouldn't pause the execution of the robot code This ensures that new values are
     * updated from the gamepads and sent to the motors
     */
    @Override
    public void robotPeriodic() {
        try {
            /**
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            CommandScheduler.getInstance().run();

            SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
            field2d.setRobotPose(swerve.getRobotPose());
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        Telemetry.print("### Disabled Init Starting ### ");
        resetCommandsAndButtons();

        if (!commandInit) {
            Command AutonStartCommand =
                    FollowPathCommand.warmupCommand()
                            .andThen(
                                    PathfindingCommand.warmupCommand()
                                            .andThen(
                                                    SwerveStates.reefAimDrive()
                                                            .ignoringDisable(true)
                                                            .withTimeout(0.5),
                                                    auton.sourceL4(false)
                                                            .ignoringDisable(true)
                                                            .withTimeout(0.5)));
            AutonStartCommand.schedule();
            commandInit = true;
        }

        Telemetry.print("### Disabled Init Complete ### ");
    }

    @Override
    public void disabledPeriodic() {
        String autoName = "";
        String newAutoName;
        List<PathPlannerPath> pathPlannerPaths = new ArrayList<>();
        newAutoName = (auton.getAutonomousCommand()).getName();
        if (!autoName.equals(newAutoName)) {
            autoName = newAutoName;
            if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                try {
                    pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                } catch (IOException | ParseException e) {
                    Telemetry.print("Could not load path planner paths");
                }
                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                    poses.addAll(
                            path.getAllPathPoints().stream()
                                    .map(
                                            point ->
                                                    new Pose2d(
                                                            point.position.getX(),
                                                            point.position.getY(),
                                                            new Rotation2d()))
                                    .collect(Collectors.toList()));
                }
                field2d.getObject("path").setPoses(poses);
            }
        }
    }

    @Override
    public void disabledExit() {
        RobotStates.coastMode.setFalse(); // Ensure motors are in brake mode
        Telemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    /**
     * This mode is run when the DriverStation Software is set to autonomous and enabled. In this
     * mode the robot is not able to read values from the gamepads
     */

    /** This method is called once when autonomous starts */
    @Override
    public void autonomousInit() {
        try {
            Telemetry.print("@@@ Auton Init Starting @@@ ");
            clearCommandsAndButtons();

            auton.init();

            Telemetry.print("@@@ Auton Init Complete @@@ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        auton.exit();
        Telemetry.print("@@@ Auton Exit @@@ ");
    }

    @Override
    public void teleopInit() {
        try {
            Telemetry.print("!!! Teleop Init Starting !!! ");
            resetCommandsAndButtons();
            field2d.getObject("path").setPoses(new ArrayList<>()); // clears auto visualizer

            Telemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        Telemetry.print("!!! Teleop Exit !!! ");
    }

    /* TEST MODE */
    /**
     * This mode is run when the DriverStation Software is set to test and enabled. In this mode the
     * is fully enabled and can move it's outputs and read values from the gamepads. This mode is
     * never enabled by the competition field It can be used to test specific features or modes of
     * the robot
     */

    /** This method is called once when test mode starts */
    @Override
    public void testInit() {
        try {

            Telemetry.print("~~~ Test Init Starting ~~~ ");
            resetCommandsAndButtons();

            Telemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        Telemetry.print("~~~ Test Exit ~~~ ");
    }

    /* SIMULATION MODE */
    /**
     * This mode is run when the software is running in simulation and not on an actual robot. This
     * mode is never enabled by the competition field
     */

    /** This method is called once when a simulation starts */
    @Override
    public void simulationInit() {
        Telemetry.print("$$$ Simulation Init Starting $$$ ");

        Telemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    /** This method is called periodically during simulation. */
    @Override
    public void simulationPeriodic() {}
}
