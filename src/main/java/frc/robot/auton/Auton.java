package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotStates;
import frc.robot.swerve.SwerveStates;
import frc.spectrumLib.Telemetry;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class Auton {

    // Setup EventTriggers
    // Should all be public static final
    public static final EventTrigger autonGroundIntake = new EventTrigger("groundIntake");
    public static final EventTrigger autonSourceIntakeOn = new EventTrigger("sourceIntakeOn");
    public static final EventTrigger autonSourceIntakeOff = new EventTrigger("sourceIntakeOff");
    public static final EventTrigger autonLowAlgae = new EventTrigger("lowAlgae");
    public static final EventTrigger autonHighAlgae = new EventTrigger("highAlgae");
    public static final EventTrigger autonPreScore = new EventTrigger("prescore");
    public static final EventTrigger autonScore = new EventTrigger("score");
    public static final EventTrigger autonLeft = new EventTrigger("left");
    public static final EventTrigger autonRight = new EventTrigger("right");
    public static final EventTrigger autonL1 = new EventTrigger("L1");
    public static final EventTrigger autonNet = new EventTrigger("net");
    public static final EventTrigger autonProcessor = new EventTrigger("processor");
    public static final EventTrigger autonClearStates = new EventTrigger("clearStates");
    public static final EventTrigger autonCoral = new EventTrigger("coral");
    public static final EventTrigger autonHome = new EventTrigger("home");
    public static final EventTrigger autonActionOn = new EventTrigger("actionOn");
    public static final EventTrigger autonActionOff = new EventTrigger("actionOff");
    public static final EventTrigger autonCoralStage = new EventTrigger("coralStage");
    public static final EventTrigger autonShoulderL4 = new EventTrigger("shoulderL4");
    public static final EventTrigger autonTwistL4R = new EventTrigger("twistL4R");
    public static final EventTrigger autonTwistL4L = new EventTrigger("twistL4L");
    public static final EventTrigger autonSwerveAlign = new EventTrigger("autoAlign");
    public static final EventTrigger autonClearFeedback = new EventTrigger("clearFeedBack");
    public static final EventTrigger autonL4 = new EventTrigger("L4");
    public static final EventTrigger autonHomeOff = new EventTrigger("homeOff");

    private final SendableChooser<Command> pathChooser = new SendableChooser<>();
    private boolean autoMessagePrinted = true;
    private double autonStart = 0;

    /**
     * This method configures the available autonomous routines that can be selected from the
     * SmartDashboard.
     */
    public void setupSelectors() {

        pathChooser.setDefaultOption("Do Nothing", Commands.print("Do Nothing Auto ran"));

        // pathChooser.addOption("1 Meter", SpectrumAuton("1 Meter", false));
        // pathChooser.addOption("3 Meter", SpectrumAuton("3 Meter", false));
        // pathChooser.addOption("5 Meter", SpectrumAuton("5 Meter", false));

        // pathChooser.addOption("test", test());

        pathChooser.addOption("Left | Source L4", sourceL4(false));
        pathChooser.addOption("Right | Source L4", sourceL4(true));

        pathChooser.addOption("Drive Forward", SpectrumAuton("Drive Forward", false));

        // pathChooser.addOption("Left | Algae Rush", centerAlgae(false));
        // pathChooser.addOption("Right | Algae Rush", centerAlgae(true));

        SmartDashboard.putData("Auto Chooser", pathChooser);
    }

    public Auton() {
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard
        Telemetry.print("Auton Subsystem Initialized: ");
    }

    public void init() {
        Command autonCommand = getAutonomousCommand();

        if (autonCommand != null) {
            autonCommand.schedule();
            startAutonTimer();
        } else {
            Telemetry.print("No Auton Command Found");
        }
    }

    public void exit() {
        printAutoDuration();
    }

    public Command test() {
        return (SwerveStates.swerveTest());
    }

    public Command sourceL4(boolean mirrored) {
        return (RobotStates.homeAll
                        .setFalse()
                        .alongWith(SpectrumAuton("L4-SideStart", mirrored).withTimeout(2))
                        .andThen(
                                aimL4score(2.5),
                                SpectrumAuton("TroughRush", mirrored),
                                aimL4score(2.5),
                                SpectrumAuton("TroughRush2", mirrored)))
                .withName("Blue Left - Source L4");
    }

    public Command centerAlgae(boolean mirrored) {
        return (SpectrumAuton("1 - Blue Center Algae", mirrored)
                .withTimeout(1)
                .andThen(
                        aimHighAlgae(1.5),
                        SpectrumAuton("2 - Blue Center Algae", mirrored),
                        algaeNet(),
                        SpectrumAuton("3 - Blue Center Algae", mirrored),
                        aimLowAlgae(1.5),
                        SpectrumAuton("4 - Blue Center Algae", mirrored),
                        algaeNet(),
                        SpectrumAuton("5 - Blue Center Algae", mirrored),
                        aimLowAlgae(1.5),
                        SpectrumAuton("6 - Blue Center Algae", mirrored),
                        algaeNet())
                .withName("Blue Center Algae Rush"));
    }

    public Command aimL4score(double alignTime) {
        return SwerveStates.reefAimDrive().withTimeout(alignTime).alongWith(l4score());
    }

    public Command aimL1score(double alignTime) {
        return SwerveStates.reefAimDrive().withTimeout(alignTime).alongWith(l1score());
    }

    public Command aimLowAlgae(double alignTime) {
        return SwerveStates.reefAimDrive().withTimeout(alignTime).alongWith(lowAlgae());
    }

    public Command aimHighAlgae(double alignTime) {
        return SwerveStates.reefAimDrive().withTimeout(alignTime).alongWith(highAlgae());
    }

    public Command algaeNet() {
        return Commands.waitSeconds(0.05)
                .andThen(
                        RobotStates.algae
                                .setTrue()
                                .alongWith(RobotStates.l4.setTrue(), RobotStates.homeAll.setFalse())
                                .andThen(
                                        Commands.waitSeconds(0.05),
                                        RobotStates.actionPrepState.setTrue(),
                                        Commands.waitSeconds(1.1),
                                        RobotStates.actionPrepState.setFalse(),
                                        Commands.waitSeconds(0.5),
                                        RobotStates.clearStates(),
                                        RobotStates.homeAll.setTrue()));
    }

    public Command lowAlgae() {
        return Commands.waitSeconds(0.05)
                .andThen(
                        RobotStates.algae
                                .setTrue()
                                .alongWith(RobotStates.l2.setTrue(), RobotStates.homeAll.setFalse())
                                .andThen(
                                        Commands.waitSeconds(0.05),
                                        RobotStates.actionPrepState.setTrue(),
                                        Commands.waitSeconds(1.1),
                                        RobotStates.actionPrepState.setFalse(),
                                        Commands.waitSeconds(0.5),
                                        RobotStates.clearStates(),
                                        RobotStates.homeAll.setTrue()));
    }

    public Command highAlgae() {
        return Commands.waitSeconds(0.05)
                .andThen(
                        RobotStates.algae
                                .setTrue()
                                .alongWith(RobotStates.l3.setTrue(), RobotStates.homeAll.setFalse())
                                .andThen(
                                        Commands.waitSeconds(0.05),
                                        RobotStates.actionPrepState.setTrue(),
                                        Commands.waitSeconds(1.1),
                                        RobotStates.actionPrepState.setFalse(),
                                        Commands.waitSeconds(0.5),
                                        RobotStates.clearStates(),
                                        RobotStates.homeAll.setTrue()));
    }

    public Command l4score() {
        return Commands.waitSeconds(0.15)
                .andThen(
                        RobotStates.coral
                                .setTrue()
                                .alongWith(RobotStates.l4.setTrue(), RobotStates.homeAll.setFalse())
                                .andThen(
                                        Commands.waitSeconds(0.05),
                                        RobotStates.actionPrepState.setTrue(),
                                        Commands.waitSeconds(1.1),
                                        RobotStates.actionPrepState.setFalse(),
                                        Commands.waitSeconds(0.5),
                                        RobotStates.clearStates(),
                                        RobotStates.homeAll.setTrue(),
                                        Commands.waitSeconds(.5)));
    }

    public Command l1score() {
        return Commands.waitSeconds(0.05)
                .andThen(
                        RobotStates.coral
                                .setTrue()
                                .alongWith(RobotStates.l1.setTrue(), RobotStates.homeAll.setFalse())
                                .andThen(
                                        Commands.waitSeconds(0.05),
                                        RobotStates.actionPrepState.setTrue(),
                                        Commands.waitSeconds(1.1),
                                        RobotStates.actionPrepState.setFalse(),
                                        Commands.waitSeconds(0.5),
                                        RobotStates.clearStates(),
                                        RobotStates.homeAll.setTrue()));
    }

    /**
     * Creates a SpectrumAuton command sequence.
     *
     * <p>This method generates a command sequence that first waits for 0.01 seconds and then
     * executes a PathPlannerAuto command with the specified autonomous routine name.
     *
     * @param autoName the name of the autonomous routine to execute
     * @param mirrored whether the autonomous routine should be mirrored
     * @return a Command that represents the SpectrumAuton sequence
     */
    public Command SpectrumAuton(String autoName, boolean mirrored) {
        Command autoCommand = new PathPlannerAuto(autoName, mirrored);
        return (Commands.waitSeconds(0.01).andThen(autoCommand)).withName(autoName);
    }

    /**
     * Retrieves the autonomous command selected on the shuffleboard.
     *
     * @return the selected autonomous command if one is chosen; otherwise, returns a PrintCommand
     *     indicating that the autonomous command is null.
     */
    public Command getAutonomousCommand() {
        Command auton = pathChooser.getSelected(); // sees what auto is chosen on shuffleboard
        if (auton != null) {
            return auton; // checks to make sure there is an auto and if there is it runs an auto
        } else {
            return new PrintCommand(
                    "*** AUTON COMMAND IS NULL ***"); // runs if there is no auto chosen, which
            // shouldn't happen because of the default
            // auto set to nothing which still runs
            // something
        }
    }

    /** This method is called in AutonInit */
    public void startAutonTimer() {
        autonStart = Timer.getFPGATimestamp();
        autoMessagePrinted = false;
    }

    /** Called at AutonExit and displays the duration of the auton command Based on 6328 code */
    public void printAutoDuration() {
        Command autoCommand = getAutonomousCommand();
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    Telemetry.print(
                            String.format(
                                    "*** Auton finished in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                } else {
                    Telemetry.print(
                            String.format(
                                    "*** Auton CANCELLED in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                }
                autoMessagePrinted = true;
            }
        }
    }

    public static Command followSinglePath(String pathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger event
            // markers.
            return AutoBuilder.followPath(path);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new PrintCommand("ERROR LOADING PATH");
    }

    public static Command pathfindingCommandToPose(
            double xPos, double yPos, double rotation, double vel, double accel) {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(xPos, yPos, Rotation2d.fromDegrees(rotation));

        // Create the constraints to use while pathfinding
        PathConstraints constraints =
                new PathConstraints(
                        vel, accel, Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand =
                AutoBuilder.pathfindToPoseFlipped(
                        targetPose, constraints, 0.0 // Goal end velocity in meters/sec
                        );

        return pathfindingCommand;
    }
    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
