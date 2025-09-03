package frc.robot.auton;

import static frc.robot.RobotStates.autonAutoScoreMode;

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
    public static final EventTrigger autonNet = new EventTrigger("net");
    public static final EventTrigger autonClearStates = new EventTrigger("clearStates");
    public static final EventTrigger autonHome = new EventTrigger("home");
    public static final EventTrigger autonActionOn = new EventTrigger("actionOn");
    public static final EventTrigger autonActionOff = new EventTrigger("actionOff");
    public static final EventTrigger autonShoulderL4 = new EventTrigger("shoulderL4");
    public static final EventTrigger autonTwistL4R = new EventTrigger("twistL4R");
    public static final EventTrigger autonTwistL4L = new EventTrigger("twistL4L");
    public static final EventTrigger autonLeft = new EventTrigger("left");
    public static final EventTrigger autonRight = new EventTrigger("right");
    public static final EventTrigger autonCoral = new EventTrigger("coral");
    public static final EventTrigger autonAtL4Coral = new EventTrigger("atL4Coral");
    public static final EventTrigger autonAlgae = new EventTrigger("algae");
    public static final EventTrigger autonL1 = new EventTrigger("L1");
    public static final EventTrigger autonL2 = new EventTrigger("L2");
    public static final EventTrigger autonL3 = new EventTrigger("L3");
    public static final EventTrigger autonL4 = new EventTrigger("L4");
    public static final EventTrigger autonReverse = new EventTrigger("reverse");
    public static final EventTrigger autonPoseUpdate = new EventTrigger("poseUpdate");
    public static final EventTrigger autonAutoScore = new EventTrigger("autoScore");

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

        // pathChooser.addOption("Left | 2 L4 Coral", houston2coral(false));
        // pathChooser.addOption("Right | 2 L4 Coral", houston2coral(true));

        pathChooser.addOption("Left | 3 L4 Coral", worlds3coral(false));
        pathChooser.addOption("Right | 3 L4 Coral", worlds3coral(true));

<<<<<<< HEAD
        pathChooser.addOption("roiSecretAuto", secretAuto(false));
        pathChooser.addOption("lefSecretAuto", secretAuto(true));

        pathChooser.addOption("roiSecretAutoTest", secretAutoTest(false));
        pathChooser.addOption("lefSecretAutoTest", secretAutoTest(true));

        // pathChooser.addOption("Left | 2.5-L4 Belton Auto", beltonAuton2(false));
        // pathChooser.addOption("Right | 2.5-L4 Belton Auto", beltonAuton2(true));
=======
        pathChooser.addOption("Center | 3 Net Algae", worlds3algae(false));

        pathChooser.addOption("test", practiceAuto());

        pathChooser.addOption("Drive Forward", SpectrumAuton("Drive Forward", false));
>>>>>>> main

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

    public Command houston2coral(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("H2C-Start", mirrored, 2),
                        fullSequenceAimL4Score(1.5),
                        SpectrumAuton("H2C-Leg1", mirrored),
                        fullSequenceAimL4Score(1.5),
                        SpectrumAuton("H2C-Leg2", mirrored))
                .withName("Houston 2 Coral");
    }

    public Command worlds3coral(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("W3C-Start", mirrored, 2),
                        autoScore(),
                        SpectrumAuton("W3C-Leg1", mirrored),
                        autoScore(),
                        SpectrumAuton("W3C-Leg2", mirrored),
                        autoScore(),
                        RobotStates.homeAll.toggleToTrue(),
                        RobotStates.autonClearStates())
                .withName("Worlds 3 Coral");
    }

    public Command worlds3algae(boolean mirrored) {
        return Commands.sequence(
                        SpectrumAuton("W3A-Start", mirrored),
                        autoScoreThenAlgae(),
                        SpectrumAuton("W3A-End", mirrored))
                .withName("W3A-Full");
    }

    public Command practiceAuto() {
        return Commands.sequence(
                        SpectrumAuton("1", false),
                        autoScore(),
                        RobotStates.homeAll.toggleToTrue(),
                        RobotStates.autonClearStates(),
                        SpectrumAuton("2", false),
                        autoScore(),
                        RobotStates.homeAll.toggleToTrue(),
                        RobotStates.autonClearStates())
                .withName("test");
    }

    public Command aimScore(double alignTime) {
        return SwerveStates.reefAimDriveVisionXY()
                .withTimeout(alignTime)
                .alongWith(autonScore())
                .withName("Auton.aimL4Score");
    }

    // vision aligns until autoScore scored or 5 seconds have passed without auto scoring
    public Command autoScore() {
        return (SwerveStates.reefAimDriveVisionXY().withTimeout(5))
                .until(autonAutoScoreMode.not())
                .andThen(
                        Commands.sequence(
                                        RobotStates.actionPrepState.setFalse(),
                                        RobotStates.actionState
                                                .setTrueForTimeWithCancel(
                                                        () -> 0.75, RobotStates.actionPrepState)
                                                .andThen(
                                                        autonAutoScoreMode
                                                                .setFalse()
                                                                .onlyIf(
                                                                        RobotStates.actionPrepState
                                                                                .not())))
                                .onlyWhile(autonAutoScoreMode))
                .withName("Auton.autoScore");
    }

<<<<<<< HEAD
    public Command secretAuto(boolean mirrored) {
        return SpectrumAuton("SecretAuto", mirrored)
            .withTimeout(-2)
            .andThen(
                SpectrumAuton("SecretAuto0.5"),
                aimL4score(),
                SpectrumAuton("SecretAuto"),
                aimL4score()
            );
    }

    public Command secretAutoTest(boolean mirrored) {
        return SpectrumAuton("SecretAutoTest", mirrored)
            .withTimeout(-2)
            .andThen(
                SpectrumAuton("SecretAutoTest0.5"),
                aimL4score(),
                SpectrumAuton("SecretAutoTest"),
                aimL4score()
            );
    }

    // Aaron's code (To see the difference between ours and his) 
    // public Command secretAutoTest(boolean mirrored) {
    //     return (SpectrumAuton("TEST1", mirrored).andThen(SpectrumAuton("TEST2", mirrored)))
    //             .withName("TEST FULL");
    // }

    // public Command secretAuto(boolean mirrored) {
    //     return (SpectrumAuton("SecretAuto0.5", mirrored)
    //             .andThen(SpectrumAuton("SecretAuto", mirrored)));
    // }

    public Command aimL4score() {
        return SwerveStates.reefAimDrive().withTimeout(1.2).alongWith(l4score());
=======
    public Command aimScoreThenAlgae(double alignTime) {
        return Commands.sequence(
                        aimScore(alignTime),
                        Commands.waitSeconds(0.5),
                        RobotStates.clearStates(),
                        RobotStates.l2.setTrue(),
                        RobotStates.algae.setTrue(),
                        Commands.waitSeconds(0.05),
                        SwerveStates.autonAlgaeReefAimDriveVisionXY().withTimeout(.25),
                        RobotStates.actionPrepState.setTrue(),
                        Commands.waitSeconds(0.2),
                        SwerveStates.autonAlgaeDriveIntake(0.4))
                .withName("Auton.aimL4ScoreThenAlgae");
>>>>>>> main
    }

    public Command autoScoreThenAlgae() {
        return Commands.sequence(
                        autoScore(),
                        RobotStates.clearStates(),
                        RobotStates.l2.setTrue(),
                        RobotStates.algae.setTrue(),
                        Commands.waitSeconds(0.05),
                        SwerveStates.autonAlgaeReefAimDriveVisionXY().withTimeout(.25),
                        RobotStates.actionPrepState.setTrue(),
                        Commands.waitSeconds(0.2),
                        SwerveStates.autonAlgaeDriveIntake(0.5))
                .withName("Auton.autoL4ScoreThenAlgae");
    }

    public Command fullSequenceAimL4Score(double alignTime) {
        return SwerveStates.reefAimDriveVisionXY()
                .withTimeout(alignTime)
                .alongWith(fullSequenceL4score())
                .withName("Auton.oldAimL4Score");
    }

    public Command autonScore() {
        return Commands.sequence(Commands.waitSeconds(.75), RobotStates.actionPrepState.setFalse())
                .withName("Auton.L4Score");
    }

    public Command fullSequenceL4score() {
        return Commands.waitSeconds(0.15)
                .andThen(
                        RobotStates.coral
                                .setTrue()
                                .alongWith(RobotStates.l4.setTrue(), RobotStates.homeAll.setFalse())
                                .andThen(
                                        Commands.waitSeconds(0.05),
                                        RobotStates.actionPrepState.setTrue(),
                                        Commands.waitSeconds(0.9),
                                        RobotStates.actionPrepState.setFalse(),
                                        Commands.waitSeconds(0.5),
                                        RobotStates.homeAll.toggleToTrue(),
                                        Commands.waitSeconds(0.5)));
    }

    public Command autonCoralL4Stage() {
        return Commands.sequence(RobotStates.coral.setTrue(), RobotStates.l4.setTrue());
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

    public Command SpectrumAuton(String autoName, boolean mirrored, double duration) {
        Command autoCommand = new PathPlannerAuto(autoName, mirrored);
        return Commands.waitSeconds(0.01)
                .andThen(autoCommand)
                .withTimeout(duration)
                .withName(autoName);
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
