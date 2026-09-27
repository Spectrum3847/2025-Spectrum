package frc.spectrumLib.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;

/**
 * Encapsulates the three SysId characterization routines for a CTRE swerve drivetrain: translation,
 * rotation, and steer-gains.
 *
 * <p>Construct one instance per robot, passing the {@link Swerve} subsystem. Then bind {@link
 * #sysIdQuasistatic} and {@link #sysIdDynamic} to test-mode triggers. Change {@link
 * #RoutineToApply} (by editing the source) to select which routine runs.
 */
public class SysID {
    // private Swerve swerve;

    /** SysId routine that characterizes linear translation drive gains. */
    @Getter private final SysIdRoutine SysIdRoutineTranslation;

    /** SysId routine that characterizes rotational drive gains. */
    @Getter private final SysIdRoutine SysIdRoutineRotation;

    /** SysId routine that characterizes steer-module gains. */
    @Getter private final SysIdRoutine SysIdRoutineSteer;

    /** The routine currently selected for quasistatic and dynamic test commands. */
    private final SysIdRoutine RoutineToApply;

    /** CTRE swerve request used during the translation characterization routine. */
    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();

    /** CTRE swerve request used during the rotation characterization routine. */
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    /** CTRE swerve request used during the steer-gains characterization routine. */
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();

    /**
     * Constructs all three SysId routines and selects {@link #SysIdRoutineTranslation} as the
     * active routine. Change {@link #RoutineToApply} at the bottom of this constructor to switch
     * which routine is exercised by {@link #sysIdQuasistatic} and {@link #sysIdDynamic}.
     *
     * @param swerve the {@link Swerve} subsystem that will be commanded during characterization
     */
    public SysID(Swerve swerve) {
        // this.swerve = swerve;

        /* Use one of these sysid routines for your particular test */
        String stateTxt = "state";
        SysIdRoutineTranslation =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(4),
                                null,
                                state -> SignalLogger.writeString(stateTxt, state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts ->
                                        swerve.setControl(
                                                TranslationCharacterization.withVolts(volts)),
                                null,
                                swerve));

        SysIdRoutineRotation =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(4),
                                null,
                                state -> SignalLogger.writeString(stateTxt, state.toString())),
                        new SysIdRoutine.Mechanism(
                                roationalRate ->
                                        swerve.setControl(
                                                RotationCharacterization.withRotationalRate(
                                                        roationalRate.baseUnitMagnitude())),
                                // it actual
                                // rotational rate
                                null,
                                swerve));

        SysIdRoutineSteer =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(7),
                                null,
                                state -> SignalLogger.writeString(stateTxt, state.toString())),
                        new SysIdRoutine.Mechanism(
                                volts -> swerve.setControl(SteerCharacterization.withVolts(volts)),
                                null,
                                swerve));

        /* Change this to the sysid routine you want to test */
        RoutineToApply = SysIdRoutineTranslation;
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */

    /**
     * Returns a quasistatic (slow ramp) characterization command for the active routine.
     *
     * @param direction the direction ({@link SysIdRoutine.Direction#kForward} or {@link
     *     SysIdRoutine.Direction#kReverse}) in which to ramp the output
     * @return a command that slowly ramps the output and logs data for system identification
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    /**
     * Returns a dynamic (step) characterization command for the active routine.
     *
     * @param direction the direction in which to apply the voltage step
     * @return a command that applies a step voltage and logs data for system identification
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }
}
