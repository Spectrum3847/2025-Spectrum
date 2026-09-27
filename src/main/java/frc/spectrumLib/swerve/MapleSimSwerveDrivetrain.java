package frc.spectrumLib.swerve;

// Copyright 2021-2025 Iron Maple 5516
// Original Source:
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/CTRE%20Swerve%20with%20maple-sim/src/main/java/frc/robot/utils/simulation/MapleSimSwerveDrivetrain.java
//
// This code is licensed under MIT license (see https://mit-license.org/)

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 *
 *
 * <h2>Injects Maple-Sim simulation data into a CTRE swerve drivetrain.</h2>
 *
 * <p>This class retrieves simulation data from Maple-Sim and injects it into the CTRE {@link
 * com.ctre.phoenix6.swerve.SwerveDrivetrain} instance.
 *
 * <p>It replaces the {@link com.ctre.phoenix6.swerve.SimSwerveDrivetrain} class.
 */
public class MapleSimSwerveDrivetrain {
    /** Simulation state of the Pigeon2 gyro, used to inject computed yaw and angular velocity. */
    private final Pigeon2SimState pigeonSim;

    /** Simulated representations of each swerve module, indexed FL/FR/BL/BR. */
    private final SimSwerveModule[] simModules;

    /** The underlying Maple-Sim drive simulation providing physics and odometry. */
    public final SwerveDriveSimulation mapleSimDrive;

    /**
     *
     *
     * <h2>Constructs a drivetrain simulation using the specified parameters.</h2>
     *
     * @param simPeriod the time period of the simulation
     * @param robotMassWithBumpers the total mass of the robot, including bumpers
     * @param bumperLengthX the length of the bumper along the X-axis (influences the collision
     *     space of the robot)
     * @param bumperWidthY the width of the bumper along the Y-axis (influences the collision space
     *     of the robot)
     * @param driveMotorModel the {@link DCMotor} model for the drive motor, typically <code>
     *     DCMotor.getKrakenX60Foc()
     *     </code>
     * @param steerMotorModel the {@link DCMotor} model for the steer motor, typically <code>
     *     DCMotor.getKrakenX60Foc()
     *     </code>
     * @param wheelCOF the coefficient of friction of the drive wheels
     * @param moduleLocations the locations of the swerve modules on the robot, in the order <code>
     *     FL, FR, BL, BR</code>
     * @param pigeon the {@link Pigeon2} IMU used in the drivetrain
     * @param modules the {@link SwerveModule}s, typically obtained via {@link
     *     SwerveDrivetrain#getModules()}
     * @param moduleConstants the constants for the swerve modules
     */
    @SuppressWarnings("unchecked")
    public MapleSimSwerveDrivetrain(
            Time simPeriod,
            Mass robotMassWithBumpers,
            Distance bumperLengthX,
            Distance bumperWidthY,
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double wheelCOF,
            Translation2d[] moduleLocations,
            Pigeon2 pigeon,
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                            ...
                    moduleConstants) {
        this.pigeonSim = pigeon.getSimState();
        simModules = new SimSwerveModule[moduleConstants.length];
        DriveTrainSimulationConfig simulationConfig =
                DriveTrainSimulationConfig.Default()
                        .withRobotMass(robotMassWithBumpers)
                        .withBumperSize(bumperLengthX, bumperWidthY)
                        .withGyro(COTS.ofPigeon2())
                        .withCustomModuleTranslations(moduleLocations)
                        .withSwerveModule(
                                new SwerveModuleSimulationConfig(
                                        driveMotorModel,
                                        steerMotorModel,
                                        moduleConstants[0].DriveMotorGearRatio,
                                        moduleConstants[0].SteerMotorGearRatio,
                                        Volts.of(moduleConstants[0].DriveFrictionVoltage),
                                        Volts.of(moduleConstants[0].SteerFrictionVoltage),
                                        Meters.of(moduleConstants[0].WheelRadius),
                                        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                                        wheelCOF));
        mapleSimDrive = new SwerveDriveSimulation(simulationConfig, Pose2d.kZero);

        SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
        for (int i = 0; i < this.simModules.length; i++)
            simModules[i] =
                    new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);

        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        arena.setEfficiencyMode(true);

        SimulatedArena.overrideSimulationTimings(simPeriod, 1);
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    /**
     *
     *
     * <h2>Update the simulation.</h2>
     *
     * <p>Updates the Maple-Sim simulation and injects the results into the simulated CTRE devices,
     * including motors and the IMU.
     */
    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
        pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        pigeonSim.setAngularVelocityZ(
                RadiansPerSecond.of(
                        mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                                .omegaRadiansPerSecond));
    }

    /**
     *
     *
     * <h1>Represents the simulation of a single {@link SwerveModule}.</h1>
     */
    protected static class SimSwerveModule {
        /** Constants (gear ratios, friction voltages, wheel radius, etc.) for this module. */
        public final SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                moduleConstant;

        /** Maple-Sim physics simulation instance for this module. */
        public final SwerveModuleSimulation moduleSimulation;

        /**
         * Constructs a simulated swerve module by wiring the Maple-Sim physics model to the CTRE
         * motor controllers.
         *
         * @param moduleConstant constants for this swerve module
         * @param moduleSimulation Maple-Sim simulation instance for this module
         * @param module the real CTRE {@link SwerveModule} whose sim states will be driven
         */
        public SimSwerveModule(
                SwerveModuleConstants<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                        moduleConstant,
                SwerveModuleSimulation moduleSimulation,
                SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.moduleConstant = moduleConstant;
            this.moduleSimulation = moduleSimulation;
            moduleSimulation.useDriveMotorController(
                    new TalonFXMotorControllerSim(module.getDriveMotor()));
            moduleSimulation.useSteerMotorController(
                    new TalonFXMotorControllerWithRemoteCanCoderSim(
                            module.getSteerMotor(), module.getEncoder()));
        }
    }

    // Static utils classes

    /**
     * Adapts a {@link TalonFX} motor controller for use as a {@link SimulatedMotorController} in
     * Maple-Sim by forwarding encoder state from the simulation into the CTRE sim state.
     */
    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        /** CAN device ID of the underlying TalonFX. */
        public final int id;

        /** CTRE simulation state object used to inject position, velocity, and voltage. */
        private final TalonFXSimState talonFXSimState;

        /**
         * Constructs the adapter for the given TalonFX.
         *
         * @param talonFX the TalonFX motor controller to wrap
         */
        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.talonFXSimState = talonFX.getSimState();
        }

        /**
         * Injects simulated rotor position, velocity, and supply voltage into the TalonFX sim
         * state, then returns the motor output voltage requested by the controller's closed-loop
         * algorithm.
         *
         * @param mechanismAngle current mechanism-side angle from the physics model
         * @param mechanismVelocity current mechanism-side angular velocity from the physics model
         * @param encoderAngle current encoder angle (rotor-side) from the physics model
         * @param encoderVelocity current encoder angular velocity (rotor-side) from the physics
         *     model
         * @return the voltage the controller is requesting from the simulated battery
         */
        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    /**
     * Extends {@link TalonFXMotorControllerSim} to also drive a remote CANcoder simulation state.
     * Used for steer motors whose feedback device is a remote CANcoder.
     */
    @SuppressWarnings("all")
    public static class TalonFXMotorControllerWithRemoteCanCoderSim
            extends TalonFXMotorControllerSim {
        /** CAN device ID of the remote CANcoder. */
        private final int encoderId;

        /** CTRE simulation state of the remote CANcoder. */
        private final CANcoderSimState remoteCancoderSimState;

        /**
         * Constructs the adapter for a steer motor paired with a remote CANcoder.
         *
         * @param talonFX the steer TalonFX motor controller
         * @param cancoder the remote CANcoder used as the steer feedback sensor
         */
        public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.remoteCancoderSimState = cancoder.getSimState();

            this.encoderId = cancoder.getDeviceID();
        }

        /**
         * Injects simulated supply voltage, position, and velocity into the remote CANcoder sim
         * state, then delegates to the parent implementation to update the TalonFX and return the
         * requested motor voltage.
         *
         * @param mechanismAngle current mechanism-side angle (written to the CANcoder)
         * @param mechanismVelocity current mechanism-side angular velocity (written to the
         *     CANcoder)
         * @param encoderAngle current encoder (rotor-side) angle (forwarded to TalonFX)
         * @param encoderVelocity current encoder angular velocity (forwarded to TalonFX)
         * @return the voltage the controller is requesting from the simulated battery
         */
        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            remoteCancoderSimState.setRawPosition(mechanismAngle);
            remoteCancoderSimState.setVelocity(mechanismVelocity);

            return super.updateControlSignal(
                    mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    /**
     *
     *
     * <h2>Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.</h2>
     *
     * <p>This method processes an array of {@link SwerveModuleConstants} to apply necessary
     * adjustments for simulation purposes, ensuring compatibility and avoiding known bugs.
     *
     * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
     */
    public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
            SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
        for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants)
            regulateModuleConstantForSimulation(moduleConstant);

        return moduleConstants;
    }

    /**
     *
     *
     * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
     *
     * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for
     * simulation purposes. These changes have no effect on real robot operations and address known
     * simulation bugs:
     *
     * <ul>
     *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted
     *       configurations.
     *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization
     *       issues.
     *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve
     *       simulation performance.
     * </ul>
     *
     * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on
     * constants used on real robot hardware.</h4>
     */
    private static void regulateModuleConstantForSimulation(
            SwerveModuleConstants<?, ?, ?> moduleConstants) {
        // Skip regulation if running on a real robot
        if (RobotBase.isReal()) return;

        // Apply simulation-specific adjustments to module constants
        moduleConstants
                // Disable encoder offsets
                .withEncoderOffset(0)
                // Disable motor inversions for drive and steer motors
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                // Disable CanCoder inversion
                .withEncoderInverted(false)
                // TODO: Adjust steer and drive motor PID gains for simulation
                .withSteerMotorGains(
                        new Slot0Configs()
                                .withKP(1000.0)
                                .withKI(0)
                                .withKD(60.0)
                                .withKS(0.15)
                                .withKV(1.5)
                                .withKA(0)
                                .withStaticFeedforwardSign(
                                        StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSteerMotorGearRatio(21.428571428571427)
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.05))
                // Adjust steer inertia
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}
