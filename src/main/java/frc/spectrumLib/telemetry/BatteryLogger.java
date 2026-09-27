// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.spectrumLib.telemetry;

import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

/**
 * Tracks and logs current draw, power, and cumulative energy consumption across subsystems each
 * robot loop cycle.
 *
 * <p>Key derivation (splitting a channel name into its parent keys) and log-key construction are
 * cached per channel name, so steady-state operation performs no string splitting or concatenation.
 * The set of channel names is small and fixed after the first loop.
 *
 * <p>Current is published at 10 Hz ({@link Telemetry#slowLogThisLoop()}) and power and energy at
 * {@link #SLOW_LOG_PERIOD_SECONDS}, because this class was the single largest producer of log
 * records on the robot and the loop period tracks that volume almost linearly. In the 2026-09-05
 * 18:38 log it accounted for 32.3 of the 94 records written per loop -- a third of everything --
 * while the loop ran at 33 Hz with a median period of 27.4 ms. Current is the channel worth
 * watching live; power is current times pack voltage and energy is its integral, so neither needs
 * 50 Hz to stay useful.
 */
public class BatteryLogger {

    /**
     * Seconds between power and energy publications.
     *
     * <p>At a 20 ms loop this drops their combined ~24 records per loop to about one, without
     * removing either channel from the log.
     */
    private static final double SLOW_LOG_PERIOD_SECONDS = 0.5;

    /**
     * Ceiling on one integration step, in seconds.
     *
     * <p>Bounds what a stall or a pause between enables can bank into the energy total. Normal
     * loops are far below it; anything above it is not time the mechanisms actually spent drawing
     * the current we just sampled.
     */
    private static final double MAX_INTEGRATION_SECONDS = 0.25;

    /** When {@code false} all methods are no-ops, allowing the logger to be disabled at runtime. */
    @Setter private boolean enabled = false;

    /**
     * Running total of current draw accumulated since the last {@link #logPower()} call, in amps.
     */
    @Getter private double totalCurrent = 0.0;

    /** Total power drawn on the most recent {@link #logPower()} call, in watts. */
    @Getter private double totalPower = 0.0;

    /** Cumulative energy consumed over the entire enabled session, in joules. */
    @Getter private double totalEnergy = 0.0;

    /** Battery terminal voltage used to convert current to power, in volts. */
    @Setter private double batteryVoltage = 12.6;

    /** Estimated current drawn by the RoboRIO itself, in amps. */
    @Setter private double rioCurrent = 0.0;

    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    private final Map<String, Double> subsystemEnergies = new HashMap<>();

    /** Channel name to its parent keys, derived once per channel by {@link #deriveParentKeys}. */
    private final Map<String, String[]> parentKeys = new HashMap<>();

    /** Channel or parent name to its fully qualified log key, built once per name. */
    private final Map<String, String> currentLogKeys = new HashMap<>();

    private final Map<String, String> powerLogKeys = new HashMap<>();
    private final Map<String, String> energyLogKeys = new HashMap<>();

    /** FPGA time of the previous {@link #logPower()} call, or NaN before the first one. */
    private double lastIntegrationSeconds = Double.NaN;

    /** FPGA time of the previous power/energy publication, or NaN before the first one. */
    private double lastSlowLogSeconds = Double.NaN;

    /**
     * Splits a channel name on "/" or "-" and returns every parent prefix joined with "/", from the
     * root down. {@code "A/B/C"} yields {@code ["A", "A/B"]}; a name with no separator yields an
     * empty array.
     */
    private static String[] deriveParentKeys(String key) {
        String[] keys = key.split("/|-");
        if (keys.length < 2) {
            return new String[0];
        }
        String[] parents = new String[keys.length - 1];
        StringBuilder subkey = new StringBuilder();
        for (int i = 0; i < keys.length - 1; i++) {
            if (i > 0) {
                subkey.append('/');
            }
            subkey.append(keys[i]);
            parents[i] = subkey.toString();
        }
        return parents;
    }

    /**
     * Records the current draw for a named subsystem channel and accumulates it into the running
     * totals. The {@code key} may use "/" or "-" as separators; parent keys are automatically
     * aggregated.
     *
     * <p>Power and energy are no longer derived here. Both are computed from these currents in
     * {@link #logPower()}, which knows this loop's pack voltage and its true elapsed time.
     *
     * @param key Hierarchical name for the current consumer (e.g. {@code "Drive/FrontLeft"})
     * @param amps One or more current readings in amps; absolute values are summed
     */
    public void reportCurrentUsage(String key, double... amps) {
        if (enabled) {
            double totalAmps = 0.0;
            for (double amp : amps) totalAmps += Math.abs(amp);

            totalCurrent += totalAmps;
            subsystemCurrents.put(key, totalAmps);

            for (String parent : parentKeys.computeIfAbsent(key, BatteryLogger::deriveParentKeys)) {
                subsystemCurrents.merge(parent, totalAmps, Double::sum);
            }
        }
    }

    /**
     * Appends control-overhead current consumers (roboRIO, CANcoders, Pigeon, CANivore, radio),
     * integrates energy over the elapsed loop, then logs current every loop and power and energy
     * every {@link #SLOW_LOG_PERIOD_SECONDS}. Resets the per-loop current accumulators afterward;
     * cumulative energy is preserved across calls.
     */
    public void logPower() {
        if (enabled) {
            // Controls overhead is added here so it is included in the totalCurrent log below.
            // Subsystem currents have already been accumulated via logBatteryUsage() in periodic().
            reportCurrentUsage("Controls/roboRIO", rioCurrent);
            reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
            reportCurrentUsage("Controls/Pigeon", 0.04);
            reportCurrentUsage("Controls/CANivore", 0.03);
            reportCurrentUsage("Controls/Radio", 0.5);

            double now = Timer.getFPGATimestamp();

            /*
             * Integrate over the loop that actually elapsed rather than an assumed 20 ms. The
             * assumption was costing us: the 2026-09-05 logs ran a 20.2 to 27.4 ms median period,
             * so every energy total that day was under-reported by roughly a fifth to a third.
             */
            double elapsed =
                    Double.isNaN(lastIntegrationSeconds)
                            ? 0.0
                            : Math.min(
                                    Math.max(now - lastIntegrationSeconds, 0.0),
                                    MAX_INTEGRATION_SECONDS);
            lastIntegrationSeconds = now;

            totalPower = totalCurrent * batteryVoltage;
            totalEnergy += totalPower * elapsed;
            for (var entry : subsystemCurrents.entrySet()) {
                subsystemEnergies.merge(
                        entry.getKey(), entry.getValue() * batteryVoltage * elapsed, Double::sum);
            }

            boolean logSlow =
                    Double.isNaN(lastSlowLogSeconds)
                            || now - lastSlowLogSeconds >= SLOW_LOG_PERIOD_SECONDS;
            if (logSlow) {
                lastSlowLogSeconds = now;
            }

            // Currents at 10 Hz; total current, pack voltage, energy and the per-mechanism currents
            // are on the Driver Station dashboard and in the robot app, hence logDash.
            boolean logCurrent = Telemetry.slowLogThisLoop();
            if (logCurrent) {
                Telemetry.logDash("BatteryLogger/Current", totalCurrent, "amps");
                Telemetry.logDash("BatteryLogger/BatteryVoltage", batteryVoltage, "volts");
            }
            if (logSlow) {
                Telemetry.log("BatteryLogger/Power", totalPower, "watts");
                Telemetry.logDashAlways(
                        "BatteryLogger/Energy", joulesToWattHours(totalEnergy), "wh");
            }

            for (var entry : subsystemCurrents.entrySet()) {
                String key = entry.getKey();
                double amps = entry.getValue();
                if (logCurrent) {
                    Telemetry.logDash(
                            currentLogKeys.computeIfAbsent(key, k -> "BatteryLogger/Current/" + k),
                            amps,
                            "amps");
                }
                if (logSlow) {
                    Telemetry.log(
                            powerLogKeys.computeIfAbsent(key, k -> "BatteryLogger/Power/" + k),
                            amps * batteryVoltage,
                            "watts");
                }
                entry.setValue(0.0);
            }
            if (logSlow) {
                for (var entry : subsystemEnergies.entrySet()) {
                    Telemetry.log(
                            energyLogKeys.computeIfAbsent(
                                    entry.getKey(), k -> "BatteryLogger/Energy/" + k),
                            joulesToWattHours(entry.getValue()),
                            "wh");
                }
            }

            // Reset the current total, before next loop
            totalCurrent = 0.0;
        }
    }

    /**
     * Current accumulated this loop for a channel or parent key, in amps. Package-private for
     * tests.
     */
    double getSubsystemCurrent(String key) {
        return subsystemCurrents.getOrDefault(key, 0.0);
    }

    /** Joules to watt hours. */
    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}
