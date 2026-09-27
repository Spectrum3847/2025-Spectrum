package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.DoubleSupplier;

/** From 254 lib imported from 1678-2024 Contains basic functions that are used often. */
public class Util {

    /** Small value used for floating-point equality comparisons. */
    public static final double EPSILON = 1e-12;

    /** Prevent this class from being instantiated. */
    private Util() {}

    /**
     * Clamps {@code v} to the range [{@code -maxMagnitude}, {@code maxMagnitude}].
     *
     * @param v the value to clamp
     * @param maxMagnitude the maximum absolute value allowed
     * @return the clamped value
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Clamps {@code v} to the range [{@code min}, {@code max}].
     *
     * @param v the value to clamp
     * @param min the lower bound (inclusive)
     * @param max the upper bound (inclusive)
     * @return the clamped value
     */
    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    /**
     * Checks whether {@code v} is strictly within [{@code -maxMagnitude}, {@code maxMagnitude}].
     *
     * @param v the value to test
     * @param maxMagnitude the maximum absolute value (exclusive bound)
     * @return {@code true} if {@code |v| < maxMagnitude}
     */
    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /** Checks if the given input is within the range (min, max), both exclusive. */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    /**
     * Checks whether the value supplied by {@code v} is strictly between the values supplied by
     * {@code min} and {@code max}.
     *
     * @param v supplier of the value to test
     * @param min supplier of the lower bound (exclusive)
     * @param max supplier of the upper bound (exclusive)
     * @return {@code true} if {@code min.get() < v.get() < max.get()}
     */
    public static boolean inRange(DoubleSupplier v, DoubleSupplier min, DoubleSupplier max) {
        double value = v.getAsDouble();
        return value > min.getAsDouble() && value < max.getAsDouble();
    }

    /**
     * Linearly interpolates between {@code a} and {@code b} by a factor {@code x}, clamped to [0,
     * 1].
     *
     * @param a the start value ({@code x = 0})
     * @param b the end value ({@code x = 1})
     * @param x the interpolation factor, clamped to [0, 1]
     * @return the interpolated value
     */
    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    /**
     * Joins a list of objects into a single string with the given delimiter.
     *
     * @param delim the delimiter placed between consecutive elements
     * @param strings the list of objects whose {@code toString()} values are joined
     * @return the joined string
     */
    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    /**
     * Checks whether {@code a} and {@code b} are within {@code epsilon} of each other.
     *
     * @param a first value
     * @param b second value
     * @param epsilon the allowed absolute difference
     * @return {@code true} if {@code |a - b| <= epsilon}
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * Checks whether {@code a} and {@code b} are within {@link #EPSILON} of each other.
     *
     * @param a first value
     * @param b second value
     * @return {@code true} if {@code |a - b| <= EPSILON}
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    /**
     * Checks whether integer {@code a} and {@code b} are within {@code epsilon} of each other.
     *
     * @param a first integer value
     * @param b second integer value
     * @param epsilon the allowed absolute difference
     * @return {@code true} if {@code |a - b| <= epsilon}
     */
    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return ((long) a - epsilon <= b) && ((long) a + epsilon >= b);
    }

    /**
     * Checks whether every element in {@code list} is within {@code epsilon} of {@code value}.
     *
     * @param list the list of doubles to check
     * @param value the target value each element is compared against
     * @param epsilon the allowed absolute difference for each comparison
     * @return {@code true} if all elements are within {@code epsilon} of {@code value}
     */
    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    /** Trigger that is true when the robot is enabled in teleop mode. */
    public static final Trigger teleop = RobotModeTriggers.teleop();

    /** Trigger that is true when the robot is enabled in autonomous mode. */
    public static final Trigger autoMode = RobotModeTriggers.autonomous();

    /** Trigger that is true when the robot is enabled in test mode. */
    public static final Trigger testMode = RobotModeTriggers.test();

    /** Trigger that is true when the robot is disabled. */
    public static final Trigger disabled = RobotModeTriggers.disabled();

    /** Trigger that is true when the DriverStation is attached. */
    public static final Trigger dsAttached = new Trigger(DriverStation::isDSAttached);
}
