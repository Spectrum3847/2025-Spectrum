package frc.spectrumLib.util.exceptions;

/**
 * Unchecked exception thrown to signal that the robot code should stop executing immediately.
 * Intended for unrecoverable error states where continued operation would be unsafe.
 */
public class KillRobotException extends RuntimeException {

    /**
     * Creates a KillRobotException with a descriptive message.
     *
     * @param message explanation of the condition that triggered the kill
     */
    public KillRobotException(String message) {
        // calling super invokes the constructors of all super classes
        // which helps to create the complete stacktrace.
        super(message);
    }

    /**
     * Creates a KillRobotException wrapping an underlying cause.
     *
     * @param cause the original exception that led to this kill condition
     */
    public KillRobotException(Throwable cause) {
        // call appropriate parent constructor
        super(cause);
    }

    /**
     * Creates a KillRobotException with both a message and an underlying cause.
     *
     * @param message explanation of the condition that triggered the kill
     * @param throwable the original exception that led to this kill condition
     */
    public KillRobotException(String message, Throwable throwable) {
        // call appropriate parent constructor
        super(message, throwable);
    }
}
