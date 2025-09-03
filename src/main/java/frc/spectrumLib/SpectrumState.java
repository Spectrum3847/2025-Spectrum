package frc.spectrumLib;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SpectrumState extends Trigger {

    private static final HashMap<String, Boolean> stateConditions = new HashMap<>();
    private String name;
    private boolean value = false;
    private Alert alert;

    /**
     * Create a new EventTrigger. This will run on the EventScheduler's event loop, which will be
     * polled any time a path following command is running.
     *
     * @param name The name of the event. This will be the name of the event marker in the GUI
     */
    public SpectrumState(String name) {
        super(pollCondition(name));
        this.name = name;
        alert = new Alert("States", name, AlertType.kInfo);
    }

    /**
     * Create a new EventTrigger that gets polled by the given event loop instead of the
     * EventScheduler
     *
     * @param eventLoop The event loop to poll this trigger
     * @param name The name of the event. This will be the name of the event marker in the GUI
     */
    public SpectrumState(EventLoop eventLoop, String name) {
        super(eventLoop, pollCondition(name));
    }

    /**
     * Directly set the spectrum state to the specified value
     *
     * @param value The value to set the state to
     */
    public void setState(boolean value) {
        this.value = value;
        alert.set(value);
        setCondition(name, value);
    }

    /**
     * Create a command that will set the state to true while the command is running Then it will
     * set to false once it is cancelled
     *
     * @return the command
     */
    public Command setTrueWhileRunning() {
        return Commands.startEnd(() -> setState(true), () -> setState(false))
                .ignoringDisable(true)
                .withName(name + " state: TrueWhileRunning");
    }

    public Command setTrueForTime(DoubleSupplier time) {
        return Commands.runOnce(() -> setState(true))
                .alongWith(new WaitCommand(time.getAsDouble()))
                .andThen(() -> setState(false))
                .ignoringDisable(true)
                .withName(name + " state: SetTrueForTime->" + time.getAsDouble());
    }

    public Command setFalseForTime(DoubleSupplier time) {
        return Commands.runOnce(() -> setState(false))
                .alongWith(new WaitCommand(time.getAsDouble()))
                .andThen(() -> setState(true))
                .ignoringDisable(false)
                .withName(name + " state: SetFalseForTime->" + time.getAsDouble());
    }

    public Command setTrueForTimeWithCancel(DoubleSupplier time, Trigger cancelCondition) {
        return Commands.runOnce(() -> setState(true))
                .alongWith(new WaitCommand(time.getAsDouble()).onlyWhile(cancelCondition.not()))
                .andThen(
                        () -> {
                            setState(false);
                        })
                .ignoringDisable(true)
                .withName(name + " state: SetTrueForTimeWithCancel->" + time.getAsDouble());
    }

    /**
     * Command to set state to false, and then to true, ensuring your state will trigger actions
     *
     * @return the command
     */
    public Command toggleToTrue() {
        return setFalse()
                .andThen(new WaitCommand(0.005), setTrue())
                .finallyDo(() -> setState(true))
                .ignoringDisable(true)
                .withName(name + " state: ToggleToTrue");
    }

    /**
     * Command to set state to true, and then to false, ensuring your state will trigger change to
     * false actions
     *
     * @return
     */
    public Command toggleToFalse() {
        return setTrue()
                .andThen(new WaitCommand(0.005), setFalse())
                .finallyDo(() -> setState(false))
                .ignoringDisable(true)
                .withName(name + " state: ToggleToFalse");
    }

    /**
     * @param value
     * @return
     */
    public Command set(boolean value) {
        return Commands.runOnce(() -> setState(value)).ignoringDisable(true);
    }

    public Command setTrue() {
        return set(true).withName(name + " state: SetTrue");
    }

    public Command setFalse() {
        return set(false).withName(name + " state: SetFalse");
    }

    public Command toggle() {
        return Commands.runOnce(
                        () -> {
                            value = !value;
                            alert.set(value);
                            setCondition(name, value);
                        })
                .ignoringDisable(true)
                .withName(name + " state: Toggle");
    }

    /**
     * Create a boolean supplier that will poll(check) a condition.
     *
     * @param name The name of the event
     * @return A boolean supplier to poll the event's condition
     */
    private static BooleanSupplier pollCondition(String name) {
        // Ensure there is a condition in the map for this name
        if (!stateConditions.containsKey(name)) {
            stateConditions.put(name, false);
        }

        return () -> stateConditions.get(name);
    }

    /**
     * Set the value of an event condition
     *
     * @param name The name of the condition
     * @param value The value of the condition
     */
    protected static void setCondition(String name, boolean value) {
        stateConditions.put(name, value);
    }
}
