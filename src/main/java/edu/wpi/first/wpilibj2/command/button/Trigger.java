// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command.button;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;

/**
 * This class provides an easy way to link commands to conditions.
 *
 * <p>It is very easy to link a button to a command. For instance, you could link the trigger button
 * of a joystick to a "score" command.
 *
 * <p>Triggers can easily be composed for advanced functionality using the {@link
 * #and(BooleanSupplier)}, {@link #or(BooleanSupplier)}, {@link #negate()} operators.
 *
 * <p>This class is provided by the NewCommands VendorDep
 *
 * <p>Spectrum modified in Fall 2024 to allow triggers to default start condition of false, so if
 * something is already true when bound it will activate the trigger. We needed this for a trigger
 * to activate only if Teleop was enabled.
 */
public class Trigger implements BooleanSupplier {
    private final BooleanSupplier m_condition;
    private final EventLoop m_loop;
    public static final Trigger kFalse = new Trigger(() -> false);
    public static final Trigger kTrue = new Trigger(() -> true);

    /**
     * Creates a new trigger based on the given condition.
     *
     * @param loop The loop instance that polls this trigger.
     * @param condition the condition represented by this trigger
     */
    public Trigger(EventLoop loop, BooleanSupplier condition) {
        m_loop = requireNonNullParam(loop, "loop", "Trigger");
        m_condition = requireNonNullParam(condition, "condition", "Trigger");
    }

    /**
     * Creates a new trigger based on the given condition.
     *
     * <p>Polled by the default scheduler button loop.
     *
     * @param condition the condition represented by this trigger
     */
    public Trigger(BooleanSupplier condition) {
        this(CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
    }

    /**
     * Starts the command when the condition changes.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onChange(Command command) {
        requireNonNullParam(command, "command", "onChange");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast != pressed) {
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command whenever the condition changes from `false` to `true`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onTrue(Command command) {
        requireNonNullParam(command, "command", "onTrue");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = false;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given commands whenever the condition changes from `false` to `true`.
     *
     * @param commands the commands to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onTrue(Command... commands) {
        requireNonNullParam(commands, "command", "onTrue");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = false;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            for (Command command : commands) {
                                command.schedule();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command whenever the condition changes from `true` to `false`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onFalse(Command command) {
        requireNonNullParam(command, "command", "onFalse");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = true;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    public Trigger onFalse(Command... commands) {
        requireNonNullParam(commands, "command", "onFalse");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = true;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            for (Command command : commands) {
                                command.schedule();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command whenever the condition changes from `true` to `false`, but has to
     * have run once prior
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onChangeToFalse(Command command) {
        requireNonNullParam(command, "command", "onFalse");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = false;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command whenever the condition changes from `false` to `true`, but has to
     * have run once prior
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onChangeToTrue(Command command) {
        requireNonNullParam(command, "command", "onTrue");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = true;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition changes to `true` and cancels it when the
     * condition changes to `false`.
     *
     * <p>Doesn't re-start the command if it ends while the condition is still `true`. If the
     * command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whileTrue(Command command) {
        requireNonNullParam(command, "command", "whileTrue");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = false;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            command.schedule();
                        } else if (m_pressedLast && !pressed) {
                            command.cancel();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition changes to `true` and cancels it when the
     * condition changes to `false`.
     *
     * <p>Doesn't re-start the command if it ends while the condition is still `true`. If the
     * command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param commands the commands to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whileTrue(Command... commands) {
        requireNonNullParam(commands, "command", "whileTrue");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = false;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        for (Command command : commands) {
                            if (!m_pressedLast && pressed) {
                                command.schedule();
                            } else if (m_pressedLast && !pressed) {
                                command.cancel();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition changes to `false` and cancels it when the
     * condition changes to `true`.
     *
     * <p>Doesn't re-start the command if it ends while the condition is still `false`. If the
     * command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whileFalse(Command command) {
        requireNonNullParam(command, "command", "whileFalse");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = true;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            command.schedule();
                        } else if (!m_pressedLast && pressed) {
                            command.cancel();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    public Trigger whileFalse(Command... commands) {
        requireNonNullParam(commands, "command", "whileFalse");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = true;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        for (Command command : commands) {
                            if (m_pressedLast && !pressed) {
                                command.schedule();
                            } else if (!m_pressedLast && pressed) {
                                command.cancel();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Toggles a command when the condition changes from `false` to `true`.
     *
     * @param command the command to toggle
     * @return this trigger, so calls can be chained
     */
    public Trigger toggleOnTrue(Command command) {
        requireNonNullParam(command, "command", "toggleOnTrue");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = false;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            if (command.isScheduled()) {
                                command.cancel();
                            } else {
                                command.schedule();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Toggles a command when the condition changes from `true` to `false`.
     *
     * @param command the command to toggle
     * @return this trigger, so calls can be chained
     */
    public Trigger toggleOnFalse(Command command) {
        requireNonNullParam(command, "command", "toggleOnFalse");
        m_loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = true;

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            if (command.isScheduled()) {
                                command.cancel();
                            } else {
                                command.schedule();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Run a command while true. Also runs a command for a certain timeout when released.
     *
     * @param runCommand the command to run while true
     * @param endCommand the command to run when released
     * @param endTimeout the time to run the end command
     * @return this trigger, so calls can be chained
     */
    public Trigger runWithEndSequence(Command runCommand, Command endCommand, double endTimeout) {
        this.whileTrue(runCommand);
        this.onFalse(endCommand.withTimeout(endTimeout).withName(endCommand.getName()));
        return this;
    }

    @Override
    public boolean getAsBoolean() {
        return m_condition.getAsBoolean();
    }

    /**
     * Composes two triggers with logical AND.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when both component triggers are active.
     */
    public Trigger and(BooleanSupplier trigger) {
        return new Trigger(m_loop, () -> m_condition.getAsBoolean() && trigger.getAsBoolean());
    }

    /**
     * Combines multiple BooleanSupplier triggers using a logical AND operation.
     *
     * @param triggers an array of BooleanSupplier triggers to be combined.
     * @return a new Trigger that represents the logical AND of all provided triggers.
     */
    public Trigger and(BooleanSupplier... triggers) {
        Trigger trig = this;
        for (BooleanSupplier t : triggers) {
            trig = trig.and(t);
        }
        return trig;
    }

    /**
     * Combines multiple BooleanSupplier triggers using a logical OR operation.
     *
     * @param triggers an array of BooleanSupplier triggers to be combined.
     * @return a new NewTrigger instance that represents the logical OR of the provided triggers.
     */
    public Trigger or(BooleanSupplier... triggers) {
        Trigger trig = this;
        for (BooleanSupplier t : triggers) {
            trig = trig.or(t);
        }
        return trig;
    }

    /**
     * Composes two triggers with logical OR.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when either component trigger is active.
     */
    public Trigger or(BooleanSupplier trigger) {
        return new Trigger(m_loop, () -> m_condition.getAsBoolean() || trigger.getAsBoolean());
    }

    /**
     * Creates a new trigger that is active when this trigger is inactive, i.e. that acts as the
     * negation of this trigger.
     *
     * @return the negated trigger
     */
    public Trigger negate() {
        return new Trigger(m_loop, () -> !m_condition.getAsBoolean());
    }

    /**
     * renamed negate
     *
     * @return the negated trigger
     */
    public Trigger not() {
        return negate();
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active when this trigger
     * has been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @return The debounced trigger (rising edges debounced only)
     */
    public Trigger debounce(double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active when this trigger
     * has been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @param type The debounce type.
     * @return The debounced trigger.
     */
    public Trigger debounce(double seconds, Debouncer.DebounceType type) {
        return new Trigger(
                m_loop,
                new BooleanSupplier() {
                    final Debouncer m_debouncer = new Debouncer(seconds, type);

                    @Override
                    public boolean getAsBoolean() {
                        return m_debouncer.calculate(m_condition.getAsBoolean());
                    }
                });
    }
}
