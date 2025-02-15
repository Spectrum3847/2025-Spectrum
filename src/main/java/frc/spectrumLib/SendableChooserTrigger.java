package frc.spectrumLib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;

/**
 * A chooser for selecting between multiple {@link Trigger} objects dynamically using
 * SmartDashboard.
 */
public class SendableChooserTrigger implements Sendable {
    private final Map<String, BooleanSupplier> m_map = new LinkedHashMap<>();
    private String m_defaultChoice = "";
    private String m_selected = null;
    private String m_previousVal;
    private static final AtomicInteger s_instances = new AtomicInteger();
    private final int m_instance;
    private final ReentrantLock m_mutex = new ReentrantLock();

    /** Creates a new SendableChooserTrigger. */
    public SendableChooserTrigger() {
        m_instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableChooser", m_instance);
    }

    /**
     * Adds a Trigger option to the chooser.
     *
     * @param name The name displayed on SmartDashboard.
     * @param trigger The {@link Trigger} associated with this option.
     */
    public void addOption(String name, Trigger trigger) {
        m_map.put(name, trigger::getAsBoolean);
    }

    /**
     * Sets a default Trigger option.
     *
     * @param name The default option name.
     * @param trigger The default {@link Trigger}.
     */
    public void setDefaultOption(String name, Trigger trigger) {
        m_defaultChoice = name;
        addOption(name, trigger);
    }

    /**
     * Gets whether the selected Trigger is active.
     *
     * @return True if the selected Trigger is active, otherwise false.
     */
    public boolean getSelectedTriggerState() {
        m_mutex.lock();
        try {
            BooleanSupplier selectedTrigger =
                    m_map.getOrDefault(m_selected, m_map.get(m_defaultChoice));
            return selectedTrigger != null && selectedTrigger.getAsBoolean();
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Converts the selected Trigger back to a {@link Trigger} object.
     *
     * @return The selected Trigger as a {@link Trigger} object.
     */
    public Trigger getSelectedTrigger() {
        return new Trigger(this::getSelectedTriggerState);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.addStringProperty("default", () -> m_defaultChoice, null);
        builder.addStringArrayProperty(
                "options", () -> m_map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(
                "selected",
                null,
                val -> {
                    m_mutex.lock();
                    try {
                        m_selected = val;
                    } finally {
                        m_mutex.unlock();
                    }
                });
    }
}
