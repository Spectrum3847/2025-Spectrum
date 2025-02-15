package frc.spectrumLib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A chooser for selecting between multiple {@link Trigger} objects dynamically using
 * SmartDashboard.
 */
public class SendableChooserTrigger2 implements Sendable {
    private final SendableChooser<SpectrumState> stateChooser;
    private SpectrumState previousState;

    /** Creates a new SendableChooserTrigger. */
    public SendableChooserTrigger2() {
        stateChooser = new SendableChooser();
    }

    /**
     * Adds a Trigger option to the chooser.
     *
     * @param name The name displayed on SmartDashboard.
     * @param trigger The {@link Trigger} associated with this option.
     */
    public void addOption(String name, Trigger trigger) {
        SpectrumState newSpectrumState = new SpectrumState(name);
        stateChooser.addOption(name, newSpectrumState);
        trigger = trigger.or(newSpectrumState);
    }

    /**
     * Sets a default Trigger option.
     *
     * @param name The default option name.
     * @param trigger The default {@link Trigger}.
     */
    public void setDefaultOption(String name, Trigger trigger) {
        SpectrumState newSpectrumState = new SpectrumState(name);
        stateChooser.setDefaultOption(name, newSpectrumState);
        trigger = trigger.or(newSpectrumState);
    }

    /**
     * Gets whether the selected Trigger is active.
     *
     * @return True if the selected Trigger is active, otherwise false.
     */
    public void getSelectedTriggerState() {
        if (previousState != null) {
            previousState.setFalse();
        }
        SpectrumState selectedState = stateChooser.getSelected();
        if (selectedState != null) {
            selectedState.setTrue();
            previousState = selectedState;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        stateChooser.initSendable(builder);
    }
}
