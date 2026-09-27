package frc.spectrumLib.hardware;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.spectrumLib.telemetry.Telemetry;
import java.util.HashMap;
import java.util.Map;

/**
 * Identifies the specific RoboRIO that is running, keyed by its serial number. Used to select
 * robot-specific configurations at startup.
 *
 * <p>Serial numbers are printed on the label on the back of the RoboRIO — prefix with a leading
 * zero if needed. Keep entries in lexical order. Note that the serial number may change after
 * reflashing the RoboRIO.
 *
 * <p>Based on:
 * https://github.com/Team100/all24/blob/2a109b28467cfddcafb93c7fc85ef60b56a628a2/lib/src/main/java/org/team100/lib/config/Identity.java
 */
public enum Rio {

    // 2026 Robots
    PHOTON2026("032B4BB3", true),
    PM_2026("0329AD07", true),
    FM_2026("", true),
    OM_2026("", true),

    // 2025 Robots
    FM_2025("0329F2D1", true),

    // 2024 Robots
    FM_2024("032B1F69", true),

    SIM("", true), // e.g. test default or simulation
    UNKNOWN(null, true);

    /** Map from serial-number string to the corresponding {@link Rio} enum constant. */
    private static final Map<String, Rio> IDs = new HashMap<>();

    static {
        for (Rio i : Rio.values()) {
            IDs.put(i.serialNumber, i);
        }
    }

    private static final Alert rioIdAlert = new Alert("RIO: ", AlertType.kInfo);
    private static final Alert rioIdUnknown = new Alert("UNKNOWN RIO: ", AlertType.kError);
    private static final Alert rio1alert = new Alert("RIO 1.0", AlertType.kWarning);

    /** The {@link Rio} constant that matches the hardware running this code. */
    public static final Rio id = checkID();

    /** CANivore bus selector that chooses the first CANivore found on the system. */
    public static final String CANIVORE = "*";
    /** CAN bus name for the native RoboRIO CAN interface. */
    public static final String RIO_CANBUS = "rio";

    private final String serialNumber;
    private final boolean isRio2;
    /**
     * Creates a new Rio instance.
     *
     * @param serialNumber the serialNumber
     * @param isRio2 the isRio2
     */
    private Rio(String serialNumber, boolean isRio2) {
        this.serialNumber = serialNumber;
        this.isRio2 = isRio2;
    }
    /** Checks the id. */
    private static Rio checkID() {
        rioIdAlert.set(false);
        rioIdUnknown.set(false);
        String serialNumber = "";
        if (RobotBase.isReal()) {
            // Calling getSerialNumber in a vscode unit test
            // SEGVs because it does the wrong
            // thing with JNIs, so don't do that.
            serialNumber = RobotController.getSerialNumber();
            Telemetry.print("RIO SERIAL: " + serialNumber);
        } else {
            serialNumber = "";
        }

        if (IDs.containsKey(serialNumber)) {
            Rio id = IDs.get(serialNumber);
            rioIdAlert.setText("Rio: " + id.name());
            rioIdAlert.set(true);

            Telemetry.print("RIO NAME: " + id.name());
            if (!id.isRio2) {
                rio1alert.set(true);
            }
            return id;
        }
        rioIdUnknown.setText("Unknown Rio: " + serialNumber);
        rioIdUnknown.set(true);
        return UNKNOWN;
    }

    /**
     * Returns {@code true} if this RoboRIO is a second-generation (RIO 2.0) controller.
     *
     * @return {@code true} for RIO 2.0, {@code false} for RIO 1.0
     */
    public boolean isRio2() {
        return isRio2;
    }
}
