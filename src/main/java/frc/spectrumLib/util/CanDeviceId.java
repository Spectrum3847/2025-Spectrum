package frc.spectrumLib.util;

// Based on 254-2023 Class
// https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java
/**
 * Identifies a CAN device by its numeric device ID and the CAN bus name it lives on. Equality and
 * hashing consider both fields, so two instances with the same device number on different buses are
 * treated as distinct.
 */
public class CanDeviceId {
    private final int mDeviceNumber;
    private final String mBus;

    /**
     * Creates a CAN device identifier with an explicit bus name.
     *
     * @param deviceNumber the numeric CAN ID assigned to the device
     * @param bus the name of the CAN bus (e.g. {@code "rio"} or {@code "canivore"})
     */
    public CanDeviceId(int deviceNumber, String bus) {
        mDeviceNumber = deviceNumber;
        mBus = bus;
    }

    // Use the default bus name (empty string).
    /**
     * Creates a CAN device identifier on the default CAN bus (empty string).
     *
     * @param deviceNumber the numeric CAN ID assigned to the device
     */
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    /**
     * Returns the numeric CAN device ID.
     *
     * @return the device number
     */
    public int getDeviceNumber() {
        return mDeviceNumber;
    }

    /**
     * Returns the CAN bus name this device is on.
     *
     * @return the bus name, or an empty string for the default bus
     */
    public String getBus() {
        return mBus;
    }

    /**
     * Type-safe equality check against another {@link CanDeviceId}.
     *
     * @param other the other instance to compare
     * @return {@code true} if both the device number and bus name match
     */
    public boolean equals(CanDeviceId other) {
        return equals((Object) other);
    }
    /** Returns a hash code for this object. */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + mDeviceNumber;
        result = prime * result + ((mBus == null) ? 0 : mBus.hashCode());
        return result;
    }
    /** Compares this object to another for equality. */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null) return false;
        if (!(obj instanceof CanDeviceId)) return false;
        CanDeviceId other = (CanDeviceId) obj;
        if (mDeviceNumber != other.mDeviceNumber) return false;
        if (mBus == null) {
            if (other.mBus != null) return false;
        } else if (!mBus.equals(other.mBus)) return false;
        return true;
    }
}
