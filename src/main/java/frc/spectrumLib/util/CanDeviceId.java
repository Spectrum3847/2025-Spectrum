package frc.spectrumLib.util;

// Based on 254-2023 Class
// https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java
public class CanDeviceId {
    private final int mDeviceNumber;
    private final String mBus;

    public CanDeviceId(int deviceNumber, String bus) {
        mDeviceNumber = deviceNumber;
        mBus = bus;
    }

    // Use the default bus name (empty string).
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() {
        return mDeviceNumber;
    }

    public String getBus() {
        return mBus;
    }

    public boolean equals(CanDeviceId other) {
        return equals((Object) other);
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + mDeviceNumber;
        result = prime * result + ((mBus == null) ? 0 : mBus.hashCode());
        return result;
    }

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
