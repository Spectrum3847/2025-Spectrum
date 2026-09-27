package frc.spectrumLib.sim;

/**
 * Represents a simulation component that other mechanisms can be attached to. Implementations
 * expose their current position and angle so child mechanisms can update their own position each
 * simulation period.
 */
public interface Mount {

    /**
     * Discriminates between the two supported mount types so child mechanisms can apply the correct
     * positioning logic.
     */
    public enum MountType {
        /** A linear (elevator-style) stage mount. */
        LINEAR,
        /** A single-jointed arm mount. */
        ARM,
    }

    /**
     * Returns the type of this mount, used by child mechanisms to select positioning logic.
     *
     * @return this mount's {@link MountType}
     */
    MountType getMountType();

    /**
     * Returns the horizontal displacement of this mount from its initial position (metres).
     *
     * @return horizontal displacement in metres
     */
    double getDisplacementX();

    /**
     * Returns the vertical displacement of this mount from its initial position (metres).
     *
     * @return vertical displacement in metres
     */
    double getDisplacementY();

    /**
     * Returns the current absolute angle of this mount in radians.
     *
     * @return angle in radians
     */
    double getAngle();

    /**
     * Returns the X coordinate that child mechanisms should use as their attachment point (metres).
     *
     * @return mount X position in metres
     */
    double getMountX();

    /**
     * Returns the Y coordinate that child mechanisms should use as their attachment point (metres).
     *
     * @return mount Y position in metres
     */
    double getMountY();
}
