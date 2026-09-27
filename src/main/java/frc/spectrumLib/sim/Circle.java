package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/**
 * Renders a filled circle in a {@link Mechanism2d} canvas using evenly-spaced radial ligaments.
 * Used by {@link RollerSim} to visualise the roller's spin state and color.
 */
public class Circle {

    private MechanismRoot2d rollerAxle;

    @SuppressWarnings("unused")
    private MechanismLigament2d rollerViz;

    /** Array of radial ligaments that together form the circle outline. */
    @Getter private MechanismLigament2d[] circleBackground;
    /** Number of radial lines used to approximate the circle. */
    @Getter private int backgroundLines;

    private double diameterInches;
    private MechanismRoot2d root;
    /** Color applied to all background lines when the circle is drawn. */
    @Setter private Color8Bit color = new Color8Bit(Color.kBlack);
    /** Label prefix used when naming Mechanism2d elements. */
    @Setter private String name;

    /**
     * Creates a Circle and immediately draws the radial background lines.
     *
     * @param backgroundLines number of evenly-spaced radial lines used to approximate the circle
     * @param diameterInches diameter of the circle in inches
     * @param name label prefix for Mechanism2d element names
     * @param root the Mechanism2d root the circle is attached to
     * @param mech the parent Mechanism2d canvas
     */
    public Circle(
            int backgroundLines,
            double diameterInches,
            String name,
            MechanismRoot2d root,
            Mechanism2d mech) {
        this(mech, backgroundLines, diameterInches, name, root, new Color8Bit(Color.kBlack));
    }

    /**
     * Creates a Circle with a specified color and immediately draws the radial background lines.
     *
     * @param mech the parent Mechanism2d canvas
     * @param backgroundLines number of evenly-spaced radial lines used to approximate the circle
     * @param diameterInches diameter of the circle in inches
     * @param name label prefix for Mechanism2d element names
     * @param root the Mechanism2d root the circle is attached to
     * @param color the initial color applied to every background line
     */
    public Circle(
            Mechanism2d mech,
            int backgroundLines,
            double diameterInches,
            String name,
            MechanismRoot2d root,
            Color8Bit color) {
        this.backgroundLines = backgroundLines;
        this.diameterInches = diameterInches;
        this.name = name;
        this.root = root;
        this.circleBackground = new MechanismLigament2d[this.backgroundLines];
        this.rollerAxle = mech.getRoot(name + " Axle", 0.0, 0.0);
        this.color = color;
        drawCircle();
    }

    /**
     * Creates and attaches all radial background ligaments that form the circle, distributing them
     * evenly around 360 degrees.
     */
    public void drawCircle() {
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i] =
                    root.append(
                            new MechanismLigament2d(
                                    name + " Background " + i,
                                    Units.inchesToMeters(diameterInches) / 2.0,
                                    (360.0 / backgroundLines) * i,
                                    diameterInches,
                                    color));
        }
    }

    /**
     * Appends a short white indicator line to the roller axle so rotation direction is visible in
     * the Mechanism2d canvas.
     */
    public void drawViz() {
        rollerViz =
                rollerAxle.append(
                        new MechanismLigament2d(
                                name + " Roller",
                                Units.inchesToMeters(diameterInches) / 2.0,
                                0.0,
                                5.0,
                                new Color8Bit(Color.kWhite)));
    }

    /**
     * Sets all background radial lines to the same color.
     *
     * @param color the color to apply to every background line
     */
    public void setBackgroundColor(Color8Bit color) {
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i].setColor(color);
        }
    }

    /**
     * Alternates two colors across the background lines, giving the circle a two-tone appearance
     * (useful for indicating reverse spin direction).
     *
     * @param color8Bit color applied to even-indexed background lines
     * @param color8Bit2 color applied to odd-indexed background lines
     */
    public void setHalfBackground(Color8Bit color8Bit, Color8Bit color8Bit2) {
        for (int i = 0; i < backgroundLines; i++) {
            if (i % 2 == 0) {
                circleBackground[i].setColor(color8Bit);
            } else {
                circleBackground[i].setColor(color8Bit2);
            }
        }
    }
}
