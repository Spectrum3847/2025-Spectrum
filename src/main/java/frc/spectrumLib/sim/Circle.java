package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

public class Circle {

    private MechanismRoot2d rollerAxle;

    @SuppressWarnings("unused")
    private MechanismLigament2d rollerViz;

    @Getter private MechanismLigament2d[] circleBackground;
    @Getter private int backgroundLines;
    private double diameterInches;
    private MechanismRoot2d root;
    @Setter private Color8Bit color = new Color8Bit(Color.kBlack);
    @Setter private String name;

    public Circle(
            int backgroundLines,
            double diameterInches,
            String name,
            MechanismRoot2d root,
            Mechanism2d mech) {
        this.backgroundLines = backgroundLines;
        this.diameterInches = diameterInches;
        this.name = name;
        this.root = root;
        this.circleBackground = new MechanismLigament2d[this.backgroundLines];
        this.rollerAxle = mech.getRoot(name + " Axle", 0.0, 0.0);
        drawCircle();
    }

    public Circle(
            Mechanism2d mech,
            int backgroundLines,
            double diameterInches,
            String name,
            MechanismRoot2d root,
            Color8Bit color) {
        this(backgroundLines, diameterInches, name, root, mech);
        this.color = color;
    }

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

    public void setBackgroundColor(Color8Bit color) {
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i].setColor(color);
        }
    }

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
