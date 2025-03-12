package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

public class ArmConfig {

    @Getter @Setter private int numMotors = 1;
    @Getter @Setter private double initialX = 0.7;
    @Getter @Setter private double initialY = 0.3;
    @Getter @Setter private double pivotX = 0.7;
    @Getter @Setter private double pivotY = 0.3;

    @Getter @Setter
    private double ratio =
            50; // the number of rotations it takes for the mechanism to do one revolution

    @Getter @Setter private double length = 0.5;
    @Getter @Setter private double simMOI = 1.2;
    @Getter @Setter private double simCGLength = 0.2;
    @Getter @Setter private double minAngle = Math.toRadians(-60);
    @Getter @Setter private double maxAngle = Math.toRadians(90);
    @Getter @Setter private double startingAngle = Math.toRadians(90);
    @Getter @Setter private boolean simulateGravity = true;
    @Getter private boolean mounted = false;
    @Getter private Mount mount;
    @Getter private double initMountX;
    @Getter private double initMountY;
    @Getter private double initMountAngle;
    @Getter private boolean absAngle;
    @Getter private Color8Bit color = new Color8Bit(Color.kBlue);

    public ArmConfig(
            double initialX,
            double initialY,
            double ratio,
            double length,
            double minAngleDegrees,
            double maxAngleDegrees,
            double startingAngleDegrees) {
        this.ratio = ratio;
        this.length = length;
        this.minAngle = Math.toRadians(minAngleDegrees);
        this.maxAngle = Math.toRadians(maxAngleDegrees);
        this.startingAngle = Math.toRadians(startingAngleDegrees);
        this.initialX = initialX;
        this.initialY = initialY;
        this.pivotX = initialX;
        this.pivotY = initialY;
    }

    public ArmConfig setColor(Color8Bit color) {
        this.color = color;
        return this;
    }

    public ArmConfig setMount(LinearSim sim, boolean fixedAngle) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
            this.absAngle = fixedAngle;
        }
        return this;
    }

    public ArmConfig setMount(ArmSim sim, boolean absAngle) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
            this.absAngle = absAngle;
        }
        return this;
    }
}
