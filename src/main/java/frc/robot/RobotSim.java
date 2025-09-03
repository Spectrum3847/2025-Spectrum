package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// General Sim principles
// Always move the root/origin to change it's display position
// Looking at the robot from the left view (right side of the robot)
public class RobotSim {
    public static final double height = 140; // 60;
    public static final double width = 60;

    public static final Translation2d origin =
            new Translation2d(Units.inchesToMeters(width / 2), 0.0);

    public static final Mechanism2d leftView =
            new Mechanism2d(Units.inchesToMeters(width) * 2, Units.inchesToMeters(height));

    public static final Mechanism2d frontView =
            new Mechanism2d(Units.inchesToMeters(width) * 2, Units.inchesToMeters(height));

    public RobotSim() {
        SmartDashboard.putData("LeftView", RobotSim.leftView);
        SmartDashboard.putData("frontView", RobotSim.frontView);
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        frontView.setBackgroundColor(new Color8Bit(Color.kLightGray));

        addReef();
    }

    private void addReef() {
        MechanismRoot2d reefBase = leftView.getRoot("Reef Base", 1.9, 0.35);

        @SuppressWarnings("unused")
        MechanismLigament2d mainReef =
                reefBase.append(
                        new MechanismLigament2d(
                                "Main Reef",
                                Units.inchesToMeters(50) + 0.5,
                                90,
                                5,
                                new Color8Bit(Color.kPurple)));

        MechanismRoot2d branch1Base = leftView.getRoot("L2 Branch Base", 1.55, 1.40);
        branch1Base.append(
                new MechanismLigament2d(
                        "0L2Branch",
                        Units.inchesToMeters(15),
                        -30,
                        3,
                        new Color8Bit(Color.kPurple)));

        MechanismRoot2d branch2Base = leftView.getRoot("L3 Branch Base", 1.55, 1.85);
        branch2Base.append(
                new MechanismLigament2d(
                        "0L3Branch",
                        Units.inchesToMeters(15),
                        -30,
                        3,
                        new Color8Bit(Color.kPurple)));

        MechanismRoot2d branch3Base = leftView.getRoot("L4 Branch Base", 1.55, 2.30);
        branch3Base.append(
                new MechanismLigament2d(
                        "0L4Branch",
                        Units.inchesToMeters(15),
                        -30,
                        3,
                        new Color8Bit(Color.kPurple)));

        branch3Base.append(
                new MechanismLigament2d(
                        "Top Extension",
                        Units.inchesToMeters(10),
                        90,
                        3,
                        new Color8Bit(Color.kPurple)));
    }
}
