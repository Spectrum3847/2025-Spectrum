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
    public static final double height = 120; // 60;
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
        MechanismRoot2d reefBase = leftView.getRoot("Reef Base", 2, 0.55);

        MechanismLigament2d mainReef =
                reefBase.append(
                        new MechanismLigament2d(
                                "Main Reef",
                                Units.inchesToMeters(50),
                                90,
                                5,
                                new Color8Bit(Color.kPurple)));

        MechanismRoot2d branch1Base = leftView.getRoot("Branch1 Base", 1.65, 1);
        branch1Base.append(
                new MechanismLigament2d(
                        "Branch1", Units.inchesToMeters(15), -30, 3, new Color8Bit(Color.kPurple)));

        MechanismRoot2d branch2Base = leftView.getRoot("Branch2 Base", 1.65, 1.35);
        branch2Base.append(
                new MechanismLigament2d(
                        "Branch2", Units.inchesToMeters(15), -30, 3, new Color8Bit(Color.kPurple)));

        MechanismRoot2d branch3Base =
                leftView.getRoot("Branch3 Base", 1.65, 0.5 + Units.inchesToMeters(50));
        branch3Base.append(
                new MechanismLigament2d(
                        "Branch3", Units.inchesToMeters(15), -30, 3, new Color8Bit(Color.kPurple)));

        branch3Base.append(
                new MechanismLigament2d(
                        "Top Extension",
                        Units.inchesToMeters(15),
                        90,
                        3,
                        new Color8Bit(Color.kPurple)));
    }
}
