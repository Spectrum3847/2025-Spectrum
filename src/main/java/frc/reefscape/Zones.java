package frc.reefscape;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.swerve.Swerve;

public class Zones {

    private static final Swerve swerve = Robot.getSwerve();
    private static final double[] robot2d = {Robot.getSwerve().getRobotPose().getX(), Robot.getSwerve().getRobotPose().getY()};

    // zones
    public static final Trigger topLeftZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger topRightZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));
    public static final Trigger bottomLeftZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger bottomRightZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));


    //TODO: add the reef face zones of the 12 tags
    public static final double[][] blueReefZones = {{17},{18},{19},{20},{21},{22}};
    public static final double[][] redReefZones = {{6},{7},{8},{9},{10},{11}};

    public static final Trigger bargeZone =
            swerve.inXzoneAlliance(
                            3 * Field.getHalfLength() / 4,
                            Field.getHalfLength()
                                    - Units.inchesToMeters(24)
                                    - swerve.getConfig().getRobotLength() / 2)
                    .and(topLeftZone);


   /**
     * Cross product of two vectors
     * @param p1 
     * @param p2  
     * @param p point of the robot
     * @return
     */
    public double crossProduct(double[] p1, double[] p2, double[] p) {
        return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p2[1] - p1[1]) * (p[0] - p1[0]);
    }


    /**
     * Confirms the robot is in a given triangular 2d plane
     * made of the cross product of the given points to check
     * if the p4 is in the given plane
     * @param p1
     * @param p2 
     * @param p3 
     * @param p4
     * @return
     */
    public boolean isPointInZone(double[] p1, double[] p2, double[] p3, double[] p4) {
        double cross1 = crossProduct(p1, p2, p4);
        double cross2 = crossProduct(p2, p3, p4);
        double cross3 = crossProduct(p3, p1, p4);

        if ((cross1 >= 0 && cross2 >= 0 && cross3 >= 0 ) 
        || (cross1 <= 0 && cross2 <= 0 && cross3 <= 0 )) {
            return true;
        }
        return false;
    }


    /**
     * Blue Zones of the reef
     *  
     * Checks the reef face points of each reef face to the position of the robot
     * to determine the angle between the robot heading and the reef face
     * with a triangle formed by the reef face points and the center of the reef
     * @param robotPose
     */
    public double getBlueReefZoneID(double[] robotPose) {
        double[] reefBlueCenter = {Field.Reef.center.getX(), Field.Reef.center.getY()};
        double distanceFactor = 85;
        double[] blueReefTagAngles = {240,180,120,60,0,300}; // Reef face ID's 17-22

        for (int i = 0; i < 6 ; i++) {
            double theta = Math.toRadians(60);
            double reefTheta = blueReefTagAngles[i]; //given theta of reef face

            if (reefTheta == 240) {
                double point1[] = {reefBlueCenter[0] - distanceFactor * Math.cos(theta), reefBlueCenter[1] - distanceFactor * Math.sin(theta)};
                double point2[] = {reefBlueCenter[0], reefBlueCenter[1] - distanceFactor};
                if (isPointInZone(point1, point2, reefBlueCenter, robotPose)) {
                    return i + 17; //returns the tag
                }
            }
            
            if (reefTheta == 180) {
                double point1[] = {reefBlueCenter[0] - distanceFactor * Math.cos(theta), reefBlueCenter[1] - distanceFactor * Math.sin(theta)};
                double point2[] = {reefBlueCenter[0] - distanceFactor * Math.cos(theta), reefBlueCenter[1] + distanceFactor * Math.sin(theta)};
                if (isPointInZone(point1, point2, reefBlueCenter, robotPose)) {
                    return i + 17; //returns the tag
                }
            }

            if (reefTheta == 120) {
                double point1[] = {reefBlueCenter[0] - distanceFactor * Math.cos(theta), reefBlueCenter[1] + distanceFactor * Math.sin(theta)};
                double point2[] = {reefBlueCenter[0], reefBlueCenter[1] + distanceFactor};
                if (isPointInZone(point1, point2, reefBlueCenter, robotPose)) {
                    return i + 17; //returns the tag
                }
            }

            if (reefTheta == 60) {
                double point1[] = {reefBlueCenter[0], reefBlueCenter[1] + distanceFactor};
                double point2[] = {reefBlueCenter[0] + distanceFactor * Math.cos(theta), reefBlueCenter[1] + Math.sin(theta)};
                if (isPointInZone(point1, point2, reefBlueCenter, robotPose)) {
                    return i + 17; //returns the tag
                }
            }

            if (reefTheta == 0) {
                double point1[] = {reefBlueCenter[0] + distanceFactor * Math.cos(theta), reefBlueCenter[1] + Math.sin(theta)};
                double point2[] = {reefBlueCenter[0] + distanceFactor * Math.cos(theta), reefBlueCenter[1] - distanceFactor * Math.sin(theta)};
                if (isPointInZone(point1, point2, reefBlueCenter, robotPose)) {
                    return i + 17; //returns the tag
                }
            }

            if (reefTheta == 300) {
                double point1[] = {reefBlueCenter[0] + distanceFactor * Math.cos(theta), reefBlueCenter[1] - distanceFactor * Math.sin(theta)};
                double point2[] = {reefBlueCenter[0], reefBlueCenter[1]- distanceFactor};
                if (isPointInZone(point1, point2, reefBlueCenter, robotPose)) {
                    return i + 17; //returns the tag
                }
            }
        }

      // Not in any reef zone
      return -1;
    }

    /**
     * Red Zones of the reef
     *  
     * Checks the reef face points of each reef face to the position of the robot
     * to determine the angle between the robot heading and the reef face
     * with a triangle formed by the reef face points and the center of the reef
     * @param robotPose
     */
    public double getRedReefZoneID(double[] robotPose) {
        double[] reefRedCenter = {Field.Reef.center.getX() + 370.1, Field.Reef.center.getY()};

        double distanceFactor = 85;
        double[] redReefTagAngles = {300,0,60,120,180,240}; // Reef face ID's 6-11

        for (int i = 0; i < 6 ; i++) {
            double theta = Math.toRadians(60);
            double reefTheta = redReefTagAngles[i]; //given theta of reef face

            if (reefTheta == 240) {
                double point1[] = {reefRedCenter[0] - distanceFactor * Math.cos(theta), reefRedCenter[1] - distanceFactor * Math.sin(theta)};
                double point2[] = {reefRedCenter[0], reefRedCenter[1] - distanceFactor};
                if (isPointInZone(point1, point2, reefRedCenter, robotPose)) {
                    return i + 6; //returns the tag
                }
            }
            
            if (reefTheta == 180) {
                double point1[] = {reefRedCenter[0] - distanceFactor * Math.cos(theta), reefRedCenter[1] - distanceFactor * Math.sin(theta)};
                double point2[] = {reefRedCenter[0] - distanceFactor * Math.cos(theta), reefRedCenter[1] + distanceFactor * Math.sin(theta)};
                if (isPointInZone(point1, point2, reefRedCenter, robotPose)) {
                    return i + 6; //returns the tag
                }
            }

            if (reefTheta == 120) {
                double point1[] = {reefRedCenter[0] - distanceFactor * Math.cos(theta), reefRedCenter[1] + distanceFactor * Math.sin(theta)};
                double point2[] = {reefRedCenter[0], reefRedCenter[1] + distanceFactor};
                if (isPointInZone(point1, point2, reefRedCenter, robotPose)) {
                    return i + 6; //returns the tag
                }
            }

            if (reefTheta == 60) {
                double point1[] = {reefRedCenter[0], reefRedCenter[1] + distanceFactor};
                double point2[] = {reefRedCenter[0] + distanceFactor * Math.cos(theta), reefRedCenter[1] + Math.sin(theta)};
                if (isPointInZone(point1, point2, reefRedCenter, robotPose)) {
                    return i + 6; //returns the tag
                }
            }

            if (reefTheta == 0) {
                double point1[] = {reefRedCenter[0] + distanceFactor * Math.cos(theta), reefRedCenter[1] + Math.sin(theta)};
                double point2[] = {reefRedCenter[0] + distanceFactor * Math.cos(theta), reefRedCenter[1] - distanceFactor * Math.sin(theta)};
                if (isPointInZone(point1, point2, reefRedCenter, robotPose)) {
                    return i + 6; //returns the tag
                }
            }

            if (reefTheta == 300) {
                double point1[] = {reefRedCenter[0] + distanceFactor * Math.cos(theta), reefRedCenter[1] - distanceFactor * Math.sin(theta)};
                double point2[] = {reefRedCenter[0], reefRedCenter[1]- distanceFactor};
                if (isPointInZone(point1, point2, reefRedCenter, robotPose)) {
                    return i + 6; //returns the tag
                }
            }
        }
        // Not in any reef zone
        return -1;
   }





}