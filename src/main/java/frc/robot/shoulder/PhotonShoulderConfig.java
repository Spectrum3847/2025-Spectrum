package frc.robot.shoulder;

import frc.robot.shoulder.Shoulder.ShoulderConfig;
import lombok.*;

public class PhotonShoulderConfig extends ShoulderConfig {
    // Positions set as percentage of photonShoulder
    @Getter private final int initializedPosition = 20;

    @Getter private final double tolerance = 0.95;

    @Getter private final double offset = -90;
    @Getter private final double initPosition = 0;

    @Getter private final double climbPrep = 90;

    @Getter private final double stationIntake = 36.03515625;
    @Getter private final double stationIntakeExtended = 36;

    @Getter private final double l1Coral = 33;
    @Getter private final double l2Coral = 10;
    @Getter private final double l3Coral = 233.4;
    @Getter private final double l4Coral = 203.927734375;

    @Getter private final double l2Algae = 43;
    @Getter private final double l3Alage = l2Algae;

    /* PhotonShoulder config settings */
    @Getter private final double zeroSpeed = -0.1;
    @Getter private final double holdMaxSpeedRPM = 18;

    @Getter private final double currentLimit = 30;
    @Getter private final double torqueCurrentLimit = 100;
    @Getter private final double positionKp = 1500;
    @Getter private final double positionKd = 140;
    @Getter private final double positionKv = 0;
    @Getter private final double positionKs = 0.06;
    @Getter private final double positionKa = 0.001;
    @Getter private final double positionKg = 12.5;
    @Getter private final double mmCruiseVelocity = 10;
    @Getter private final double mmAcceleration = 50;
    @Getter private final double mmJerk = 0;

    /* Cancoder config settings */
    @Getter private final double CANcoderGearRatio = 30.0 / 36.0;
    @Getter private double CANcoderOffset = 0;
    @Getter private boolean isCANcoderAttached = false;

    /* Sim properties */
    @Getter private double photonShoulderX = 0.8;
    @Getter private double photonShoulderY = 1.1;

    @Getter @Setter private double simRatio = 1;

    @Getter private double length = 0.3;

    public PhotonShoulderConfig() {
        super();
        setPhoton(true);
        setInitPosition(initPosition);
        setClimbPrep(climbPrep);
        setStationIntake(stationIntake);
        setStationExtendedIntake(stationIntakeExtended);
        setL1Coral(l1Coral);
        setL2Coral(l2Coral);
        setL2Score(l2Coral);
        setL3Coral(l3Coral);
        setL3Score(l3Coral);
        setL4Coral(l4Coral);
        setL4CoralScore(l4Coral);
        setL2Algae(l2Algae);
        setL3Algae(l3Alage);
        configPIDGains(0, positionKp, 0, positionKd);
        configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
        configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
        configGearRatio(102.857);
        configSupplyCurrentLimit(currentLimit, true);
        configForwardTorqueCurrentLimit(torqueCurrentLimit);
        configReverseTorqueCurrentLimit(-torqueCurrentLimit);
        configMinMaxRotations(-.25, 0.5);
        configReverseSoftLimit(getMinRotations(), true);
        configForwardSoftLimit(getMaxRotations(), true);
        configNeutralBrakeMode(true);
        configCounterClockwise_Positive();
        configGravityType(true);
        setSimRatio(102.857);
    }
}
