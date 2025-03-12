package frc.robot.elevator;

import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.spectrumLib.Rio;
import frc.spectrumLib.mechanism.Mechanism.FollowerConfig;
import lombok.Getter;

public class PhotonElevatorConfig extends ElevatorConfig {
    /* Elevator constants in rotations */
    @Getter private double maxRotations = 21;
    @Getter private double minRotations = 0.3;

    @Getter private final double stationIntake = 9.67;
    @Getter private final double stationExtendedIntake = 8.5;

    @Getter private final double L1Coral = 0;
    @Getter private final double L2Coral = 10;
    @Getter private final double L3Coral = 8;
    @Getter private final double L4Coral = 19.5;

    @Getter private final double L2Algae = 9.8;
    @Getter private final double L3Algae = 19.5;

    @Getter private final double zeroSpeed = -0.1;
    @Getter private final double holdMaxSpeedRPM = 18;
    @Getter private final double positionKp = 100;
    @Getter private final double positionKd = 6;
    @Getter private final double positionKa = 0.2;
    @Getter private final double positionKv = 0;
    @Getter private final double positionKs = 5;
    @Getter private final double positionKg = 25.3;
    @Getter private final double mmCruiseVelocity = 40;
    @Getter private final double mmAcceleration = 280;
    @Getter private final double mmJerk = 2000;

    @Getter private double currentLimit = 40;
    @Getter private double torqueCurrentLimit = 160;

    public PhotonElevatorConfig() {
        super();
        setPhoton(true);
        setStationIntake(stationIntake);
        setStationExtendedIntake(stationExtendedIntake);
        setL1Coral(L1Coral);
        setL2Coral(L2Coral);
        setL2Score(L2Coral);
        setL3Coral(L3Coral);
        setL3Score(L3Coral);
        setL4Coral(L4Coral);
        setL4Score(L4Coral);
        setProcessorAlgae(0);
        setL2Algae(L2Algae);
        setL3Algae(L3Algae);
        setNetAlgae(0);
        configMinMaxRotations(minRotations, maxRotations);
        configForwardSoftLimit(maxRotations, true);
        configReverseSoftLimit(minRotations, true);
        configPIDGains(0, positionKp, 0, positionKd);
        configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
        configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
        configSupplyCurrentLimit(currentLimit, true);
        configStatorCurrentLimit(torqueCurrentLimit, true);
        configForwardTorqueCurrentLimit(torqueCurrentLimit);
        configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
        configNeutralBrakeMode(true);
        configCounterClockwise_Positive();
        setFollowerConfigs(new FollowerConfig("ElevatorRear", 41, Rio.CANIVORE, true));
    }
}
