package frc.robot.intake;

import frc.robot.intake.Intake.IntakeConfig;
import lombok.Getter;
import lombok.Setter;

public class PhotonIntakeConfig extends IntakeConfig {

    // Coral Voltages and Current
    @Getter @Setter private double coralIntakeVoltage = 9.0;
    @Getter @Setter private double coralIntakeSupplyCurrent = 12.0;
    @Getter @Setter private double coralIntakeTorqueCurrent = 60.0;

    @Getter @Setter private double coralScoreVoltage = -6.0;
    @Getter @Setter private double coralScoreSupplyCurrent = 12;
    @Getter @Setter private double coralScoreTorqueCurrent = 60;

    @Getter @Setter private double coralL1ScoreVoltage = -6;
    @Getter @Setter private double coralL1ScoreSupplyCurrent = 12;
    @Getter @Setter private double coralL1ScoreTorqueCurrent = 60;

    /* Intake config values */
    @Getter private double currentLimit = 15;
    @Getter private double torqueCurrentLimit = 100;
    @Getter private double velocityKp = 12; // 0.156152;
    @Getter private double velocityKv = 0.2; // 0.12;
    @Getter private double velocityKs = 0;

    public PhotonIntakeConfig() {
        super();
        setCoralIntakeVoltage(coralIntakeVoltage);
        setCoralIntakeSupplyCurrent(coralIntakeSupplyCurrent);
        setCoralIntakeTorqueCurrent(coralIntakeTorqueCurrent);
        setCoralScoreVoltage(coralScoreVoltage);
        setCoralScoreSupplyCurrent(coralScoreSupplyCurrent);
        setCoralScoreTorqueCurrent(coralScoreTorqueCurrent);
        setCoralL1ScoreVoltage(coralL1ScoreVoltage);
        setCoralL1ScoreSupplyCurrent(coralL1ScoreSupplyCurrent);
        setCoralL1ScoreTorqueCurrent(coralL1ScoreTorqueCurrent);

        configPIDGains(0, velocityKp, 0, 0);
        configFeedForwardGains(velocityKs, velocityKv, 0, 0);
        configGearRatio(1);
        configSupplyCurrentLimit(currentLimit, true);
        configStatorCurrentLimit(torqueCurrentLimit, true);
        configForwardTorqueCurrentLimit(torqueCurrentLimit);
        configReverseTorqueCurrentLimit(torqueCurrentLimit);
        configNeutralBrakeMode(true);
        configCounterClockwise_Positive();
    }
}
