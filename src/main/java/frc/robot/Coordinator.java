package frc.robot;

import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.ElevatorStates;
import frc.robot.intake.IntakeStates;
import frc.robot.shoulder.ShoulderStates;
import frc.robot.twist.TwistStates;

public class Coordinator {

    public void update() {}

    public void applyRobotState(State state) {
        switch (state) {
            case REHOME -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                TwistStates.home();
                IntakeStates.neutral();
            }
            case IDLE_EMPTY -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                TwistStates.home();
                IntakeStates.neutral();
            }
            case IDLE_ALGAE -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                TwistStates.home();
                IntakeStates.intakeAlgae();
            }
            case IDLE_CORAL -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                TwistStates.home();
                IntakeStates.holdCoral();
            }
            case STARTING_POS -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                TwistStates.home();
                IntakeStates.neutral();
            }
            case STARTING_POS_CORAL -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                TwistStates.home();
                IntakeStates.holdCoral();
            }
            case ALGAE_INTAKE_FLOOR -> {
                ElevatorStates.groundAlgae();
                ShoulderStates.groundAlgae();
                ElbowStates.groundAlgae();
                TwistStates.groundAlgae();
                IntakeStates.intakeAlgae();
            }
            case ALGAE_INTAKE_L2 -> {
                ElevatorStates.L2Algae();
                ShoulderStates.L2Algae();
                ElbowStates.L2Algae();
                TwistStates.intakeAlgae();
                IntakeStates.intakeAlgae();
            }
            case ALGAE_INTAKE_L3 -> {
                ElevatorStates.L3Algae();
                ShoulderStates.L3Algae();
                ElbowStates.L3Algae();
                TwistStates.intakeAlgae();
                IntakeStates.intakeAlgae();
            }
            case ALGAE_NET_READY -> {
                ElevatorStates.netAlgae();
                ShoulderStates.netAlgae();
                ElbowStates.netAlgae();
                TwistStates.netAlgae();
                IntakeStates.intakeAlgae();
            }
            case ALGAE_NET_RELEASE -> {
                ElevatorStates.netAlgae();
                ShoulderStates.netAlgae();
                ElbowStates.netAlgae();
                TwistStates.netAlgae();
                IntakeStates.scoreAlgae();
            }
            case CORAL_INTAKE_FLOOR -> {
                ElevatorStates.groundCoral();
                ShoulderStates.groundCoral();
                ElbowStates.groundCoral();
                TwistStates.groundCoral();
                IntakeStates.intakeCoral();
            }
            case CORAL_INTAKE_HUMAN -> {
                ElevatorStates.humanCoral();
                ShoulderStates.humanCoral();
                ElbowStates.humanCoral();
                TwistStates.humanIntake();
                IntakeStates.intakeCoral();
            }
            case CORAL_L1_READY -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                TwistStates.L1Coral();
                IntakeStates.holdCoral();
            }
            case CORAL_L1_PREP -> {
                ElevatorStates.L1Coral();
                ShoulderStates.L1Coral();
                ElbowStates.L1Coral();
                TwistStates.L1Coral();
                IntakeStates.holdCoral();
            }
            case CORAL_L1_RELEASE -> {
                ElevatorStates.L1Coral();
                ShoulderStates.L1Coral();
                ElbowStates.L1Coral();
                TwistStates.L1Coral();
                IntakeStates.scoreCoral();
            }
            case CORAL_L2_READY, CORAL_L3_READY, CORAL_L4_READY -> {
                ElevatorStates.home();
                ShoulderStates.home();
                ElbowStates.home();
                if (state.isLeft()) {
                    TwistStates.coralLeft();
                } else {
                    TwistStates.coralRight();
                }
                IntakeStates.holdCoral();
            }
            case CORAL_L2_PREP -> {
                ElevatorStates.L2CoralPrep();
                ShoulderStates.L2CoralPrep();
                ElbowStates.L2CoralPrep();
                if (state.isLeft()) {
                    TwistStates.coralLeft();
                } else {
                    TwistStates.coralRight();
                }
                IntakeStates.holdCoral();
            }
            case CORAL_L2_RELEASE -> {
                ElevatorStates.L2CoralRelease();
                ShoulderStates.L2CoralRelease();
                ElbowStates.L2CoralRelease();
                if (state.isLeft()) {
                    TwistStates.coralLeft();
                } else {
                    TwistStates.coralRight();
                }
                IntakeStates.scoreCoral();
            }
            case CORAL_L3_PREP -> {
                ElevatorStates.L3CoralPrep();
                ShoulderStates.L3CoralPrep();
                ElbowStates.L3CoralPrep();
                if (state.isLeft()) {
                    TwistStates.coralLeft();
                } else {
                    TwistStates.coralRight();
                }
                IntakeStates.holdCoral();
            }
            case CORAL_L3_RELEASE -> {
                ElevatorStates.L3CoralRelease();
                ShoulderStates.L3CoralRelease();
                ElbowStates.L3CoralRelease();
                if (state.isLeft()) {
                    TwistStates.coralLeft();
                } else {
                    TwistStates.coralRight();
                }
                IntakeStates.scoreCoral();
            }
            case CORAL_L4_PREP -> {
                ElevatorStates.L4CoralPrep();
                ShoulderStates.L4CoralPrep();
                ElbowStates.L4CoralPrep();
                if (state.isLeft()) {
                    TwistStates.coralLeft();
                } else {
                    TwistStates.coralRight();
                }
                IntakeStates.holdCoral();
            }
            case CORAL_L4_RELEASE -> {
                ElevatorStates.L4CoralRelease();
                ShoulderStates.L4CoralRelease();
                ElbowStates.L4CoralRelease();
                if (state.isLeft()) {
                    TwistStates.coralLeft();
                } else {
                    TwistStates.coralRight();
                }
                IntakeStates.scoreCoral();
            }
            case CLIMING_APPROACH -> {
                ElevatorStates.home();
                ShoulderStates.climbPrep();
                ElbowStates.climbPrep();
                TwistStates.climbPrep();
                IntakeStates.neutral();
            }

            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
