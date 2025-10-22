package frc.robot;


public class Coordinator {
    
    public void applyRobotState(State state) {
        switch (state) {
            case REHOME -> {
                // Implement rehome logic
                State.setLeft(true);
                State.setReversed(false);
            }
            case IDLE_EMPTY -> {
                // Implement scoring logic
            }
            case IDLE_ALGAE -> {
                // Implement climbing logic
            }
            case IDLE_CORAL -> {
                // Implement idle logic
            }
            case STARTING_POS -> {
                // Implement starting position logic
            }
            case STARTING_POS_CORAL -> {
                // Implement starting position coral logic
            }
            case ALGAE_INTAKE_FLOOR -> {
                // Implement algae intake from floor logic
            }
            case ALGAE_INTAKE_L3, ALGAE_INTAKE_L2 -> {
                // Implement algae intake level 3 logic
            }
            case ALGAE_NET_READY -> {
                // Implement algae net waiting logic
            }
            case ALGAE_NET_RELEASE -> {
                // Implement algae net release logic
            }
            case CORAL_INTAKE_FLOOR -> {
                // Implement coral intake from floor logic
            }

            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
