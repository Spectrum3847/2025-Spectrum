package frc.robot;

import com.google.common.collect.ImmutableMap;
import frc.robot.intake.IntakeStates;
import java.util.Map;
import lombok.Getter;

public enum State {
    REHOME(Claw.EMPTY, true),

    IDLE_EMPTY(Claw.EMPTY, false),
    IDLE_ALGAE(Claw.ALGAE, false),
    IDLE_CORAL(Claw.CORAL, false),
    STARTING_POS(Claw.EMPTY, false),
    STARTING_POS_CORAL(Claw.CORAL, false),

    ALGAE_INTAKE_FLOOR(Claw.ALGAE, false),
    ALGAE_INTAKE_L2(Claw.ALGAE, false),
    ALGAE_INTAKE_L3(Claw.ALGAE, false),

    ALGAE_NET_WAITING(Claw.ALGAE, false),
    ALGAE_NET_RELEASE(Claw.ALGAE, false),

    CORAL_INTAKE_FLOOR(Claw.CORAL, false),

    CORAL_L2_LINEUP_LEFT(Claw.CORAL, false),
    CORAL_L2_LINEUP_RIGHT(Claw.CORAL, false),
    CORAL_L2_PLACE_LEFT(Claw.CORAL, false),
    CORAL_L2_PLACE_RIGHT(Claw.CORAL, false),
    CORAL_L2_RELEASE_LEFT(Claw.CORAL, false),
    CORAL_L2_RELEASE_RIGHT(Claw.CORAL, false),

    CORAL_L3_LINEUP_LEFT(Claw.CORAL, false),
    CORAL_L3_LINEUP_RIGHT(Claw.CORAL, false),
    CORAL_L3_PLACE_LEFT(Claw.CORAL, false),
    CORAL_L3_PLACE_RIGHT(Claw.CORAL, false),
    CORAL_L3_RELEASE_LEFT(Claw.CORAL, false),
    CORAL_L3_RELEASE_RIGHT(Claw.CORAL, false),

    CORAL_L4_LINEUP_LEFT(Claw.CORAL, false),
    CORAL_L4_LINEUP_RIGHT(Claw.CORAL, false),
    CORAL_L4_PLACE_LEFT(Claw.CORAL, false),
    CORAL_L4_PLACE_RIGHT(Claw.CORAL, false),
    CORAL_L4_RELEASE_LEFT(Claw.CORAL, false),
    CORAL_L4_RELEASE_RIGHT(Claw.CORAL, false),

    CLIMING_APPROACH(Claw.EMPTY, true),
    CLIMBING_HANG(Claw.EMPTY, true),
    CLIMBING_LOCK(Claw.EMPTY, true);

    public enum Claw {
        EMPTY,
        ALGAE,
        CORAL
    }

    @Getter public final Claw claw;
    @Getter public final boolean specialMode;

    private State(Claw clawGp, boolean specialMode) {
        this.claw = clawGp;
        this.specialMode = specialMode;
    }

    private static final ImmutableMap<State, State> scoreSequence =
            ImmutableMap.ofEntries(
                    Map.entry(CORAL_L2_LINEUP_LEFT, CORAL_L2_PLACE_LEFT),
                    Map.entry(CORAL_L2_LINEUP_RIGHT, CORAL_L2_PLACE_RIGHT),
                    Map.entry(CORAL_L3_LINEUP_LEFT, CORAL_L3_PLACE_LEFT),
                    Map.entry(CORAL_L3_LINEUP_RIGHT, CORAL_L3_PLACE_RIGHT),
                    Map.entry(CORAL_L4_LINEUP_LEFT, CORAL_L4_PLACE_LEFT),
                    Map.entry(CORAL_L4_LINEUP_RIGHT, CORAL_L4_PLACE_RIGHT),
                    Map.entry(CORAL_L2_PLACE_LEFT, CORAL_L2_RELEASE_LEFT),
                    Map.entry(CORAL_L2_PLACE_RIGHT, CORAL_L2_RELEASE_RIGHT),
                    Map.entry(CORAL_L3_PLACE_LEFT, CORAL_L3_RELEASE_LEFT),
                    Map.entry(CORAL_L3_PLACE_RIGHT, CORAL_L3_RELEASE_RIGHT),
                    Map.entry(CORAL_L4_PLACE_LEFT, CORAL_L4_RELEASE_LEFT),
                    Map.entry(CORAL_L4_PLACE_RIGHT, CORAL_L4_RELEASE_RIGHT));

    public static boolean isLineupState(State state) {
        return switch (state) {
            case CORAL_L4_LINEUP_RIGHT,
                    CORAL_L4_LINEUP_LEFT,
                    CORAL_L3_LINEUP_LEFT,
                    CORAL_L3_LINEUP_RIGHT,
                    CORAL_L2_LINEUP_LEFT,
                    CORAL_L2_LINEUP_RIGHT -> true;
            default -> false;
        };
    }

    public static boolean missingGP(State state, boolean hasGp) {
        return (!state.claw.equals(Claw.EMPTY) && !hasGp);
    }

    public static boolean isReleaseState(State state) {
        return switch (state) {
            case CORAL_L2_RELEASE_LEFT,
                    CORAL_L2_RELEASE_RIGHT,
                    CORAL_L3_RELEASE_LEFT,
                    CORAL_L3_RELEASE_RIGHT,
                    CORAL_L4_RELEASE_LEFT,
                    CORAL_L4_RELEASE_RIGHT -> true;
            default -> false;
        };
    }

    public static boolean isSpecialMode(State state) {
        return state.specialMode;
    }

    public State getNextScoreState() {
        return scoreSequence.getOrDefault(this, this);
    }

    protected State getNextState(State currentState) {
        State nextState = this; // Default to the current state

        if (State.missingGP(currentState, IntakeStates.hasGamePiece.getAsBoolean())) {
            return State.IDLE_EMPTY;
        }

        return switch (currentState) {
            case CORAL_L2_PLACE_LEFT,
                    CORAL_L2_PLACE_RIGHT,
                    CORAL_L3_PLACE_LEFT,
                    CORAL_L3_PLACE_RIGHT,
                    CORAL_L4_PLACE_LEFT,
                    CORAL_L4_PLACE_RIGHT -> {
                nextState = currentState.getNextScoreState();
                yield nextState;
            }

            case CORAL_L2_LINEUP_LEFT,
                    CORAL_L2_LINEUP_RIGHT,
                    CORAL_L3_LINEUP_LEFT,
                    CORAL_L3_LINEUP_RIGHT,
                    CORAL_L4_LINEUP_LEFT,
                    CORAL_L4_LINEUP_RIGHT -> {
                nextState = currentState.getNextScoreState();
                yield nextState;
            }

            case CORAL_INTAKE_FLOOR -> {
                nextState = State.IDLE_CORAL;
                yield nextState;
            }

            case ALGAE_INTAKE_L2, ALGAE_INTAKE_L3, ALGAE_INTAKE_FLOOR -> {
                nextState = State.IDLE_ALGAE;
                yield nextState;
            }

            case ALGAE_NET_WAITING -> {
                nextState = State.ALGAE_NET_RELEASE;
                yield nextState;
            }

            default -> nextState;
        };
    }
}
