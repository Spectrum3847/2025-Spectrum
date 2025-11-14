package frc.robot;

import com.google.common.collect.ImmutableMap;
import frc.spectrumLib.Telemetry;
import java.util.Map;
import java.util.function.BooleanSupplier;
import lombok.Setter;

public enum State {
    REHOME,

    IDLE_EMPTY,
    IDLE_ALGAE,
    IDLE_CORAL,
    STARTING_POS,
    STARTING_POS_CORAL,

    ALGAE_INTAKE_FLOOR,
    ALGAE_INTAKE_L2,
    ALGAE_INTAKE_L3,

    ALGAE_NET_READY,
    ALGAE_NET_RELEASE,

    CORAL_INTAKE_FLOOR,
    CORAL_INTAKE_HUMAN,

    CORAL_L1_READY,
    CORAL_L1_PREP,
    CORAL_L1_RELEASE,

    CORAL_L2_READY,
    CORAL_L2_PREP,
    CORAL_L2_RELEASE,

    CORAL_L3_READY,
    CORAL_L3_PREP,
    CORAL_L3_RELEASE,

    CORAL_L4_READY,
    CORAL_L4_PREP,
    CORAL_L4_RELEASE,

    CLIMING_APPROACH,
    CLIMBING_HANG,
    CLIMBING_LOCK;

    @Setter private static boolean isReversed = false;
    @Setter private static boolean isLeft = false;

    private State() {}

    // Define the scoring sequence map, the 2nd state is the next state after the current one
    private static final ImmutableMap<State, State> scoreSequence =
            ImmutableMap.ofEntries(
                    Map.entry(CORAL_L1_READY, CORAL_L1_PREP),
                    Map.entry(CORAL_L2_READY, CORAL_L2_PREP),
                    Map.entry(CORAL_L3_READY, CORAL_L3_PREP),
                    Map.entry(CORAL_L4_READY, CORAL_L4_PREP),
                    Map.entry(CORAL_L1_PREP, CORAL_L1_RELEASE),
                    Map.entry(CORAL_L2_PREP, CORAL_L2_RELEASE),
                    Map.entry(CORAL_L3_PREP, CORAL_L3_RELEASE),
                    Map.entry(CORAL_L4_PREP, CORAL_L4_RELEASE),
                    Map.entry(CORAL_L1_RELEASE, IDLE_CORAL),
                    Map.entry(CORAL_L2_RELEASE, IDLE_CORAL),
                    Map.entry(CORAL_L3_RELEASE, IDLE_CORAL),
                    Map.entry(CORAL_L4_RELEASE, IDLE_CORAL),
                    Map.entry(ALGAE_NET_RELEASE, IDLE_ALGAE));

    // ------ STATE ATTRIBUTES ------//
    public State reversed() {
        setReversed(true);
        return this;
    }

    public State forward() {
        setReversed(false);
        return this;
    }

    public State left() {
        setLeft(true);
        Telemetry.print("Set scoring side to LEFT");
        return this;
    }

    public State right() {
        setLeft(false);
        Telemetry.print("Set scoring side to RIGHT");
        return this;
    }

    public static boolean isReversed() {
        return isReversed;
    }

    public static boolean isLeft() {
        return isLeft;
    }

    private static BooleanSupplier isReadyState(State state) {
        return () ->
                switch (state) {
                    case CORAL_L4_READY, CORAL_L3_READY, CORAL_L2_READY, CORAL_L1_READY -> true;
                    default -> false;
                };
    }

    public BooleanSupplier isReady() {
        return isReadyState(this);
    }

    private static boolean isSpecialModeState(State state) {
        return switch (state) {
            case CLIMING_APPROACH, CLIMBING_HANG, CLIMBING_LOCK -> true;
            default -> false;
        };
    }

    public boolean isSpecialMode() {
        return isSpecialModeState(this);
    }

    private static boolean isAlgaeState(State state) {
        return switch (state) {
            case IDLE_ALGAE,
                    ALGAE_INTAKE_FLOOR,
                    ALGAE_INTAKE_L2,
                    ALGAE_INTAKE_L3,
                    ALGAE_NET_READY,
                    ALGAE_NET_RELEASE -> true;
            default -> false;
        };
    }

    public boolean isAlgae() {
        return isAlgaeState(this);
    }

    private static boolean isCoralState(State state) {
        return switch (state) {
            case IDLE_CORAL,
                    CORAL_INTAKE_FLOOR,
                    CORAL_L2_READY,
                    CORAL_L2_PREP,
                    CORAL_L2_RELEASE,
                    CORAL_L3_READY,
                    CORAL_L3_PREP,
                    CORAL_L3_RELEASE,
                    CORAL_L4_READY,
                    CORAL_L4_PREP,
                    CORAL_L4_RELEASE -> true;
            default -> false;
        };
    }

    public boolean isCoral() {
        return isCoralState(this);
    }

    private static boolean isIntakeState(State state) {
        return switch (state) {
            case ALGAE_INTAKE_FLOOR, ALGAE_INTAKE_L2, ALGAE_INTAKE_L3, CORAL_INTAKE_FLOOR -> true;
            default -> false;
        };
    }

    public boolean isIntake() {
        return isIntakeState(this);
    }

    public State getNextScoreState() {
        return scoreSequence.getOrDefault(this, this);
    }

    public State getNextState(State currentState) {
        State nextState = this; // Default to the current state

        return switch (currentState) {
            case CORAL_L2_READY, CORAL_L3_READY, CORAL_L4_READY -> {
                nextState = currentState.getNextScoreState();
                yield nextState;
            }

            case CORAL_L2_PREP, CORAL_L3_PREP, CORAL_L4_PREP -> {
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

            case ALGAE_NET_READY -> {
                nextState = State.ALGAE_NET_RELEASE;
                yield nextState;
            }

            default -> nextState;
        };
    }

    public State getNext() {
        return getNextState(this);
    }
}
