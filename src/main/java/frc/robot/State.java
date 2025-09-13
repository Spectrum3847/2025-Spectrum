package frc.robot;

import lombok.Getter;

public enum State {
    REHOME(Claw.EMPTY, true),

    IDLE_EMPTY(Claw.EMPTY, false),
    IDLE_ALGAE(Claw.ALGAE, false),
    IDLE_CORAL(Claw.CORAL, false),
    STARTING_POS(Claw.EMPTY, false),
    STARTING_POS_CORAL(Claw.CORAL, false),

    ALGAE_INTAKE_FLOOR(Claw.ALGAE, false),
    ALGAE_INTAKE_L2_LEFT(Claw.ALGAE, false),
    ALGAE_INTAKE_L2_RIGHT(Claw.ALGAE, false),
    ALGAE_INTAKE_L3_LEFT(Claw.ALGAE, false),
    ALGAE_INTAKE_L3_RIGHT(Claw.ALGAE, false),

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
}
