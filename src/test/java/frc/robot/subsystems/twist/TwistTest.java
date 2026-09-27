package frc.robot.subsystems.twist;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TwistTest {
    @Test
    void goesStraightThereWhenClose() {
        assertEquals(100, Twist.route(100, 80, true, 45), 1e-9);
        assertEquals(360 + 100, Twist.route(100, 360 + 120, false, 45), 1e-9);
    }

    @Test
    void crossesTheSeamTheShortWay() {
        // 5 -> 355 is 10 degrees back, not 350 forward (the 2025 code went the wrong way here).
        assertEquals(-5, Twist.route(355, 5, true, 45), 1e-9);
        assertEquals(365, Twist.route(5, 355, true, 45), 1e-9);
    }

    @Test
    void goesTheLongWayInTheChosenDirection() {
        assertEquals(-90, Twist.route(270, 90, true, 45), 1e-9);
        assertEquals(270, Twist.route(270, 90, false, 45), 1e-9);
    }

    @Test
    void neverRoutesThroughTheElevator() {
        // Left branch (90) to right branch (270) on the front: 180 is where the elevator is, so
        // the claw must go down through 0 instead.
        boolean clockwise = Twist.clockwiseAwayFromElevator(270, 90, false);
        assertTrue(clockwise);
        assertEquals(-90, Twist.route(270, 90, clockwise, 45), 1e-9);
    }

    @Test
    void mirrorsForTheRear() {
        assertEquals(270, Twist.mirrorForRear(90), 1e-9);
        assertEquals(-0.1, Twist.mirrorForRear(179.9), 1e-9);
        assertEquals(180, Twist.mirrorForRear(0), 1e-9);
    }
}
