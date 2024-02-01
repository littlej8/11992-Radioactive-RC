package org.firstinspires.ftc.teamcode;

/**
 *    Concept for autonomous structure
 */
public class Environment {
    public class Auto {
        static final double WHEEL_POWER = 0.2;

        static final double PID_KP = 2.0;
        static final double PID_KI = 0.2;
        static final double PID_KD = 0.3;
        static final double PID_TOLERANCE = 2.5;

        static final double MOVE_UP =  24.0;
        static final double MOVE_BACK =  20.0;
        static final double PARK_CORNER_CLOSE =  48.0;
        static final double PARK_CORNER_FAR =  84.0;
        static final double MIDDLE_PARK_1_CLOSE =  40.0;
        static final double MIDDLE_PARK_1_FAR =  76.0;
        static final double MIDDLE_PARK_2 =  48.0;
        static final double MIDDLE_PARK_3 =  18.0;

        static final double deg90 = 440.0;
    }

    public class TeleOp {
        static final double WHEEL_POWER = 0.5;
        static final double ARM_POWER = 1;
        static final double LIFT_POWER = 1;

        static final double CLAW_WRIST_SPEED = 0.04;
        static final double CLAW_GRABBER_SPEED = 0.04;
        static final double CLAW_WRIST_MIN = 0.2;
        static final double CLAW_WRIST_MAX = 0.75;
        static final double CLAW_GRABBER_MIN = 0.25;
        static final double CLAW_GRABBER_MAX = 0.75;
    }
}