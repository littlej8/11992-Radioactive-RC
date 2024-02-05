package org.firstinspires.ftc.teamcode.Util;

/**
 *    Concept for autonomous structure
 */
public class Environment {
    public static class Auto {
        public static final double WHEEL_POWER = 0.2;
        public static final double TURN_POWER = 0.1;

        public static final double PID_KP = 10.0;
        public static final double PID_KI = 3.0;
        public static final double PID_KD = 0.0;
        public static final double PID_TOLERANCE = 1.0;

        public static final double ODOMETRY_TOLERANCE = 0.5;

        public static final double COUNTS_PER_REVOLUTION = 145.1;
        public static final double WHEEL_DIAMETER_INCHES = 3.0;
        public static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * 3.1415);
        public static final double COUNTS_PER_DEGREE = COUNTS_PER_REVOLUTION / 360;

        public static final double MAX_ACCELERATION = 0.1 * COUNTS_PER_INCH;
        public static final double MAX_VELOCITY = 1.0 * COUNTS_PER_INCH;

        public static final double MAX_TURN_ACCELERATION = 1.0;
        public static final double MAX_TURN_VELOCITY = 6.0;

        public static class MovementInches {
            public static final double MOVE_UP = 25.0;
            public static final double MOVE_BACK = 23.0;
            public static final double PARK_CORNER_CLOSE = 48.0;
            public static final double PARK_CORNER_FAR = 100.0;
            public static final double MIDDLE_PARK_1_CLOSE = 40.0;
            public static final double MIDDLE_PARK_1_FAR = 84.0;
            public static final double MIDDLE_PARK_2 = 48.0;
            public static final double MIDDLE_PARK_3 = 18.0;

            public static final double DEG90 = 38.0;
        }

        public static class MovementCounts {
            public static final double MOVE_UP = 290;

            public static final double LEFT_ADJUST = 0.0;
            public static final double RIGHT_ADJUST = 0.0;
            public static final double FRONT_ADJUST_1 = -25;
            public static final double FRONT_ADJUST_2 = 0;

            public static final double MOVE_BACK_STRAIGHT = 265;
            public static final double MOVE_BACK_RIGHT = 375;
            public static final double MOVE_BACK_LEFT = 375;

            public static final double PARK_CORNER_CLOSE_STRAIGHT = 500;
            public static final double PARK_CORNER_FAR_STRAIGHT = 1200;
            public static final double PARK_CORNER_CLOSE_STRAFE = 525;
            public static final double PARK_CORNER_FAR_STRAFE = 1600;

            public static final double DEG90 = 350.0;
        }

        public static final int LIGHT_SENSITIVITY = 30;
        public static final int LEFT_BOUND = 145;
        public static final int RIGHT_BOUND = 170;
        
        public static final double TURN_TOLERANCE = 5.0;
    }

    public static class TeleOp {
        public static final double WHEEL_POWER = 0.5;
        public static final double ARM_POWER = 1;
        public static final double LIFT_POWER = 1;

        public static final double CLAW_WRIST_SPEED = 0.0025;
        public static final double CLAW_GRABBER_SPEED = 0.005;
        public static final double CLAW_WRIST_MIN = 0.2;
        public static final double CLAW_WRIST_MAX = 0.75;
        public static final double CLAW_GRABBER_MIN = 0.25;
        public static final double CLAW_GRABBER_MAX = 0.75;
    }
}