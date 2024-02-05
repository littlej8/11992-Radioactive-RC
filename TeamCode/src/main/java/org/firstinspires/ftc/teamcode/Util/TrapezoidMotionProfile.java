package org.firstinspires.ftc.teamcode.Util;

public class TrapezoidMotionProfile {
    public final double MaxAccel;
    public final double MaxVel;

    public TrapezoidMotionProfile(double max_accel, double max_vel) {
        MaxAccel = max_accel;
        MaxVel = max_vel;
    }

    public double get_position(double distance, double elapsed_time) {
        double max_acceleration = MaxAccel;
        double max_velocity = MaxVel;

        double accel_dt = max_velocity / max_acceleration;

        double half_dist = distance / 2;
        double accel_dist = 0.5 * max_acceleration * accel_dt ** 2;

        if (accel_dist > half_dist) {
            accel_dt = Math.sqrt(half_dist / (0.5 * max_acceleration));
        }

        accel_dist = 0.5 * max_acceleration * accel_dt ** 2;

        max_velocity = max_acceleration * accel_dt;

        decel_dt = accel_dt;

        double cruise_dist = distance - 2 * accel_dist;
        double cruise_dt = cruise_dist / max_velocity;
        double decel_time = accel_dt + cruise_dt;

        // check if move is finished
        double entire_dt = accel_dt + cruise_dt + decel_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        if (elapsed_time < accel_dt) { // accelerating
            return 0.5 * max_accel * elapsed_time ** 2;
        } else if (elapsed_time < decel_dt) { // cruising
            accel_dist = 0.5 * max_accel * accel_dt ** 2;
            double cruise_current_dt = elapsed_time - accel_dt;

            return accel_dist + max_velocity * cruise_current_dt;
        } else { // decelerating
            accel_dist = 0.5 * max_accel * accel_dt ** 2;
            cruise_dist = max_velocity * cruise_dt;
            decel_time = elapsed_time - decel_time;

            return accel_dist + cruise_dist + max_velocity * decel_time - 0.5 * max_accel * decel_time ** 2;
        }
    }
}