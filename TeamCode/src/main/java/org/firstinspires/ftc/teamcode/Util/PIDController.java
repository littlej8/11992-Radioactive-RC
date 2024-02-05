package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final double Kp;
    private final double Ki;
    private final double Kd;

    private final double MaxAcceleration;
    private final double MaxVelocity;

    private final TrapezoidMotionProfile MotionProfile;

    private double lastError;
    private double integralSum;

    private final ElapsedTime timer;

    public PIDController(double kp, double ki, double kd, double max_acceleration, double max_velocity) {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        MaxAcceleration = max_acceleration;
        MaxVelocity = max_velocity;

        MotionProfile = new TrapezoidMotionProfile(max_acceleration, max_velocity);

        timer = new ElapsedTime();
        lastError = 0;
        integralSum = 0;
    }

    public void reset() {
        lastError = 0;
        integralSum = 0;

        timer.reset();
    }

    public double update(double target, double cur_state) {
        double state = MotionProfile.get_position(target - cur_state, timer.seconds());

        double error = target - state;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (error * timer.seconds());

        lastError = error;

        return out;
    }
}