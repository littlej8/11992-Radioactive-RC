package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double lastError;
    private double integralSum;

    private final ElapsedTime timer;

    public PIDController(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        timer = new ElapsedTime();
        lastError = 0;
        integralSum = 0;
    }

    public void reset() {
        lastError = 0;
        integralSum = 0;

        timer.reset();
    }

    public double update(double target, double state) {
        double error = target - state;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (error * timer.seconds());

        lastError = error;

        return out;
    }
}