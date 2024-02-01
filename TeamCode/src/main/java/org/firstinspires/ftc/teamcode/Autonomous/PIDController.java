package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double lastError;
    private double integralSum;

    private ElapsedTime timer;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

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