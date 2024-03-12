package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    public double Kp, Ki, Kd, Kf, Kv, Ka;
    public double error, integral, derivative, dt, prevError, velocity, acceleration;
    public long prevTime;

    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;

        error = 0;
        integral = 0;
        prevError = 0;
        prevTime = 0;
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double Kv, double Ka) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Kv = Kv;
        this.Ka = Ka;

        error = 0;
        integral = 0;
        prevError = 0;
        prevTime = 0;
    }

    public double calculate(double current, double target, double feedForward) {
        prevError = error;

        dt = (System.nanoTime() - prevTime) * 1E-9;
        prevTime = System.nanoTime();

        error = target - current;

        integral += (error * dt);
        integral = Range.clip(integral, -1, 1);

        derivative = (error - prevError) / dt;

        return error * Kp + integral * Ki + derivative * Kd + feedForward * Kf;
    }

    public double calculate(double current, double target, double feedForward, double velocity, double acceleration) {
        prevError = error;

        dt = (System.nanoTime() - prevTime) * 1E-9;
        prevTime = System.nanoTime();

        error = target - current;

        integral += (error * dt);
        integral = Range.clip(integral, -1, 1);

        derivative = (error - prevError) / dt;

        return error * Kp + integral * Ki + derivative * Kd + feedForward * Kf + velocity * Kv + acceleration * Ka;
    }
}
