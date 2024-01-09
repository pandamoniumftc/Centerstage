package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

public class PIDFController {
    public double Kp, Ki, Kd, Kf;

    public double P, I, D, F;

    public double integral;
    public boolean firstFrame = true;

    public double prevTime;
    public double prevError;

    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public double calculate(double current, double target, double feedForward){

        double error = target - current;
        double dt = firstFrame ? 0 : (System.nanoTime() - prevTime) * 1E-9;

        P = error * Kp;

        I = (integral += error) * dt * Ki;

        D = firstFrame ? 0 : (error - prevError)/dt;

        F = feedForward * Kf;

        prevError = error;
        prevTime = System.nanoTime();
        firstFrame = false;

        return P + I + D + F;
    }

}
