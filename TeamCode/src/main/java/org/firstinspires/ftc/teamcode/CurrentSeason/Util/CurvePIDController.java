package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;

public class CurvePIDController {
    public VariantDegreeBezier Cp, Ci, Cd;

    private double target;
    private double angle;
    public double error;

    public double P;
    public double I;
    public double D;

    private double integral;
    private double prevError;

    private long prevTime;

    private boolean firstFrame = true;

    public CurvePIDController(VariantDegreeBezier Cp, VariantDegreeBezier Ci, VariantDegreeBezier Cd) {
        this.Cp = Cp;
        this.Cd = Cd;
        this.Ci = Ci;
    }

    public double PIDOutput(double target, double current) throws Exception {
        //if (target < -Math.PI*2 || target > Math.PI*2) throw new Exception("target angle must be between positive and negative 2PI");

        this.target = target;

        angle = current;
        error = target - angle;

        double dt = (firstFrame) ? 0 : (System.nanoTime() - prevTime) * 1E-9;

        integral += (error) * dt;

        P = PIDEvaluateCurve(Cp, error);
        I = PIDEvaluateCurve(Ci, integral);
        D = (firstFrame) ? 0 : PIDEvaluateCurve(Cd, ( prevError - error) / dt);
        //D = 0;
        double PID = P + I + D;


        prevError = error;
        prevTime = System.nanoTime();

        firstFrame = false;

        return PID;
    }

    private double PIDEvaluateCurve(VariantDegreeBezier curve, double input) {
        int sign = (int)Math.signum(input);

        input = Range.clip(abs(input), curve.minX, curve.maxX);

        return sign * curve.evaluate(input);


    }
}