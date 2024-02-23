package org.firstinspires.ftc.teamcode.CurvesPort;

import org.opencv.core.Point;


public abstract class Curve {
    public Point[] points;

    public double minX;
    public double maxX;

    public double scaleFactor;
    public Point endpoint;

    /**
     * @return the output of the curve at a value (t) between 0 and 1
     */
    static double[] evaluate(double t) {
        return new double[0];
    }
}
