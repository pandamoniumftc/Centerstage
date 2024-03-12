package org.firstinspires.ftc.teamcode.CurvesPort;

import static java.lang.Math.pow;

import org.opencv.core.Point;

public class LinearBezierCurve extends Curve {
    public Point[] points;
    public double minX;
    public double maxX;
    public LinearBezierCurve(Point[] points) throws ArrayIndexOutOfBoundsException {
        if (points.length != 2) {
            throw new ArrayIndexOutOfBoundsException("curve does not have a length of 2");
        }

        this.points = points;
        this.endpoint = points[1];

        super.minX = points[0].x;
        super.maxX = points[1].x;

        for (int i = 1; i < points.length; i++) {
            super.minX = Math.min(super.minX, points[i].x);
            super.maxX = Math.max(super.maxX, points[i].x);
        }

        this.minX = super.minX;
        this.maxX = super.maxX;

        System.out.println("bezier min: " + super.minX + ", bezier max: " + super.maxX);
    }
    public double[] evaluate(double t) {

        /** evaluates the curve given t
         * @param t The distance along a curve (double)
         * @return out an array consisting of the y value and the derivative of t
         */

        double[] out = new double[3];

        //y given t (position)
        out[0] = ((1 - t) * points[0].y) +
                (t * points[1].y);

        //derivative of t (velocity)
        out[1] = (points[1].x > points[0].x ? points[1].y - points[0].y : points[0].y - points[1].y)/(maxX - minX);

        //derived x2 (accel)
        out[2] = 0;

        return out;
    }
}
