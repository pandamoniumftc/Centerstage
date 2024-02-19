package org.firstinspires.ftc.teamcode.CurvesPort;

import static java.lang.Math.pow;

import org.opencv.core.Point;

public class VariantDegreeBezier extends Curve {

    /**
     * Must be length of 4
     */
    public Point[] points;

    public double minX;
    public double maxX;

    public VariantDegreeBezier(Point[] points) throws ArrayIndexOutOfBoundsException {
        if (points.length != 4) {
            throw new ArrayIndexOutOfBoundsException("curve does not have a length of 4");
        }

        this.points = points;
        this.endpoint = points[3];

        super.minX = points[0].x;
        super.maxX = points[0].x;

        for (int i = 1; i < points.length; i++) {
            super.minX = Math.min(super.minX, points[i].x);
            super.maxX = Math.max(super.maxX, points[i].x);
        }

        this.minX = super.minX;
        this.maxX = super.maxX;

        System.out.println("bezier min: " + super.minX + ", bezier max: " + super.maxX);
    }

    public double[] evaluate(double t) {
        //pos:   f(x) = (1-t)^3 * p0 + (3t(1-t)^2 * p1) + (3(1-t)t^2 * p2 + (t^3 * p3)
        //velo:  f'(x) = -3(1-t)^2 * p0 + (3-12t + 9x^2) * p1 + (6x-9x^2) * p2 + (3x^2) * p3
        //accel: f''(x) = 6 * (1-t) * p0 + (-12 + 18t) * p1 + (6-18t) * p2 + 6t * p3

        /** evaluates the curve given t
         * @param t The distance along a curve (double)
         * @return out an array consisting of the y value and the derivative of t
         */

        double[] out = new double[3];

        //y given t (position)
        out[0] = (pow(1 - t, 3) * points[0].y) +
                (3 * t * pow(1 - t, 2) * points[1].y) +
                (3 * (1 - t) * pow(t, 2) * points[2].y) +
                (pow(t, 3) * points[3].y);

        //derivative of t (velocity)
        out[1] = (-3 * pow(1-t, 2)) * points[0].y +
                ((3 - 12 * t) + 9 * pow(t, 2)) * points[1].y +
                (6 * t - 9 * pow(t, 2)) * points[2].y +
                (3 * pow(t, 2)) * points[3].y;

        //derived x2 (accel)
        out[2] = 6 * (1 - t) * points[0].y +
                (-12 + 18 * t) * points[1].y +
                (6 - 18 * t) * points[2].y +
                (6 * t * points[3].y);

        return out;
    }

}
