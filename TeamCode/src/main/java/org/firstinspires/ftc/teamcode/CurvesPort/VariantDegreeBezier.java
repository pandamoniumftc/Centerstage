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

    public double evaluate(double t) {
        double y = (pow(1 - t, 3) * points[0].y) +
                   (3 * t * pow(1 - t, 2) * points[1].y) +
                   (3 * (1 - t) * pow(t, 2) * points[2].y) +
                   (pow(t, 3) * points[3].y);
        return y;
    }

}
