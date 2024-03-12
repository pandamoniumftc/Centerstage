package org.firstinspires.ftc.teamcode.CurvesPort;

import org.opencv.core.Point;

public class CurveSequence {

    public Curve[] curves;
    public double length;
    public double minX;
    public double maxX;

    public CurveSequence(Curve[] curves) {

        this.curves = curves;

        this.minX = this.curves[0].minX;
        this.maxX = this.curves[0].maxX;

        for (int i = 1; i < curves.length; i++) {
            this.minX = Math.min(this.minX, this.curves[i].minX);
            this.maxX = Math.max(this.maxX, this.curves[i].maxX);
            System.out.println(curves[i].minX);
        }

        this.length = this.maxX - this.minX;
        System.out.println("minX: " + this.minX + ", maxX: " + this.maxX + ", length: " + this.length);
    }

    public static CurveSequence init(Point[][] points) {
        Curve[] curve = new Curve[points.length];
        for (int i = 0; i < points.length; i++) {
            if (points[i].length == 4) {
                VariantDegreeBezier vdbc = new VariantDegreeBezier(points[i]);
                curve[i] = vdbc;
            }
            else if (points[i].length == 2) {
                LinearBezierCurve lbc = new LinearBezierCurve(points[i]);
                curve[i] = lbc;
            }
        }
        return new CurveSequence(curve);
    }

    public double[] evaluate(double t) {
        for (Curve curve : curves) {
            double minT = (curve.minX - minX)/length;
            double maxT = (curve.maxX - minX)/length;

            if (t >= minT && t <= maxT) {
                double tx = this.minX + (this.maxX - this.minX) * t;
                System.out.println("at time = " + t + ", x = " + (this.minX + (this.maxX - this.minX) * t));
                System.out.println((tx - curve.minX) / (curve.maxX - curve.minX));
                return curve.evaluate((tx - curve.minX) / (curve.maxX - curve.minX));
            }
        }

        throw new ArrayIndexOutOfBoundsException("could not locate a point at t = " + t + ", x = " + (this.minX + (this.maxX - this.minX) * t));
    }

}
