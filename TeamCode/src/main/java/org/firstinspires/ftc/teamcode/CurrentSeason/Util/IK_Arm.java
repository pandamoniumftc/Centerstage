package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import org.opencv.core.Point;

public class IK_Arm {

    public double x, y, angle, length;
    public IK_Arm child;

    public Point rotatePoint(Point p, double angle) {
        return new Point(p.x * Math.cos(angle) - p.y * Math.sin(angle), p.x * Math.sin(angle) + p.y * Math.cos(angle));
    }

    public Point translatePoint(Point p, double x, double y) {
        return new Point(p.x + x, p.y + y);
    }

    public double angle(Point p) {
        return Math.atan2(p.y, p.x);
    }

    public IK_Arm(double x, double y, double angle, double length, IK_Arm child) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.length = length;
        this.child = child;
    }

    public Point updatePoint(Point target) {
        Point localTarget = rotatePoint(translatePoint(target, this.x, this.y), this.angle);

        Point endPoint;
        if (this.child != null) {
            endPoint = this.child.updatePoint(localTarget);
        } else {
            endPoint = new Point(this.length, 0);
        }

        double shiftAngle = angle(localTarget) - angle(endPoint);
        this.angle += shiftAngle;

        return translatePoint(rotatePoint(endPoint, this.angle), this.x, this.y);
    }

    public double radiansToEncoder(double encoderResolution, double angle, double gearRatio) {
        return (angle / 2 * Math.PI) * encoderResolution * gearRatio;
    }

}
