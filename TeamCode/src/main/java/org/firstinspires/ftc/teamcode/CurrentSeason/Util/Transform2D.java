package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

public class Transform2D {
    public double x = 0;
    public double y = 0;

    public double heading;

    public Transform2D() {}

    public Transform2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Transform2D(Transform2D copy) {
        this.x = copy.x;
        this.y = copy.y;
        this.heading = copy.heading;
    }

}
