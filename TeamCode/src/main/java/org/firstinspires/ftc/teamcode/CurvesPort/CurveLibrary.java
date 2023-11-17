package org.firstinspires.ftc.teamcode.CurvesPort;

import org.opencv.core.Point;

public class CurveLibrary {
    private final Point[] AccelerationPoints = new Point[] {
            new Point(-1, -1),
            new Point(0, 0.9),
            new Point(0, -0.9),
            new Point(1, 1)
    };
    private final Point[] ExponentialPoints = new Point[] {
            new Point(0, 0),
            new Point(.5, 0),
            new Point(1, 0),
            new Point(1, 1)
    };

    private final Point[] LinearPoints = new Point[] {
            new Point(0, 0),
            new Point(0.2175, .335),
            new Point(0.66, .6475),
            new Point(1, 1)
    };

    public CurveSequence AccelerationProfile, ExponentialProfile, LinearProfile;
    public CurveLibrary () {

        Point[][] points = new Point[][]{
                AccelerationPoints,
                ExponentialPoints,
                LinearPoints
        };

        CurveSequence[] sequence = new CurveSequence[] {
                AccelerationProfile,
                ExponentialProfile,
                LinearProfile
        };

        for (Point[] point : points) {

            VariantDegreeBezier vdbc = new VariantDegreeBezier(point);

            Curve[] curve = new Curve[]{vdbc};

            for (int i = 0; i < sequence.length; i++) {
                sequence[i] = new CurveSequence(curve);
            }

        }

    }

}
