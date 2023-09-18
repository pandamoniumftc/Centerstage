package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CurvesPort.Curve;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;
import org.opencv.core.Point;

public class GamepadSettings {

    public volatile double left_stick_x, left_stick_y, right_stick_x, right_stick_y;

    public double leftStickDeadzone, rightStickDeadzone;

    public GamepadSettings(Gamepad gamepad, double leftStickDeadzone, double rightStickDeadzone, Point[] CurveProfile) {

        this.leftStickDeadzone = leftStickDeadzone;
        this.rightStickDeadzone = rightStickDeadzone;

        VariantDegreeBezier variantDegreeBezierCurve = new VariantDegreeBezier(CurveProfile);
        Curve[] Curve = new Curve[]{variantDegreeBezierCurve};
        CurveSequence sequence = new CurveSequence(Curve);

        left_stick_x = sequence.evaluate((gamepad.left_stick_x - (variantDegreeBezierCurve.minX)) / (variantDegreeBezierCurve.maxX - variantDegreeBezierCurve.minX));
        left_stick_y = sequence.evaluate((gamepad.left_stick_y - (variantDegreeBezierCurve.minX)) / (variantDegreeBezierCurve.maxX - variantDegreeBezierCurve.minX));
        right_stick_x = sequence.evaluate((gamepad.right_stick_x - (variantDegreeBezierCurve.minX)) / (variantDegreeBezierCurve.maxX - variantDegreeBezierCurve.minX));
        right_stick_y = sequence.evaluate((gamepad.right_stick_y - (variantDegreeBezierCurve.minX)) / (variantDegreeBezierCurve.maxX - variantDegreeBezierCurve.minX));

    }

}
