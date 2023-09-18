package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.GamepadSettings;
import org.opencv.core.Point;

public class BaoBao extends AbstractRobot {
    public GamepadSettings Gamepad1, Gamepad2;

    Point[] AccelerationProfile = new Point[] {
            new Point(-1, -1),
            new Point(0, 0.9),
            new Point(0, -0.9),
            new Point(1, 1)
    };

    public BaoBao(OpMode opMode) {
        super(opMode);
        Gamepad1 = new GamepadSettings(this.gamepad1, 0.001, 0.001, AccelerationProfile);

    }
}
