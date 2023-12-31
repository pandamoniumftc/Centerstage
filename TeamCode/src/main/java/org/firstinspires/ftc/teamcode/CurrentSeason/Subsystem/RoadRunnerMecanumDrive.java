package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.CurvesPort.Curve;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

import java.io.IOException;
import java.text.DecimalFormat;

public class RoadRunnerMecanumDrive extends AbstractSubsystem {
    public SampleMecanumDrive drive;
    private final Point[] JumpPoints = new Point[] {
            new Point(0, 0),
            new Point(0, 1),
            new Point(1, 0),
            new Point(1, 1)
    };

    VariantDegreeBezier vdbc = new VariantDegreeBezier(JumpPoints);

    Curve[] curve = new Curve[]{vdbc};
    //public CurveSequence movementCurve, pivotCurve;
    public CurveSequence curveSequence = new CurveSequence(curve);

    /*
    flm = control hub, port 3
    frm = expansion hub, port 0
    blm = control hub, port 2
    brm = expansion hub, port 1
     */

    public RoadRunnerMecanumDrive(AbstractRobot robot/*CurveSequence movementCurve, CurveSequence pivotCurve*/) {
        super(robot);
    }

    @Override
    public void init() {
        drive = new SampleMecanumDrive(robot.hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {

        Vector2d input = new Vector2d(
                robot.gamepad1.left_stick_y,
                robot.gamepad1.left_stick_x
        ).rotated(-drive.getPoseEstimate().getHeading()); //field centric

        drive.setWeightedDrivePower(
                new Pose2d(
                        this.curveSequence.evaluate(Math.abs(Range.clip(input.getX(), -1, 1))) * Math.signum(input.getX()),
                        this.curveSequence.evaluate(Math.abs(Range.clip(input.getY(), -1, 1))) * Math.signum(input.getY()),
                        robot.gamepad1.right_stick_x//this.pivotCurve.evaluate((robot.gamepad1.right_stick_x - (pivotCurve.minX)) / (pivotCurve.maxX - pivotCurve.minX))
                )
        );

        drive.update();

        Pose2d currentPos = drive.getPoseEstimate();

        telemetry.addData("x: ", currentPos.getX());
        telemetry.addData("y: ", currentPos.getY());
        telemetry.addData("c: ", currentPos.getHeading());

    }

    @Override
    public void stop() {

    }
}
