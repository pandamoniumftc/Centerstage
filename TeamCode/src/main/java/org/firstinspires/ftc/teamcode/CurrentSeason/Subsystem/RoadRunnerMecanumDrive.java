package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.CurvesPort.Curve;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

import java.io.IOException;

public class RoadRunnerMecanumDrive extends AbstractSubsystem {
    public SampleMecanumDrive drive;
    Point[] AccelerationProfile = new Point[] {
            new Point(-1, -1),
            new Point(0, 0.9),
            new Point(0, -0.9),
            new Point(1, 1)
    };
    VariantDegreeBezier variantDegreeBezierCurve = new VariantDegreeBezier(AccelerationProfile);
    Curve[] Curve = new Curve[]{variantDegreeBezierCurve};
    public CurveSequence sequence = new CurveSequence(Curve);

    /*
    flm = control hub, port 3
    frm = expansion hub, port 0
    blm = control hub, port 2
    brm = expansion hub, port 1
     */

    public RoadRunnerMecanumDrive(AbstractRobot robot) {
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
                //sequence.evaluate((robot.gamepad1.left_stick_y - (variantDegreeBezierCurve.minX)) / (variantDegreeBezierCurve.maxX - variantDegreeBezierCurve.minX))
                robot.gamepad1.left_stick_x,
                //sequence.evaluate((robot.gamepad1.left_stick_x - (variantDegreeBezierCurve.minX)) / (variantDegreeBezierCurve.maxX - variantDegreeBezierCurve.minX))
                robot.gamepad1.left_stick_y
        );//.rotated(drive.getPoseEstimate().getHeading()); //field centric

        double speedMultiplier = (1 - robot.gamepad1.right_trigger) * 0.75 + 0.25;

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * speedMultiplier,
                        input.getY() * speedMultiplier,
                        //sequence.evaluate((robot.gamepad1.right_stick_x - (variantDegreeBezierCurve.minX)) / (variantDegreeBezierCurve.maxX - variantDegreeBezierCurve.minX)) * speedMultiplier
                        -robot.gamepad1.right_stick_x * speedMultiplier
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
