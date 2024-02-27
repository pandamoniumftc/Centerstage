package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.PixelPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.CurvesPort.Curve;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.opencv.core.Point;

public class RoadRunnerMecanumDrive extends AbstractSubsystem {
    Po robot;
    public SampleMecanumDrive drive;
    Toggle resetAngle = new Toggle(false);

    public RoadRunnerMecanumDrive(AbstractRobot robot, Point[] MovementCurve) {
        super(robot);
        this.robot = (Po) robot;

        //this.MovementProfile = CurveSequence.init(MovementCurve);
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
        resetAngle.updateState(robot.gamepad1.right_bumper);

        Vector2d input = new Vector2d(
                -robot.gamepad1.left_stick_y,
                -robot.gamepad1.left_stick_x
        ).rotated(-drive.getPoseEstimate().getHeading()); //field centric

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -robot.gamepad1.right_stick_x
                )
        );

        drive.update();

        if (resetAngle.state) {
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            resetAngle.state = false;
        }

        telemetry.addData("x: ", drive.getPoseEstimate().getX());
        telemetry.addData("y: ", drive.getPoseEstimate().getY());
        telemetry.addData("c: ", drive.getPoseEstimate().getHeading());
        //telemetry.addData("left: ", drive.leftRear.getCurrentPosition());
        //telemetry.addData("right: ", drive.rightFront.getCurrentPosition());
        //telemetry.addData("front: ", drive.leftFront.getCurrentPosition());
    }

    @Override
    public void stop() {

    }
}