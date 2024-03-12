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
    VariantDegreeBezier MovementProfile;
    public Toggle resetAngle, aligning;

    public RoadRunnerMecanumDrive(AbstractRobot robot, Point[] MovementPoints) {
        super(robot);
        this.robot = (Po) robot;

        resetAngle = new Toggle(false);
        aligning = new Toggle(false);

        this.MovementProfile = new VariantDegreeBezier(MovementPoints);
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
        aligning.updateState(robot.gamepad1.left_bumper);

        Vector2d input = new Vector2d(
                -robot.gamepad1.left_stick_y,
                -robot.gamepad1.left_stick_x
        ).rotated(-drive.getPoseEstimate().getHeading()); //field centric

        if (aligning.state && robot.state == Po.robotState.INTAKING) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            this.MovementProfile.evaluate(Math.abs(Range.clip(input.getX(), -1.0, 1.0)))[0] * Math.signum(input.getX()) * (robot.gamepad1.right_trigger * 0.5 + (1 - robot.gamepad1.right_trigger)),
                            this.MovementProfile.evaluate(Math.abs(Range.clip(input.getY(), -1.0, 1.0)))[0] * Math.signum(input.getY()) * (robot.gamepad1.right_trigger * 0.5 + (1 - robot.gamepad1.right_trigger)),
                            0
                    )
            );
        } else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            this.MovementProfile.evaluate(Math.abs(Range.clip(input.getX(), -1.0, 1.0)))[0] * Math.signum(input.getX()) * robot.slides.strength,
                            this.MovementProfile.evaluate(Math.abs(Range.clip(input.getY(), -1.0, 1.0)))[0] * Math.signum(input.getY()) * robot.slides.strength,
                            this.MovementProfile.evaluate(Math.abs(Range.clip(-robot.gamepad1.right_stick_x, -1.0, 1.0)))[0] * Math.signum(-robot.gamepad1.right_stick_x) * robot.slides.strength
                    )
            );
        }

        drive.update();

        if (resetAngle.state) {
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            resetAngle.state = false;
        }

        telemetry.addData("CURRENT POS: ", drive.getPoseEstimate().getX() + " " + drive.getPoseEstimate().getY() + " " + drive.getPoseEstimate().getHeading());
    }

    public void align(double error, double kp) {
        robot.mecanum.drive.turn(error * kp);
    }

    @Override
    public void stop() {

    }
}