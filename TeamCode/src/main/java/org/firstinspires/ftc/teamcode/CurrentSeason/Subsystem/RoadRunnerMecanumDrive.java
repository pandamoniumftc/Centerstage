package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
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
    public CurveSequence MovementProfile;
    Toggle isFieldCentric = new Toggle(true);
    Toggle acceleration = new Toggle(true);

    public RoadRunnerMecanumDrive(AbstractRobot robot, Point[] MovementCurve) {
        super(robot);
        this.robot = (Po) robot;

        this.MovementProfile = CurveSequence.init(MovementCurve);
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
        double speedMultiplier = (1 - robot.gamepad1.right_trigger) * 0.75 + 0.25;

        isFieldCentric.updateState(robot.gamepad1.left_stick_button);
        acceleration.updateState(robot.gamepad1.right_stick_button);

        Vector2d input;

        if (isFieldCentric.state) {
            input = new Vector2d(
                    -robot.gamepad1.left_stick_y,
                    -robot.gamepad1.left_stick_x
            ).rotated(-drive.getPoseEstimate().getHeading()); //field centric
        }
        else {
            input = new Vector2d(
                    -robot.gamepad1.left_stick_y,
                    -robot.gamepad1.left_stick_x
            );
        }

        if (acceleration.state) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            this.MovementProfile.evaluate(Math.abs(Range.clip(input.getX(), -1.0, 1.0))) * Math.signum(input.getX()) * speedMultiplier,
                            this.MovementProfile.evaluate(Math.abs(Range.clip(input.getY(), -1.0, 1.0))) * Math.signum(input.getY())  * speedMultiplier,
                            this.MovementProfile.evaluate(Math.abs(Range.clip(-robot.gamepad1.right_stick_x, -1.0, 1.0))) * Math.signum(-robot.gamepad1.right_stick_x) * speedMultiplier
                    )
            );
        }
        else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -input.getX() * speedMultiplier,
                            -input.getY() * speedMultiplier,
                            -robot.gamepad1.right_stick_x * speedMultiplier)
            );
        }

        drive.update();

        Pose2d currentPos = drive.getPoseEstimate();

        telemetry.addData("x: ", currentPos.getX());
        telemetry.addData("y: ", currentPos.getY());
        telemetry.addData("c: ", currentPos.getHeading());
        telemetry.addData("left: ", drive.leftRear.getCurrentPosition());
        telemetry.addData("right: ", drive.rightFront.getCurrentPosition());
        telemetry.addData("front: ", drive.leftFront.getCurrentPosition());
    }

    @Override
    public void stop() {

    }
}
