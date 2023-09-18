package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.IOException;

public class RoadRunnerMecanumDrive extends AbstractSubsystem {
    BaoBao robot;
    SampleMecanumDrive drive;

    public RoadRunnerMecanumDrive(AbstractRobot robot) {
        super(robot);
        this.robot = (BaoBao) robot;
    }

    @Override
    public void init() throws IOException {
        drive = new SampleMecanumDrive(robot.hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {

        Vector2d input = new Vector2d(
                robot.Gamepad1.left_stick_y,
                robot.Gamepad1.left_stick_x
        ).rotated(drive.getPoseEstimate().getHeading()); //field centric

        double speedMultiplier = (1 - robot.gamepad1.right_trigger) * 0.75 + 0.25;

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getY() * speedMultiplier,
                        input.getX() * speedMultiplier,
                        robot.Gamepad1.right_stick_x * speedMultiplier
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
