package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import  com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp (name="pidf tuner")
public class PIDFTuner extends OpMode {
    Po robot;
    PIDFController controller;
    public static double kp = 1, ki = 0.0075, kd = 0.75, kf = 0.002;
    public static int target = 0;
    double encoderResolution = 537.7;
    double gearRatio = 3;

    @Override
    public void init() {
        this.robot = new Po(this);
        controller = new PIDFController(kp, ki, kd, kf);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        //int lSlidesPos = robot.slides.lSlideMotor.getCurrentPosition();
        //int rSlidesPos = robot.slides.rSlideMotor.getCurrentPosition();
        //double Lpidf = controller.calculate(lSlidesPos, target, ff);
        //double Rpidf = controller.calculate(rSlidesPos, target, ff);

        double armAngle = ((((double) robot.arm.armMotor.getCurrentPosition() - 85) / encoderResolution) * 2 * Math.PI) / gearRatio;
        double pid = controller.calculate(robot.arm.armMotor.getCurrentPosition(), target, robot.arm.armAngleFeedforward(armAngle));
        robot.arm.armMotor.setPower(robot.arm.ArmProfile.evaluate(Math.abs(Range.clip(pid, -1, 1))) * Math.signum(pid));

        //robot.slides.lSlideMotor.setPower(power);
        //robot.slides.rSlideMotor.setPower(power);

        //telemetry.addData("left pos: ", lSlidesPos);
        //telemetry.addData("right pos: ", rSlidesPos);
        //telemetry.addData("left: ", Lpidf);
        //telemetry.addData("right: ", Rpidf);
        telemetry.addData("pos: ", robot.arm.armMotor.getCurrentPosition());
        telemetry.addData("angle: ", armAngle);
        telemetry.addData("target: ", target);
        telemetry.addData("pid: ", robot.arm.armMotor.getPower());
        telemetry.update();
        //telemetry.addData("left dir", robot.slides.lSlideMotor.getDirection());
        //telemetry.addData("right dir", robot.slides.rSlideMotor.getDirection());
    }
}