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
    public static double kp = 0, ki = 0, kd = 0, kf = 0;
    public static int target = 0;
    public static double pos;
    double encoderResolution = 537.7;
    double gearRatio = 3;

    @Override
    public void init() {
        this.robot = new Po(this);
        controller = new PIDFController(kp, ki, kd, kf);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.slides.lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slides.rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        robot.slides.lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slides.rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        int lSlidesPos = robot.slides.lSlideMotor.getCurrentPosition();
        int rSlidesPos = robot.slides.rSlideMotor.getCurrentPosition();
        double Lpidf = controller.calculate(lSlidesPos, target, 1);
        double Rpidf = controller.calculate(lSlidesPos, target, 1);

        //double armAngle = ((((double) robot.arm.armMotor.getCurrentPosition() - 100) / encoderResolution) * 2 * Math.PI) / gearRatio;
        //double pid = controller.calculate(robot.arm.armMotor.getCurrentPosition(), target, robot.arm.armAngleFeedforward(armAngle));
        //robot.arm.armMotor.setPower(robot.arm.ArmProfile.evaluate(Math.abs(Range.clip(pid, -1, 1))) * Math.signum(pid));
        //robot.airplane.airplaneservo.setPosition(pos);
        robot.slides.lSlideMotor.setPower(Lpidf);
        robot.slides.rSlideMotor.setPower(Rpidf);

        //robot.slides.lSlideMotor.setPower(power);
        //robot.slides.rSlideMotor.setPower(power);

        telemetry.addData("left pos: ", lSlidesPos);
        telemetry.addData("right pos: ", rSlidesPos);
        //telemetry.addData("left: ", Lpidf);
        //telemetry.addData("right: ", Rpidf);
        //telemetry.addData("pos: ", robot.arm.armMotor.getCurrentPosition());
        //telemetry.addData("angle: ", armAngle);
        telemetry.addData("target: ", target);
        //telemetry.addData("pid: ", robot.arm.armMotor.getPower());
        //telemetry.addData("error: ", controller.error / controller.initError);
        telemetry.update();
        //telemetry.addData("left dir", robot.slides.lSlideMotor.getDirection());
        //telemetry.addData("right dir", robot.slides.rSlideMotor.getDirection());
    }
}