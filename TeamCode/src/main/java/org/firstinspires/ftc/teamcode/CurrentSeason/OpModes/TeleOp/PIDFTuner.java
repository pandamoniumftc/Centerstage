package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;

@Config
@TeleOp (name="pidf tuner")
public class PIDFTuner extends OpMode {
    BaoBao robot;
    PIDFController controller;

    public static double kp = 0, ki = 0, kd = 0, kf = 0;
    public static double ff = 0;
    public static double target = 0;

    @Override
    public void init() {
        this.robot = new BaoBao(this);
        controller = new PIDFController(kp, ki, kd, kf);
        robot.outtake.lSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        int lSlidesPos = robot.outtake.lSlideMotor.getCurrentPosition();
        int rSlidesPos = robot.outtake.rSlideMotor.getCurrentPosition();
        double Lpidf = controller.calculate(lSlidesPos, target, ff);
        double Rpidf = controller.calculate(rSlidesPos, target, ff);

        robot.outtake.lSlideMotor.setPower(Lpidf);
        robot.outtake.rSlideMotor.setPower(Rpidf);

        telemetry.addData("left pos: ", lSlidesPos);
        telemetry.addData("right pos: ", rSlidesPos);
        telemetry.addData("left: ", Lpidf);
        telemetry.addData("right: ", Rpidf);
        telemetry.addData("target: ", target);
    }
}