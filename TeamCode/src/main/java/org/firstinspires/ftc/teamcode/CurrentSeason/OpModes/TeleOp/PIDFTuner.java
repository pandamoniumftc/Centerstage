package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;

@Config
@TeleOp (name="pidf tuner")
public class PIDFTuner extends AbstractTeleOp {
    BaoBao robot;
    PIDFController controller;
    public static double kp = 0, ki = 0, kd = 0, kf = 0;
    public static double ff = 0;
    public static double target = 0;

    @Override
    public AbstractRobot instantiateRobot() {
        this.robot = new BaoBao(this);
        return robot;
    }

    @Override
    public void onInit() {
        controller = new PIDFController(kp, ki, kd, kf);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onDriverUpdate() {
        int lSlidesPos = robot.outtake.lSlideMotor.getCurrentPosition();
        int rSlidesPos = robot.outtake.rSlideMotor.getCurrentPosition();
        double Lpidf = controller.calculate(lSlidesPos, target, ff);
        double Rpidf = controller.calculate(rSlidesPos, target, ff);

        robot.outtake.lSlideMotor.setPower(Lpidf);
        robot.outtake.rSlideMotor.setPower(Rpidf);

        telemetry.addData("left pos: ", lSlidesPos);
        telemetry.addData("right pos: ", rSlidesPos);
        telemetry.addData("target: ", target);
    }
}