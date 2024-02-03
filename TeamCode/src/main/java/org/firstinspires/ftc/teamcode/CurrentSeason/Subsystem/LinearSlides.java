package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.IOException;

public class LinearSlides extends AbstractSubsystem {
    Po robot;
    public DcMotor lSlideMotor, rSlideMotor;
    public int[] slidesPos = new int[] {
            /* default pos */ 0,
            /* first level */ 500,
            /* second level */ 1000,
            /* third level*/ 2000};
    public final int[] slidesBounds = new int[] {
            /* min */ 0,
            /* max */ 2000};
    public PIDFController slidesController = new PIDFController(1, 0, 0, 1);
    public Toggle resetSlidePosition = new Toggle(false);
    public Toggle goToBottomLine = new Toggle(false);
    public Toggle goToMiddleLine = new Toggle(false);
    public Toggle goToTopLine = new Toggle(false);
    public LinearSlides(AbstractRobot robot, String lsm, String rsm) {
        super(robot);
        this.robot = (Po) robot;

        lSlideMotor = robot.hardwareMap.get(DcMotor.class, lsm);
        rSlideMotor = robot.hardwareMap.get(DcMotor.class, rsm);

        lSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init() throws IOException {
        lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driverLoop() {
        double power = -robot.gamepad2.right_stick_y;

        if (robot.state == Po.robotState.GRABBED_PIXEL || robot.state == Po.robotState.WAITING_TO_DEPOSIT || robot.state == Po.robotState.DEPOSITED) {


            resetSlidePosition.updateState(robot.gamepad2.dpad_up);
            goToBottomLine.updateState(robot.gamepad2.dpad_left);
            goToMiddleLine.updateState(robot.gamepad2.dpad_down);
            goToTopLine.updateState(robot.gamepad2.dpad_right);
        }

        lSlideMotor.setPower(power);
        rSlideMotor.setPower(power);

        if (robot.state == Po.robotState.HANGING) {

        }

        if (lSlideMotor.getCurrentPosition() < slidesBounds[0] && rSlideMotor.getCurrentPosition() < slidesBounds[0]) {
            power = Math.min(0, power);
        }
        if (lSlideMotor.getCurrentPosition() > slidesBounds[1] && rSlideMotor.getCurrentPosition() > slidesBounds[1]) {
            power = Math.max(0, power);
        }

        if (resetSlidePosition.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[0], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[0], 0));
        }
        if (goToBottomLine.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[1], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[1], 0));
        }
        if (goToMiddleLine.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[2], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[2], 0));
        }
        if (goToTopLine.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[3], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[3], 0));
        }

        telemetry.addData("left slide motor: ", lSlideMotor.getCurrentPosition());
        telemetry.addData("right slide motor: ", rSlideMotor.getCurrentPosition());
    }

    public void setSlidesPosition(int targetPos) {
        lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), targetPos, 0));
        rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), targetPos, 0));
    }

    @Override
    public void stop() {

    }
}
