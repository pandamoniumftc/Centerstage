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
    public final int[] slidesBounds = new int[] {
            /* min */ 0,
            /* max */ 2000};
    public PIDFController slidesController;
    public Toggle liftSlide = new Toggle(false);
    public Toggle lowerSlide = new Toggle(false);
    public Toggle hang = new Toggle(false);
    private long initTime;
    public int targetPos = 0;
    public LinearSlides(AbstractRobot robot, String lsm, String rsm) {
        super(robot);
        this.robot = (Po) robot;

        lSlideMotor = robot.hardwareMap.get(DcMotor.class, lsm);
        rSlideMotor = robot.hardwareMap.get(DcMotor.class, rsm);

        slidesController = new PIDFController(0.005, 0.001, 0.0001, 0.0001);

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
        initTime = System.currentTimeMillis();
    }

    @Override
    public void driverLoop() {
        // timer for hang
        if (System.currentTimeMillis() - initTime > 120000) {
            hang.updateState(robot.gamepad2.y);
        }
        // changes the height by this amount
        int delta = 100;

        // increases target pos by delta val, caps slides at max
        if (liftSlide.state) {
            targetPos += delta;
            liftSlide.state = false;
        }
        else if ((targetPos + delta) < slidesBounds[1] && robot.state == Po.robotState.WAITING_TO_DEPOSIT) {
            liftSlide.updateState(robot.gamepad2.dpad_up);
        }

        // decreases target pos by delta val, caps slides at min
        if (lowerSlide.state) {
            targetPos -= delta;
            lowerSlide.state = false;
        }
        else if ((targetPos - delta) > slidesBounds[0] && robot.state == Po.robotState.WAITING_TO_DEPOSIT) {
            lowerSlide.updateState(robot.gamepad2.dpad_down);
        }

        // set slides to target pos during deposit state, resets slides position after deposit state
        if (robot.state == Po.robotState.WAITING_TO_DEPOSIT) {
            setSlidesPosition(targetPos);
        }
        else if (robot.state == Po.robotState.DEPOSITED) {
            setSlidesPosition(0);
        }

        // goes into hang state
        if (hang.state) {
            robot.state = Po.robotState.HANGING;
            lSlideMotor.setPower(-robot.gamepad2.right_stick_y);
            rSlideMotor.setPower(-robot.gamepad2.right_stick_y);
        }

        /*if (robot.state == Po.robotState.WAITING_TO_DEPOSIT || robot.state == Po.robotState.DEPOSITED) {
            double power = -robot.gamepad2.right_stick_y;

            if (lSlideMotor.getCurrentPosition() < slidesBounds[0] && rSlideMotor.getCurrentPosition() < slidesBounds[0]) {
                power = Math.min(0, power);
            }
            if (lSlideMotor.getCurrentPosition() > slidesBounds[1] && rSlideMotor.getCurrentPosition() > slidesBounds[1]) {
                power = Math.max(0, power);
            }

            lSlideMotor.setPower(power);
            rSlideMotor.setPower(power);
        }*/

        telemetry.addData("left slide motor: ", lSlideMotor.getCurrentPosition());
        telemetry.addData("right slide motor: ", rSlideMotor.getCurrentPosition());
        telemetry.addData("lift: ", liftSlide.state);
        telemetry.addData("lower: ", lowerSlide.state);
        telemetry.addData("target: ", targetPos);
    }
    public void setSlidesPosition(int targetPos) {
        lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), targetPos, 1));
        rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), targetPos, 1));
    }
    @Override
    public void stop() {

    }
}