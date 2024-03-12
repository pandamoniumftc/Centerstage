package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Timer;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.CurvesPort.Curve;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Point;

import java.io.IOException;

public class LinearSlides extends AbstractSubsystem {
    Po robot;
    public DcMotor lSlideMotor, rSlideMotor;
    public final int[] slidesBounds = new int[] {
            /* min */ 0,
            /* max */ 2000};
    public PIDFController slidesController;
    public CurveSequence SlidesProfile;
    public Toggle liftSlide, lowerSlide, hang;
    public int targetPos;
    public double strength;
    public Timer timer;
    public LinearSlides(AbstractRobot robot, String lsm, String rsm, Point[][] SlidesPoint) {
        super(robot);
        this.robot = (Po) robot;

        lSlideMotor = robot.hardwareMap.get(DcMotor.class, lsm);
        rSlideMotor = robot.hardwareMap.get(DcMotor.class, rsm);

        slidesController = new PIDFController(0.004, 0.001, 0, 0.0001);

        lSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftSlide = new Toggle(false);
        lowerSlide = new Toggle(false);
        hang = new Toggle(false);

        targetPos = 0;

        this.SlidesProfile = CurveSequence.init(SlidesPoint);
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
        // timer for hang
        hang.updateState(robot.gamepad2.y);
        // changes the height by this amount
        int delta = 200;

        // adjust speed based on height of the slides
        double n = (double) (lSlideMotor.getCurrentPosition() - 400) / (slidesBounds[1] - 400);
        strength = n * 0.5 + (1.0 - n);

        // increases target pos by delta val, caps slides at max
        if (liftSlide.state) {
            targetPos += delta;
            liftSlide.state = false;
        }
        else if ((targetPos + delta) < slidesBounds[1] && robot.state == Po.robotState.DEPOSIT) {
            liftSlide.updateState(robot.gamepad2.dpad_up);
        }

        // decreases target pos by delta val, caps slides at min
        if (lowerSlide.state) {
            targetPos -= delta;
            lowerSlide.state = false;
        }
        else if ((targetPos - delta) > slidesBounds[0] && robot.state == Po.robotState.DEPOSIT) {
            lowerSlide.updateState(robot.gamepad2.dpad_down);
        }

        // set slides to target pos during deposit state, resets slides position after deposit state
        if (robot.state == Po.robotState.DEPOSIT) {
            if (!robot.wait(3, robot.arm.leftOpened.state && robot.arm.rightOpened.state)) {
                // automatically lowers arm and closes claw
                setSlidesPosition(targetPos);
            }
            else {
                setSlidesPosition(0);
                if (lSlideMotor.getCurrentPosition() == 0) {robot.state = Po.robotState.NEUTRAL;}
            }


        }

        // goes into hang state
        if (hang.state) {
            robot.state = Po.robotState.HANGING;
            lSlideMotor.setPower(-robot.gamepad2.right_stick_y);
            rSlideMotor.setPower(-robot.gamepad2.right_stick_y);
        }

        telemetry.addData("TARGET: ", targetPos + " | MOTORS POS: " + lSlideMotor.getCurrentPosition() + " " + rSlideMotor.getCurrentPosition());
    }
    public void setSlidesPosition(int targetPos) {
        lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), targetPos, 1));
        rSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), targetPos, 1));
        /*timer.newSlideTask(animTime, targetPos);

        if ((targetPos > timer.slides_task[4] ? 1 : 0) == timer.slides_task[2] && timer.slides_task[0] != 0) {
            double t = (Range.clip(timer.x, 0, 1) - SlidesProfile.minX)/(SlidesProfile.maxX - SlidesProfile.minX);

            double slidePosition = SlidesProfile.evaluate(t) [0];
            double slideVelocity = SlidesProfile.evaluate(t) [1];
            double slideAcceleration = SlidesProfile.evaluate(t) [2];

            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidePosition * timer.slides_task[3] + (1.0 - slidePosition) * timer.slides_task[4], 1, slideVelocity, slideAcceleration));
            rSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidePosition * timer.slides_task[3] + (1.0 - slidePosition) * timer.slides_task[4], 1, slideVelocity, slideAcceleration));
        }*/
    }
    @Override
    public void stop() {

    }
}