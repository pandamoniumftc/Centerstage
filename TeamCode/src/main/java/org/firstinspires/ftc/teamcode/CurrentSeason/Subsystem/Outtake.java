package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

public class Outtake extends AbstractSubsystem {
    BaoBao robot;
    public DcMotor lSlideMotor, rSlideMotor;
    public Servo lTiltServo, rTiltServo, pivotServo, clawServo;
    public ColorSensor leftColorSensor, rightColorSensor;
    public DistanceSensor leftDistanceSensor, rightDistanceSensor;
    public int[] slidesPos = new int[] {
            /* default pos */ 0,
            /* first level */ 500,
            /* second level */ 1000,
            /* third level*/ 2000};
    public final int[] slidesBounds = new int[] {
            /* min */ 0,
            /* max */ 2000};
    public final double[] clawServoPos = new double[] {
            /* closed */ .6,
            /* opened */ .2};
    public final double[] tiltServoPos = new double[] {
            /* left, grabbing pos */ 0,
            /* left, default pos */ .1,
            /* left, deposit pos */ .95,
            /* right, grabbing pos */ 1,
            /* right, default pos */ 0.9,
            /* right, deposit pos */ 0.25};
    public final double[] pivotServoPos = new double[] {
            /* default pos */ 0.25,
            /* grabbing pos */ 0.3,
            /* deposit pos */ .7};
    public detected leftSensorState, rightSensorState;
    public PIDFController slidesController = new PIDFController(1, 0, 0, 1);
    public enum detected {
        WHITE_PIXEL,
        YELLOW_PIXEL,
        PURPLE_PIXEL,
        GREEN_PIXEL,
        NO_PIXEL,
        PIXEL
    }

    public Toggle opened = new Toggle(true);
    public Toggle lifted = new Toggle(false);
    public Toggle pivot = new Toggle(false);
    public Toggle x = new Toggle(false);
    public Toggle y = new Toggle(false);
    public Toggle resetSlidePosition = new Toggle(false);
    public Toggle goToBottomLine = new Toggle(false);
    public Toggle goToMiddleLine = new Toggle(false);
    public Toggle goToTopLine = new Toggle(false);
    public Outtake(AbstractRobot robot, String lsm, String rsm, String lts, String rts, String ps, String rs, String lSensor, String rSensor) {
        super(robot);
        this.robot = (BaoBao)robot;

        lSlideMotor = robot.hardwareMap.get(DcMotor.class, lsm);
        rSlideMotor = robot.hardwareMap.get(DcMotor.class, rsm);

        lTiltServo = robot.hardwareMap.get(Servo.class, lts);
        rTiltServo = robot.hardwareMap.get(Servo.class, rts);
        pivotServo = robot.hardwareMap.get(Servo.class, ps);
        clawServo = robot.hardwareMap.get(Servo.class, rs);

        leftColorSensor = robot.hardwareMap.get(ColorSensor.class, lSensor);
        rightColorSensor = robot.hardwareMap.get(ColorSensor.class, rSensor);

        leftDistanceSensor = robot.hardwareMap.get(DistanceSensor.class, lSensor);
        rightDistanceSensor = robot.hardwareMap.get(DistanceSensor.class, rSensor);

        lSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void init() {
        lSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lTiltServo.setDirection(Servo.Direction.REVERSE);
        rTiltServo.setDirection(Servo.Direction.REVERSE);

        lTiltServo.setPosition(tiltServoPos[1]);
        rTiltServo.setPosition(tiltServoPos[4]);
        clawServo.setPosition(clawServoPos[1]);
        pivotServo.setPosition(pivotServoPos[0]);
        robot.state = BaoBao.robotState.NEUTRAL;
    }

    @Override
    public void start() {
        lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driverLoop() {
        double power = robot.gamepad2.right_stick_y;

        if (robot.state == BaoBao.robotState.GRABBED_PIXEL || robot.state == BaoBao.robotState.WAITING_TO_DEPOSIT || robot.state == BaoBao.robotState.DEPOSITED) {
            lSlideMotor.setPower(-power);
            rSlideMotor.setPower(-power);

            resetSlidePosition.updateState(robot.gamepad2.dpad_up);
            goToBottomLine.updateState(robot.gamepad2.dpad_left);
            goToMiddleLine.updateState(robot.gamepad2.dpad_down);
            goToTopLine.updateState(robot.gamepad2.dpad_right);
        }

        // prevents slide position to exceed minimum and maximum height

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

        // power motors and servos

        opened.updateState(robot.gamepad2.left_bumper);
        lifted.updateState(robot.gamepad2.right_bumper);
        pivot.updateState(robot.gamepad2.b);
        x.updateState(robot.gamepad2.x);
        y.updateState(robot.gamepad2.y);

        if (pivot.state) {
            lTiltServo.setPosition(tiltServoPos[0]);
            rTiltServo.setPosition(tiltServoPos[3]);
            pivotServo.setPosition(pivotServoPos[1]);
            robot.state = BaoBao.robotState.WAITING_TO_GRAB;
            if (!opened.state) {
                clawServo.setPosition(clawServoPos[0]);
                robot.state = BaoBao.robotState.GRABBED_PIXEL;
            }
        }
        if (robot.state == BaoBao.robotState.GRABBED_PIXEL && x.state) {
            pivot.state = false;
            lTiltServo.setPosition(tiltServoPos[1]);
            rTiltServo.setPosition(tiltServoPos[4]);
            pivotServo.setPosition(pivotServoPos[0]);
        }
        if (y.state) {
            x.state = false;
            lTiltServo.setPosition(tiltServoPos[1] + .1);
            rTiltServo.setPosition(tiltServoPos[4] - .1);
            pivotServo.setPosition(pivotServoPos[0] - 0.12);
        }
        if (lifted.state) {
            y.state = false;
            lTiltServo.setPosition(tiltServoPos[2]);
            rTiltServo.setPosition(tiltServoPos[5]);
            robot.state = BaoBao.robotState.WAITING_TO_DEPOSIT;
        }

        if (robot.state == BaoBao.robotState.WAITING_TO_DEPOSIT) {
            pivot.state = false;
            pivotServo.setPosition(pivotServoPos[2]);
            if (opened.state) {
                clawServo.setPosition(clawServoPos[1]);
                robot.state = BaoBao.robotState.DEPOSITED;
            }
        }
        if (robot.state == BaoBao.robotState.DEPOSITED && !lifted.state) {
            lTiltServo.setPosition(tiltServoPos[1]);
            rTiltServo.setPosition(tiltServoPos[4]);
            pivotServo.setPosition(pivotServoPos[0]);
            robot.state = BaoBao.robotState.NEUTRAL;
        }

        //lTiltServo.setPosition(tiltServoPos[lifted.state ? 2 : 0]);
        //rTiltServo.setPosition(tiltServoPos[lifted.state ? 5 : 3]);
        //clawServo.setPosition(clawServoPos[opened.state ? 1 : 0]);
        //pivotServo.setPosition(pivotServoPos[pivot.state ? 1 : 0]);

        // keeps track of pixels collected in outtake

        if (leftDistanceSensor.getDistance(DistanceUnit.INCH) > .1) {leftSensorState = detected.NO_PIXEL;}
        else {leftSensorState = detected.PIXEL;}
        if (rightDistanceSensor.getDistance(DistanceUnit.INCH) > .1) {rightSensorState = detected.NO_PIXEL;}
        else {rightSensorState = detected.PIXEL;}

        // function to deposit pixel

        telemetry.addData("opened: ", opened.state);
        telemetry.addData("lifted: ", lifted.state);
        telemetry.addData("pivot: ", pivot.state);
        telemetry.addData("robot state: ", robot.state);
        telemetry.addData("front pixel: ", leftSensorState);
        telemetry.addData("back pixel: ", rightSensorState);
        telemetry.addData("front dis: ", leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("back dis: ", rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left slide motor: ", lSlideMotor.getCurrentPosition());
        telemetry.addData("right slide motor: ", rSlideMotor.getCurrentPosition());
    }

    @Override
    public void stop() {

    }

}
