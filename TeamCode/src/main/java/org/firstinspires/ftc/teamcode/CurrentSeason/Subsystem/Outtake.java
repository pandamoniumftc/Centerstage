package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.CurvesPort.Curve;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;
import org.opencv.core.Point;

public class Outtake extends AbstractSubsystem {
    public DcMotor lSlideMotor, rSlideMotor;
    public Servo lTiltServo, rTiltServo, releaseServo;
    public ColorSensor backColorSensor, frontColorSensor;
    public DistanceSensor backDistanceSensor, frontDistanceSensor;
    public int[] slidesPos = new int[] {0, 500, 1000, 1500};
    public final int[] slidesBounds = new int[] {0, 2000};
    public final double[] releaseServoPos = new double[] {1, 0.8};
    public final double[] tiltServoPos = new double[] {.55, .25, 0.5, 0.85};
    private final Point[] TiltServoPoints = new Point[] {
            new Point(0, 0),
            new Point(0, .44),
            new Point(.25, 0.44),
            new Point(.25, 0)
    };
    VariantDegreeBezier vdbc = new VariantDegreeBezier(TiltServoPoints);

    Curve[] curve = new Curve[]{vdbc};
    public CurveSequence TiltServoCurve;
    public detected frontSensorState, backSensorState;
    public PIDFController slidesController = new PIDFController(1, 0, 0, 1);
    public enum detected {
        WHITE_PIXEL,
        YELLOW_PIXEL,
        PURPLE_PIXEL,
        GREEN_PIXEL,
        NO_PIXEL,
        PIXEL
    }

    public Toggle released = new Toggle(false);
    public Toggle lifted = new Toggle(false);
    public Toggle resetSlidePosition = new Toggle(false);
    public Toggle firstLevel = new Toggle(false);
    public Toggle secondLevel = new Toggle(false);
    public Toggle thirdLevel = new Toggle(false);
    public Outtake(AbstractRobot robot, String lsm, String rsm, String lts, String rts, String rs, String fSensor, String bSensor/*, CurveSequence slidePowerCurve*/) {
        super(robot);

        lSlideMotor = robot.hardwareMap.get(DcMotor.class, lsm);
        rSlideMotor = robot.hardwareMap.get(DcMotor.class, rsm);

        lTiltServo = robot.hardwareMap.get(Servo.class, lts);
        rTiltServo = robot.hardwareMap.get(Servo.class, rts);
        releaseServo = robot.hardwareMap.get(Servo.class, rs);

        frontColorSensor = robot.hardwareMap.get(ColorSensor.class, fSensor);
        backColorSensor = robot.hardwareMap.get(ColorSensor.class, bSensor);

        frontDistanceSensor = robot.hardwareMap.get(DistanceSensor.class, fSensor);
        backDistanceSensor = robot.hardwareMap.get(DistanceSensor.class, bSensor);

        lSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.TiltServoCurve = new CurveSequence(curve);
    }

    @Override
    public void init() {
        rSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lTiltServo.setDirection(Servo.Direction.REVERSE);
        rTiltServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start() {
        lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driverLoop() {
        double power = robot.gamepad2.right_stick_y;

        // prevents slide position to exceed minimum and maximum height

        if (lSlideMotor.getCurrentPosition() < slidesBounds[0] && rSlideMotor.getCurrentPosition() < slidesBounds[0]) {
            power = Math.min(0, power);
        }
        if (lSlideMotor.getCurrentPosition() > slidesBounds[1] && rSlideMotor.getCurrentPosition() > slidesBounds[1]) {
            power = Math.max(0, power);
        }

        resetSlidePosition.updateState(robot.gamepad2.dpad_up);
        firstLevel.updateState(robot.gamepad2.dpad_left);
        secondLevel.updateState(robot.gamepad2.dpad_down);
        thirdLevel.updateState(robot.gamepad2.dpad_right);

        if (resetSlidePosition.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[0], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[0], 0));
        }
        if (firstLevel.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[1], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[1], 0));
        }
        if (secondLevel.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[2], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[2], 0));
        }
        if (thirdLevel.state) {
            lSlideMotor.setPower(slidesController.calculate(lSlideMotor.getCurrentPosition(), slidesPos[3], 0));
            rSlideMotor.setPower(slidesController.calculate(rSlideMotor.getCurrentPosition(), slidesPos[3], 0));
        }

        // power motors and servos

        released.updateState(robot.gamepad2.left_bumper);
        lifted.updateState(robot.gamepad2.right_bumper);

        lSlideMotor.setPower(-power);
        rSlideMotor.setPower(-power);

        lTiltServo.setPosition(tiltServoPos[lifted.state ? 0 : 1]);
        rTiltServo.setPosition(tiltServoPos[lifted.state ? 2 : 3]);

        releaseServo.setPosition(releaseServoPos[released.state ? 1 : 0]);

        // keeps track of pixels collected in outtake

        if (frontDistanceSensor.getDistance(DistanceUnit.INCH) > .50) {frontSensorState = detected.NO_PIXEL;}
        else {frontSensorState = detected.PIXEL;}
        if (backDistanceSensor.getDistance(DistanceUnit.INCH) > .40) {backSensorState = detected.NO_PIXEL;}
        else {backSensorState = detected.PIXEL;}

        // function to deposit pixel


        telemetry.addData("released: ", released.state);
        telemetry.addData("lifted: ", lifted.state);
        telemetry.addData("front pixel: ", frontSensorState);
        telemetry.addData("back pixel: ", backSensorState);
        telemetry.addData("front dis: ", frontDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("back dis: ", backDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left slide motor: ", lSlideMotor.getCurrentPosition());
        telemetry.addData("right slide motor: ", rSlideMotor.getCurrentPosition());
    }

    @Override
    public void stop() {

    }

}
