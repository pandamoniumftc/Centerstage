package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.CurvesPort.Curve;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;
import org.opencv.core.Point;

import java.io.IOException;

public class Outtake extends AbstractSubsystem {
    public DcMotor lSlideMotor, rSlideMotor;
    public Servo lTiltServo, rTiltServo, fReleaseServo, bReleaseServo;
    public RevColorSensorV3 backSensor, frontSensor;
    public int[] slidesPos = new int[] {0, 100, 200, 300};
    public final int[] slidesBounds = new int[] {0, 1500};
    public final double[] releaseServoPos = new double[] {0, 0.2, 0.4, 0.58};
    public final double[] tiltServoPos = new double[] {.9, .6, 0.1, 0.45};
    private final Point[] TiltServoPoints = new Point[] {
            new Point(0, 0),
            new Point(0, .44),
            new Point(.25, 0.44),
            new Point(.25, 0)
    };
    VariantDegreeBezier vdbc = new VariantDegreeBezier(TiltServoPoints);

    Curve[] curve = new Curve[]{vdbc};
    public CurveSequence TiltServoCurve;
    public int pixelsCollected;
    public detected frontSensorState, backSensorState;
    public enum detected {
        PIXEL,
        NO_PIXEL
    }
    public enum color {
        WHITE,
        YELLOW,
        PURPLE,
        GREEN
    }

    public Toggle released = new Toggle(false);
    public Toggle lifted = new Toggle(false);
    public Toggle reset = new Toggle(false);
    public Outtake(AbstractRobot robot, String lsm, String rsm, String lts, String rts, String frs, String brs, String fSensor, String bSensor/*, CurveSequence slidePowerCurve*/) {
        super(robot);

        lSlideMotor = robot.hardwareMap.get(DcMotor.class, lsm);
        rSlideMotor = robot.hardwareMap.get(DcMotor.class, rsm);

        lTiltServo = robot.hardwareMap.get(Servo.class, lts);
        rTiltServo = robot.hardwareMap.get(Servo.class, rts);
        fReleaseServo = robot.hardwareMap.get(Servo.class, frs);
        bReleaseServo = robot.hardwareMap.get(Servo.class, brs);

        frontSensor = robot.hardwareMap.get(RevColorSensorV3.class, fSensor);
        backSensor = robot.hardwareMap.get(RevColorSensorV3.class, bSensor);

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
        fReleaseServo.setPosition(releaseServoPos[3]);
        bReleaseServo.setPosition(releaseServoPos[0]);
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

        // power motors and servos

        released.updateState(robot.gamepad2.left_bumper);
        lifted.updateState(robot.gamepad2.right_bumper);
        reset.updateState(robot.gamepad2.dpad_up);

        lSlideMotor.setPower(-power);
        rSlideMotor.setPower(-power);

        lTiltServo.setPosition(tiltServoPos[lifted.state ? 1 : 0]);
        rTiltServo.setPosition(tiltServoPos[lifted.state ? 3 : 2]);

        // keeps track of pixels collected in outtake

        if (frontSensor.getDistance(DistanceUnit.INCH) > .55) {frontSensorState = detected.NO_PIXEL;}
        if (backSensor.getDistance(DistanceUnit.INCH) > .55) {backSensorState = detected.NO_PIXEL;}

        if (frontSensor.getDistance(DistanceUnit.INCH) < .55) {frontSensorState = detected.PIXEL;}
        if (backSensor.getDistance(DistanceUnit.INCH) < .55) {backSensorState = detected.PIXEL;}

        if (frontSensorState == detected.NO_PIXEL && backSensorState == detected.NO_PIXEL) {
            pixelsCollected = 0;
        }
        if (frontSensorState == detected.PIXEL ^ backSensorState == detected.PIXEL) {
            pixelsCollected = 1;
        }
        else {
            pixelsCollected = 2;
        }

        // function to deposit pixel

        if (released.state && lifted.state) {
            fReleaseServo.setPosition(releaseServoPos[2]); // front up
            bReleaseServo.setPosition(releaseServoPos[1]); // back down
            if (frontSensorState == detected.NO_PIXEL || backSensorState == detected.NO_PIXEL) {
                fReleaseServo.setPosition(releaseServoPos[3]); // front down
                bReleaseServo.setPosition(releaseServoPos[0]); // back up
                released.state = false;
            }
        }

        if (reset.state) {
            fReleaseServo.setPosition(releaseServoPos[2]);
            bReleaseServo.setPosition(releaseServoPos[0]);
            if (frontSensorState == detected.NO_PIXEL && backSensorState == detected.NO_PIXEL) {
                fReleaseServo.setPosition(releaseServoPos[3]);
                reset.state = false;
                released.state = false;
            }
        }

        /*if (frontSensor.argb() > 20) {frontSensorState = detected.WHITE;}
        if (frontSensor.argb() > 3.14) {frontSensorState = detected.PURPLE;}
        if (frontSensor.argb() > 159) {frontSensorState = detected.YELLOW;}
        if (frontSensor.argb() > 265) {frontSensorState = detected.GREEN;}
        if (backSensor.argb() > 20) {backSensorState = detected.WHITE;}
        if (backSensor.argb() > 3.14) {backSensorState = detected.PURPLE;}
        if (backSensor.argb() > 159) {backSensorState = detected.YELLOW;}
        if (backSensor.argb() > 265) {backSensorState = detected.GREEN;}*/

        telemetry.addData("released: ", released.state);
        telemetry.addData("lifted: ", lifted.state);
        telemetry.addData("reset: ", reset.state);
        telemetry.addData("pixels: ", pixelsCollected);
        telemetry.addData("front pixel: ", frontSensorState);
        telemetry.addData("back pixel: ", backSensorState);
        telemetry.addData("bump :", robot.gamepad2.left_bumper);
        telemetry.addData("left slide motor: ", lSlideMotor.getCurrentPosition());
        telemetry.addData("right slide motor: ", rSlideMotor.getCurrentPosition());
    }

    @Override
    public void stop() {

    }

    /*public double ServoCurveOutput (double start, double end, CurveSequence sequence, long seconds, Toggle toggle) {
        long stamp = System.currentTimeMillis();
        if (System.currentTimeMillis() - stamp > seconds) {

        }
        double delta = end - start;
        return sequence.evaluate((System.currentTimeMillis() - (seconds*1000) * 1000.0) / (1000.0 * seconds));
    }*/

}
