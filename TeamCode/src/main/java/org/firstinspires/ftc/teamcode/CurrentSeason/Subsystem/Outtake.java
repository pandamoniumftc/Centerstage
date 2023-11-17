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
    public final double[] releaseServoPos = new double[] {0, 0.08, 0.45, 0.58};
    public final double[] tiltServoPos = new double[] {0, 1, 0.3, 0.35};
    private final Point[] ExponentialPoints = new Point[] {
            new Point(0, 0),
            new Point(.5, .1),
            new Point(1, 0.25),
            new Point(1, 1)
    };
    VariantDegreeBezier vdbc = new VariantDegreeBezier(ExponentialPoints);

    Curve[] curve = new Curve[]{vdbc};
    public CurveSequence slidePowerCurve;
    public int pixelsCollected;
    public enum detectedColor {
        WHITE,
        PURPLE,
        GREEN,
        YELLOW
    }
    public long startTimeStamp;
    public Toggle released = new Toggle(false);
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

        this.slidePowerCurve = new CurveSequence(curve);
    }

    @Override
    public void init() {
        rSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rTiltServo.setDirection(Servo.Direction.REVERSE);
        lTiltServo.setDirection(Servo.Direction.REVERSE);
        fReleaseServo.setPosition(releaseServoPos[3]);
        bReleaseServo.setPosition(releaseServoPos[0]);
        rTiltServo.setPosition(releaseServoPos[2]);
        lTiltServo.setPosition(releaseServoPos[0]);
    }

    @Override
    public void start() {
        lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void driverLoop() {
        double power = robot.gamepad2.right_stick_y;//slidePowerCurve.evaluate((Math.abs(robot.gamepad2.right_stick_y) - slidePowerCurve.minX) / (slidePowerCurve.maxX - slidePowerCurve.minX));

        if (lSlideMotor.getCurrentPosition() < slidesBounds[0] && rSlideMotor.getCurrentPosition() < slidesBounds[0]) {
            power = Math.min(0, power);
        }
        if (lSlideMotor.getCurrentPosition() > slidesBounds[1] && rSlideMotor.getCurrentPosition() > slidesBounds[1]) {
            power = Math.max(0, power);
        }

        lSlideMotor.setPower(-power * .75/*robot.gamepad2.right_stick_y < 0 ? power : -power*/);
        rSlideMotor.setPower(-power * .75/*robot.gamepad2.right_stick_y < 0 ? power : -power*/);

        // keeps track of pixels collected in outtake

        if (frontSensor.getDistance(DistanceUnit.INCH) > .55 && backSensor.getDistance(DistanceUnit.INCH) > .55) {
            pixelsCollected = 0;
        }
        if (frontSensor.getDistance(DistanceUnit.INCH) < .55 ^ backSensor.getDistance(DistanceUnit.INCH) < .55) {
            pixelsCollected = 1;
        }
        else {
            pixelsCollected = 2;
        }

        // function to deposit pixel

        released.updateState(robot.gamepad2.left_bumper);

        if (frontSensor.getDistance(DistanceUnit.INCH) < 0.55 && backSensor.getDistance(DistanceUnit.INCH) > 0.55) {
            bReleaseServo.setPosition(releaseServoPos[1]); // back down
        }

        if (robot.gamepad2.left_bumper) {
            startTimeStamp = System.currentTimeMillis();
        }

        if (released.state) {
            fReleaseServo.setPosition(releaseServoPos[2]); // front up
            if (frontSensor.getDistance(DistanceUnit.INCH) > 0.55 && System.currentTimeMillis() - startTimeStamp > 3000L) {
                fReleaseServo.setPosition(releaseServoPos[3]); // front down
                bReleaseServo.setPosition(releaseServoPos[0]); // back up
                released.state = false;
            }
        }

        lTiltServo.setPosition(tiltServoPos[robot.gamepad2.a ? 1 : 0]);
        //rTiltServo.setPosition(tiltServoPos[robot.gamepad2.b ? 3 : 2]);

        lTiltServo.setPosition(robot.gamepad2.right_trigger * tiltServoPos[0] + (1 - robot.gamepad2.right_trigger) * tiltServoPos[1]);
        //rTiltServo.setPosition(robot.gamepad2.right_trigger * tiltServoPos[2] + (1 - robot.gamepad2.right_trigger) * tiltServoPos[3]);

        telemetry.addData("slide motor power: ", power);
        telemetry.addData("left slide motor: ", lSlideMotor.getCurrentPosition());
        telemetry.addData("right slide motor: ", rSlideMotor.getCurrentPosition());
        telemetry.addData("front servo pos: ", fReleaseServo.getPosition());
        telemetry.addData("back servo pos: ", bReleaseServo.getPosition());
        telemetry.addData("left servo pos: ", lTiltServo.getPosition());
        telemetry.addData("right servo pos: ", rTiltServo.getPosition());
        telemetry.addData("front sensor dis: ", frontSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("back sensor dis: ", backSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("time: ", System.currentTimeMillis());
        telemetry.addData("stamp: ", startTimeStamp);
        telemetry.addData("pixels: ", pixelsCollected);
    }

    @Override
    public void stop() {

    }
}
