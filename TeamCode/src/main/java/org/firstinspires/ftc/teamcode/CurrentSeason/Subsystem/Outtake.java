package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

import java.io.IOException;

public class Outtake extends AbstractSubsystem {
    public DcMotor lSlideMotor, rSlideMotor;
    public Servo lTiltServo, rTiltServo, fReleaseServo, bReleaseServo;
    public RevColorSensorV3 backSensor, frontSensor;
    public int[] slidesPos = new int[] {0, 100, 200, 300};
    public final int[] slidesBounds = new int[] {0, 1650};
    public final double[] releaseServoPos = new double[] {0, 0.08, 0.45, 0.58};
    public int pixelsCollected;
    public enum detectedColor {
        FRONT_PURPLE(),
    }
    public long startTimeStamp;
    public Toggle released = new Toggle(false);
    public Outtake(AbstractRobot robot, String lsm, String rsm, String lts, String rts, String frs, String brs, String fSensor, String bSensor) {
        super(robot);

        lSlideMotor = robot.hardwareMap.get(DcMotor.class, lsm);
        rSlideMotor = robot.hardwareMap.get(DcMotor.class, rsm);

        //lTiltServo = robot.hardwareMap.get(Servo.class, lts);
        //rTiltServo = robot.hardwareMap.get(Servo.class, rts);
        fReleaseServo = robot.hardwareMap.get(Servo.class, frs);
        bReleaseServo = robot.hardwareMap.get(Servo.class, brs);

        frontSensor = robot.hardwareMap.get(RevColorSensorV3.class, fSensor);
        backSensor = robot.hardwareMap.get(RevColorSensorV3.class, bSensor);

        lSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() throws IOException {
        lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        double speedMultiplier = (1 - robot.gamepad2.left_trigger) * 0.75 + 0.25;

        // trigger input at 0: 100% power, trigger input at 1: 25%

        if (lSlideMotor.getCurrentPosition() < slidesBounds[0] && rSlideMotor.getCurrentPosition() < slidesBounds[0]) {
            power = Math.min(0, power);
        }
        if (lSlideMotor.getCurrentPosition() > slidesBounds[1] && rSlideMotor.getCurrentPosition() > slidesBounds[1]) {
            power = Math.max(0, power);
        }

        /*if (frontColorSensor.argb() == 1 && backColorSensor.argb() == 1) {
            pixelsCollected = 2;
        }
        if (frontColorSensor.argb() == 1 && backColorSensor.argb() == 1) {
            pixelsCollected = 2;
        }*/

        released.updateState(robot.gamepad2.left_bumper);

        if (frontSensor.getDistance(DistanceUnit.INCH) < 0.26) {
            startTimeStamp = System.currentTimeMillis();
            if (System.currentTimeMillis() - startTimeStamp > 1500.0) {bReleaseServo.setPosition(releaseServoPos[1]);}
        }

        if (released.state) {
            startTimeStamp = System.currentTimeMillis();
            fReleaseServo.setPosition(releaseServoPos[2]);
            if (frontSensor.getDistance(DistanceUnit.INCH) > 0.26 && (System.currentTimeMillis() - startTimeStamp > 1500.0)) {
                fReleaseServo.setPosition(releaseServoPos[3]);
                bReleaseServo.setPosition(releaseServoPos[0]);
            }
            if (backSensor.getDistance(DistanceUnit.INCH) > 0.55 && frontSensor.getDistance(DistanceUnit.INCH) < 0.55) {bReleaseServo.setPosition(releaseServoPos[1]);}
            released.state = false;
        }

        lSlideMotor.setPower(-power * speedMultiplier);
        rSlideMotor.setPower(-power * speedMultiplier);

        telemetry.addData("slide motor power: ", power);
        telemetry.addData("left slide motor: ", lSlideMotor.getCurrentPosition());
        telemetry.addData("right slide motor: ", rSlideMotor.getCurrentPosition());
        telemetry.addData("left servo pos: ", fReleaseServo.getPosition());
        telemetry.addData("right servo pos: ", bReleaseServo.getPosition());
        telemetry.addData("front sensor dis: ", frontSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("back sensor dis: ", backSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("time: ", System.currentTimeMillis());
        telemetry.addData("stamp: ", startTimeStamp);
    }

    @Override
    public void stop() {

    }
}
