package org.firstinspires.ftc.teamcode.OffSeason.Subsystem;

import static java.lang.Math.signum;
import static com.qualcomm.robotcore.util.Range.clip;

import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.OffSeason.Robots.Po;
import org.opencv.core.Point;

import java.io.IOException;

public class MecanumDrive extends AbstractSubsystem {
    Po robot;
    public CuttleMotor leftFront, rightFront, leftRear, rightRear;
    public MecanumController driveTrain;
    public CurveSequence MovementProfile;
    public MecanumDrive(AbstractRobot robot, int flm, int frm, int blm, int brm, Point[][] curve) {
        super(robot);
        this.robot = (Po) robot;

        leftFront = this.robot.expHub.getMotor(Math.abs(flm));
        rightFront = this.robot.ctrlHub.getMotor(Math.abs(frm));
        leftRear = this.robot.expHub.getMotor(Math.abs(blm));
        rightRear = this.robot.ctrlHub.getMotor(Math.abs(brm));

        rightFront.setDirection(signum(frm) == 1 ? Direction.FORWARD : Direction.REVERSE);
        rightRear.setDirection(signum(brm) == 1 ? Direction.FORWARD : Direction.REVERSE);
        leftRear.setDirection(signum(blm) == 1 ? Direction.FORWARD : Direction.REVERSE);
        leftFront.setDirection(signum(flm) == 1 ? Direction.FORWARD : Direction.REVERSE);

        driveTrain = new MecanumController(rightFront, rightRear, leftFront, leftRear);
        MovementProfile = CurveSequence.init(curve);
    }

    @Override
    public void init() throws IOException {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        double scale = 12.0 / (robot.ctrlHub.getBatteryVoltage() / 1000.0);

        driveTrain.setVec(new Pose(
                MovementProfile.evaluate(Math.abs(clip(robot.gamepad1.left_stick_x, -1, 1)))[0] * signum(robot.gamepad1.left_stick_x),
                MovementProfile.evaluate(Math.abs(clip(-robot.gamepad1.left_stick_y, -1, 1)))[0] * signum(-robot.gamepad1.left_stick_y),
                MovementProfile.evaluate(Math.abs(clip(-robot.gamepad1.right_stick_x, -1, 1)))[0] * signum(-robot.gamepad1.right_stick_x)
        ));

        leftFront.power *= scale;
        rightFront.power *= scale;
        leftRear.power *= scale;
        rightRear.power *= scale;

        telemetry.addData("DRIVE POWER: ", leftFront.power + " " + rightFront.power + " " + leftRear.power + " " + rightRear.power);
        telemetry.addData("VOLT SCALE: ", scale);
    }

    @Override
    public void stop() {

    }
}
