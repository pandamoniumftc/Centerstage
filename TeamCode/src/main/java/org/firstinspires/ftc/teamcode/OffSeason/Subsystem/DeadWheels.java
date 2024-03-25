package org.firstinspires.ftc.teamcode.OffSeason.Subsystem;

import static java.lang.Math.signum;

import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.OffSeason.Robots.Po;

import java.io.IOException;

public class DeadWheels extends AbstractSubsystem {
    Po robot;
    public CuttleEncoder leftEncoder, rightEncoder, frontEncoder;
    public ThreeEncoderLocalizer localizer;
    public DeadWheels(AbstractRobot robot, int l, int r, int f, int ticks, double wheelRad, double wheelDist, double calibration) {
        super(robot);
        this.robot = (Po) robot;

        leftEncoder = this.robot.expHub.getEncoder(Math.abs(l), ticks);
        rightEncoder = this.robot.ctrlHub.getEncoder(Math.abs(r), ticks);
        frontEncoder = this.robot.expHub.getEncoder(Math.abs(f), ticks);

        leftEncoder.setDirection(signum(l) == 1 ? Direction.FORWARD : Direction.REVERSE);
        rightEncoder.setDirection(signum(r) == 1 ? Direction.FORWARD : Direction.REVERSE);
        frontEncoder.setDirection(signum(f) == 1 ? Direction.FORWARD : Direction.REVERSE);

        localizer = new ThreeEncoderLocalizer(
                leftEncoder,
                frontEncoder,
                rightEncoder,
                wheelRad,
                wheelDist,
                calibration
        );
    }

    @Override
    public void init() throws IOException {
        localizer.setPos(Po.currentPos);
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        localizer.update();

        if (robot.gamepad1.right_bumper) {
            localizer.reset();
        }

        telemetry.addData("CURRENT POS: ", localizer.getPos().getX() + " " + localizer.getPos().getY() + " " + localizer.getPos().getR());
    }

    @Override
    public void stop() {

    }
}
