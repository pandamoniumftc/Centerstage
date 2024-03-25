package org.firstinspires.ftc.teamcode.OffSeason.Subsystem;

import static java.lang.Math.signum;

import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.OffSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.OffSeason.Util.Toggle;
import org.opencv.core.Point;

import java.io.IOException;

public class LinearSlides extends AbstractSubsystem {
    Po robot;
    public CuttleMotor lSlideMotor, rSlideMotor;
    public CuttleEncoder slideEncoder;
    public final int[] slidesBounds = new int[] {
            /* min */ 0,
            /* max */ 2000};
    public CurveSequence SlidesProfile;
    public Toggle liftSlide, lowerSlide, hang;
    public int targetPos;
    public double strength;
    public LinearSlides(AbstractRobot robot, int lsm, int rsm, Point[][] SlidesPoint) {
        super(robot);
        this.robot = (Po) robot;

        lSlideMotor = this.robot.expHub.getMotor(Math.abs(lsm));
        rSlideMotor = this.robot.ctrlHub.getMotor(Math.abs(rsm));

        lSlideMotor.setDirection(signum(lsm) == 1 ? Direction.FORWARD : Direction.REVERSE);
        rSlideMotor.setDirection(signum(rsm) == 1 ? Direction.FORWARD : Direction.REVERSE);

        liftSlide = new Toggle(false);
        lowerSlide = new Toggle(false);
        hang = new Toggle(false);

        targetPos = 0;

        this.SlidesProfile = CurveSequence.init(SlidesPoint);
    }

    @Override
    public void init() throws IOException {
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {

    }
    public void setSlidesPosition(int targetPos) {
    }
    @Override
    public void stop() {

    }
}