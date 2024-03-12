package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.ClawArm;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.LinearSlides;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.RoadRunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.airplane;
import org.opencv.core.Point;

public class Po extends AbstractRobot {
    public RoadRunnerMecanumDrive mecanum;
    public LinearSlides slides;
    public ClawArm arm;
    public airplane airplane;
    public enum robotState {
        NEUTRAL(false, 0.5),
        INTAKING(false, 0.5),
        GRABBED_PIXEL(false, 0.5),
        DEPOSIT(true, .83),
        HANGING(true, .83);
        private final boolean lifted;
        private final double pivotPos;

        robotState(boolean lifted, double pivotPos) {
            this.lifted = lifted;
            this.pivotPos = pivotPos;
        }
        public boolean getLifted() { return lifted; }
        public double getPivotPos() { return pivotPos; }
    }
    public robotState state;
    private long initTime;
    public static Pose2d currentPos = new Pose2d();
    private final Point[] MovementPoints = new Point[] {
            new Point(0, 0.2),
            new Point(1, 0.2),
            new Point(1, 0.2),
            new Point(1, .66)
    };
    private static final Point[][] ArmPoints = new Point[][] {
                new Point[] {
                        new Point(0, 0),
                        new Point(0, 0),
                        new Point(0, 0),
                        new Point(.5, 415)
                },
                new Point[] {
                        new Point(.5, 415),
                        new Point(1, 830),
                        new Point(1, 830),
                        new Point(1, 830)
            }
    };
    private static final Point[][] SlidesPoint = new Point[][] {
            new Point[] {
                    new Point(0, 0),
                    new Point(0, 0),
                    new Point(0, 0),
                    new Point(.5, .5)
            },
            new Point[] {
                    new Point(.5, .5),
                    new Point(1, 1),
                    new Point(1, 1),
                    new Point(1, 1)
            }
    };
    public Po(OpMode opMode) {
        super(opMode);
        mecanum = new RoadRunnerMecanumDrive(this, MovementPoints);
        slides = new LinearSlides(this, "lsm", "rsm", SlidesPoint);
        arm = new ClawArm(this, "am", "ps", "cs1", "cs2", ArmPoints);
        airplane = new airplane(this, "as");

        state = robotState.NEUTRAL;
    }
    public boolean wait(double time, boolean startCondition) {
        if (startCondition) {
            return System.currentTimeMillis() - initTime > time * 1E3;
        }
        else {
            initTime = System.currentTimeMillis();
        }
        return false;
    }
}