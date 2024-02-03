package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

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
        NEUTRAL(85, 0.25),
        INTAKING(0, 0.3),
        GRABBED_PIXEL(85, 0.25),
        WAITING_TO_DEPOSIT(650, 0.7),
        DEPOSITED(625, 0.7),
        HANGING(50, 0.3);

        private final int armPos;
        private final double pivotPos;

        robotState(int armPos, double pivotPos) {
            this.armPos = armPos;
            this.pivotPos = pivotPos;
        }

        public int getArmPos() { return armPos; }
        public double getPivotPos() { return pivotPos; }
    }
    public robotState state;
    private final Point[] MovementPoints = new Point[] {
            new Point(0, 0.125),
            new Point(0, 1),
            new Point(1, 0.1875),
            new Point(1, 1)
    };
    private final Point[] ArmPoints = new Point[] {
            new Point(0, 0),
            new Point(1, 0),
            new Point(0, 1),
            new Point(1, 1)
    };

    public Po(OpMode opMode) {
        super(opMode);
        mecanum = new RoadRunnerMecanumDrive(this, MovementPoints);
        slides = new LinearSlides(this, "lsm", "rsm");
        arm = new ClawArm(this, "am", "ps", "cs1", "cs2", ArmPoints);
        //airplane = new airplane(this, "as");

        state = robotState.NEUTRAL;
    }
}
