package org.firstinspires.ftc.teamcode.OffSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.OffSeason.Subsystem.ClawArm;
import org.firstinspires.ftc.teamcode.OffSeason.Subsystem.DeadWheels;
import org.firstinspires.ftc.teamcode.OffSeason.Subsystem.MecanumDrive;
import org.opencv.core.Point;

public class Po extends AbstractRobot {
    /*
     *
     *    /--------------\
     *  (e,3)   _fl_   (c,1)
     *    |     ----     |
     *    | |b|      |f| |
     *    | |l|      |r| |
     *    |              |
     *  (e,1)          (c,3)
     *    \--------------/
     *
     */
    public CuttleRevHub ctrlHub, expHub;
    public ClawArm arm;
    public MecanumDrive drive;
    public DeadWheels deadWheels;
    public static Pose currentPos = new Pose();
    public enum robotState {
        NEUTRAL,
        INTAKING,
        GRABBED_PIXEL,
        DEPOSIT,
        HANGING;
    }
    public robotState state;
    private static final Point[][] DriveCurve = new Point[][] {
            new Point[] {
                    new Point(0, 0),
                    new Point(0, 0.2),
                    new Point(0, 0.2),
                    new Point(0.05, 0.2)
            },
            new Point[] {
                    new Point(0.05, 0.2),
                    new Point(0.25, 0.2)
            },
            new Point[] {
                    new Point(0.25, 0.2),
                    new Point(1.0, 0.2),
                    new Point(1.0, 0.2),
                    new Point(1.0, 1.0)
            }
    };
    public Po(OpMode opMode) {
        super(opMode);
        ctrlHub = new CuttleRevHub(hardwareMap, "Control Hub");
        expHub = new CuttleRevHub(hardwareMap, "Expansion Hub 2");
        arm = new ClawArm(this, 0, 2, 0, 1);
        drive = new MecanumDrive(this, -3, 1, -1, 3, DriveCurve);
        deadWheels = new DeadWheels(this, -1, -1, 3, 8192, 17.5, 243.84, 1.0);

        //state = robotState.NEUTRAL;
    }
}