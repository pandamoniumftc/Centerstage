package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.RoadRunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.airplane;

public class BaoBao extends AbstractRobot {
    public RoadRunnerMecanumDrive mecanum;
    public Intake intake;
    public Outtake outtake;
    public airplane airplane;
    public enum robotState {
        NEUTRAL,
        INTAKING,
        WAITING_TO_GRAB,
        GRABBED_PIXEL,
        WAITING_TO_DEPOSIT,
        DEPOSITED
    }
    public robotState state;

    public BaoBao(OpMode opMode) {
        super(opMode);
        mecanum = new RoadRunnerMecanumDrive(this);
        outtake = new Outtake(this, "lsm", "rsm", "ps", "lSensor", "rSensor", "cs1", "cs2");
        intake = new Intake(this, "intakeM");
        airplane = new airplane(this, "as");
    }
}
