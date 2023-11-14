package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.RoadRunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.GamepadSettings;
import org.opencv.core.Point;

public class BaoBao extends AbstractRobot {
    public RoadRunnerMecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public BaoBao(OpMode opMode) {
        super(opMode);
        drive = new RoadRunnerMecanumDrive(this);
        outtake = new Outtake(this, "lsm", "rsm", "lts", "rts", "frs", "brs", "fSensor", "bSensor");
        intake = new Intake(this, "intakeM", "intakeS");
    }
}
