package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

import java.io.IOException;

public class Intake extends AbstractSubsystem {
    BaoBao robot;
    public DcMotor intakeMotor;
    public Toggle intake_toggle = new Toggle(false);
    DcMotorEx iMotor;
    public Intake(AbstractRobot robot, String intakeM) {
        super(robot);
        this.robot = (BaoBao) robot;

        intakeMotor = robot.hardwareMap.get(DcMotor.class, intakeM);

    }

    @Override
    public void init() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        intakeMotor.setPower(robot.gamepad2.left_stick_y);
    }

    @Override
    public void stop() {

    }
}
