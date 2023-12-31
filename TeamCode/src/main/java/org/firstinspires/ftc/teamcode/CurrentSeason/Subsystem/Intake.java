package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

import java.io.IOException;

public class Intake extends AbstractSubsystem {
    BaoBao robot;
    public DcMotor intakeMotor;
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
        intakeMotor.setPower(-robot.gamepad2.left_stick_y * .8);
    }

    @Override
    public void stop() {

    }
}
