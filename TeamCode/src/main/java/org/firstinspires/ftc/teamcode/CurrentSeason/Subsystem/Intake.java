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
    public Servo intakeServo;
    public Intake(AbstractRobot robot, String intakeM, String intakeS) {
        super(robot);
        this.robot = (BaoBao) robot;

        intakeMotor = robot.hardwareMap.get(DcMotor.class, intakeM);
        intakeServo = robot.hardwareMap.get(Servo.class, intakeS);

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
        intakeMotor.setPower(-robot.gamepad2.left_stick_y * .5);
        double servoPower = robot.gamepad2.left_stick_y * 0.5 + .5;
        intakeServo.setPosition(servoPower);
    }

    @Override
    public void stop() {

    }
}
