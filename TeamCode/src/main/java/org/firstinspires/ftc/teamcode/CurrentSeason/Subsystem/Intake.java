package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

import java.io.IOException;

public class Intake extends AbstractSubsystem {
    DcMotor intakeMotor;
    Toggle intake_toggle = new Toggle(false);
    DcMotorEx iMotor;
    public Intake(AbstractRobot robot, String intakeM) {
        super(robot);

        intakeMotor = robot.hardwareMap.get(DcMotor.class, intakeM);
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

    @Override
    public void stop() {

    }
}
