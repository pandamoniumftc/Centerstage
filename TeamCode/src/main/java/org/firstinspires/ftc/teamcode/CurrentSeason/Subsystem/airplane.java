package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;

import java.io.IOException;

public class airplane extends AbstractSubsystem {
    Servo airplaneservo;
    double [] servoposition = new double[] {0 , 1};
    public airplane(AbstractRobot robot, String as) {
        super(robot);
        airplaneservo = robot.hardwareMap.get(Servo.class, as);
    }

    @Override
    public void init() throws IOException {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        airplaneservo.setPosition(robot.gamepad2.a ? servoposition[1]:servoposition[0]);
    }

    @Override
    public void stop() {

    }
}
