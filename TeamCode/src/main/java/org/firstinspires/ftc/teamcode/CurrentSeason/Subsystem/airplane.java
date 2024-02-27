package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

import java.io.IOException;

public class airplane extends AbstractSubsystem {
    Po robot;
    public Servo airplaneservo;
    Toggle release = new Toggle(false);
    double [] servoposition = new double[] {0.5 , 0.75};
    private long initTime;
    public airplane(AbstractRobot robot, String as) {
        super(robot);
        airplaneservo = robot.hardwareMap.get(Servo.class, as);
    }

    @Override
    public void init() throws IOException {
        initTime = System.currentTimeMillis();
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        if (System.currentTimeMillis() - initTime > 120000) {
            release.updateState(robot.gamepad2.a);
        }
        airplaneservo.setPosition(servoposition[release.state ? 1 : 0]);
    }

    @Override
    public void stop() {

    }
}