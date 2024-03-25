package org.firstinspires.ftc.teamcode.OffSeason.Subsystem;

import static java.lang.Math.signum;

import com.roboctopi.cuttlefish.components.Motor;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.OffSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.OffSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.OffSeason.Task.ArmTask;
import org.firstinspires.ftc.teamcode.OffSeason.Task.ClawTask;

import java.io.IOException;
import java.util.ArrayList;

public class ClawArm extends AbstractSubsystem {
    Po robot;
    public CuttleMotor armMotor;
    public CuttleServo pivotServo, clawServo1, clawServo2;
    public CuttleServo[] claw;
    public CuttleEncoder armEncoder;
    public Toggle left, right;
    private ArrayList<Double> presetPos;
    public int INTAKE_LEVEL = 0, DEPOSIT_LEVEL_1 = 1, DEPOSIT_LEVEL_2 = 2, DEPOSIT_LEVEL_3 = 3;
    public ClawArm(AbstractRobot robot, int am, int ps, int cs1, int cs2) {
        super(robot);
        this.robot = (Po) robot;

        armEncoder = new CuttleEncoder(this.robot.expHub, am, 537.7);
        armMotor = new CuttleMotor(this.robot.expHub, Math.abs(am));
        armMotor.setDirection(Direction.FORWARD);

        //armMotor.positionController = new MotorPositionController(3.14, armMotor, armEncoder, false);

        /*armMotor.positionController.getPid().setPGain(0.01);
        armMotor.positionController.getPid().setIGain(0);
        armMotor.positionController.getPid().setDGain(0);
        armMotor.positionController.getPid().setILimit(1);
        armMotor.positionController.setLowerLimitEnabled(true);
        armMotor.positionController.setUpperLimitEnabled(true);
        armMotor.positionController.setLowerLimit(0.0);
        armMotor.positionController.setUpperLimit(7.0 * Math.PI / 6.0);*/
        //armMotor.positionController.setScale(3.0);

        pivotServo = this.robot.expHub.getServo(ps);
        clawServo1 = this.robot.expHub.getServo(cs1);
        clawServo2 = this.robot.expHub.getServo(cs2);

        clawServo1.addPreset(0.48); // close pos
        clawServo1.addPreset(0.43); // open pos
        clawServo2.addPreset(0.07); // close pos
        clawServo2.addPreset(0.12); // open pos

        pivotServo.addPreset(0.5); // down pos
        pivotServo.addPreset(0.83); // lvl 1
        pivotServo.addPreset(0.83); // lvl 2
        pivotServo.addPreset(0.83); // lvl 3

        presetPos = new ArrayList<>();

        presetPos.add(0.0); // down pos
        presetPos.add(5.0 * Math.PI / 6.0); // lvl 1
        presetPos.add(Math.PI * 1); // lvl 2
        presetPos.add(7.0 * Math.PI / 6.0); // lvl 3

        claw = new CuttleServo[] {clawServo1, clawServo2};

        right = new Toggle(false);
        left = new Toggle(false);
    }

    @Override
    public void init() throws IOException {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {

        /*switch (robot.state) {
            case NEUTRAL:

                if (armController.isAtGoal(0.05F)) {
                    armController.disable();
                }
            case INTAKING:
                setArmPosition(INTAKE_LEVEL);
            case GRABBED_PIXEL:
                setArmPosition(INTAKE_LEVEL);
            case DEPOSIT:
                if (left.state) {
                    controlClaw(true, false);
                    robot.queue.addTask(new DelayTask(1500));
                    controlClaw(false, false);
                }
                else {
                    robot.queue.addTask(new DelayTask(2000));
                    setArmPosition(INTAKE_LEVEL);
                }
            case HANGING:
                setArmPosition(DEPOSIT_LEVEL_3);
        }*/
    }

    public void controlClaw(boolean leftOpened, boolean rightOpened) {
        robot.queue.addTask(new ClawTask(claw, leftOpened, rightOpened));
    }
    public void setArmPosition(int level) {
        robot.queue.addTask(new ArmTask(level, armMotor, pivotServo, presetPos));
    }
    @Override
    public void stop() {

    }
}