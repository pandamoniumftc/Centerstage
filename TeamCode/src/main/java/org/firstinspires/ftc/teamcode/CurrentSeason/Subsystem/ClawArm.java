package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.opencv.core.Point;

import java.io.IOException;

public class ClawArm extends AbstractSubsystem {
    Po robot;
    public DcMotor armMotor;
    public Servo pivotServo, clawServo1, clawServo2;
    public Toggle leftOpened = new Toggle(false);
    public Toggle rightOpened = new Toggle(false);
    public Toggle lifted = new Toggle(false);
    public Toggle finiteState = new Toggle(true);
    public double armAngle;
    public CurveSequence ArmProfile;
    private final double[] servoposition = new double[] {0.5, 0.43, 0.05, .12};
    public PIDFController armController;
    private final double armPosOffset = 100.0;
    private final double encoderResolution = 537.7;
    private final double gearRatio = 3;
    private long initTime = 0;

    public ClawArm(AbstractRobot robot, String am, String ps, String cs1, String cs2, Point[] ArmCurve) {
        super(robot);
        this.robot = (Po) robot;

        armMotor = robot.hardwareMap.get(DcMotor.class, am);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armController = new PIDFController(0.0005, 0.002, 0.008, 0.005);
        armAngle = ((-armPosOffset / encoderResolution) * 2 * Math.PI) / gearRatio;

        pivotServo = robot.hardwareMap.get(Servo.class, ps);
        clawServo1 = robot.hardwareMap.get(Servo.class, cs1);
        clawServo2 = robot.hardwareMap.get(Servo.class, cs2);

        pivotServo.setDirection(Servo.Direction.REVERSE);
        this.ArmProfile = CurveSequence.init(ArmCurve);
    }

    @Override
    public void init() throws IOException {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driverLoop() {
        // enables or disables buttons during the different states

        if (robot.state == Po.robotState.NEUTRAL || robot.state == Po.robotState.INTAKING || robot.state == Po.robotState.WAITING_TO_DEPOSIT) {
            leftOpened.updateState(robot.gamepad2.left_bumper);
            rightOpened.updateState(robot.gamepad2.right_bumper);
        }

        if (robot.state == Po.robotState.GRABBED_PIXEL || robot.state == Po.robotState.DEPOSITED) {
            lifted.updateState(robot.gamepad2.left_stick_button);
        }

        // presses left and right bumper to open claw and go intake

        if (robot.state == Po.robotState.NEUTRAL && leftOpened.state && rightOpened.state) {
            initTime = 0;
            robot.state = Po.robotState.INTAKING;
        }

        // presses left and right bumper again to close claw and lift claw

        if (robot.state == Po.robotState.INTAKING && !leftOpened.state && !rightOpened.state) {
            if (System.currentTimeMillis() - initTime == System.currentTimeMillis()) {
                initTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - initTime > 1500) {
                robot.state = Po.robotState.GRABBED_PIXEL;
            }
        }

        // presses left stick button to lift arm

        if (robot.state == Po.robotState.GRABBED_PIXEL && lifted.state) {
            initTime = 0;
            robot.state = Po.robotState.WAITING_TO_DEPOSIT;
        }

        // presses left and right bumper to release pixels
        // once claw is completely open, goes on to the next stage

        if (robot.state == Po.robotState.WAITING_TO_DEPOSIT && leftOpened.state && rightOpened.state) {
            if (System.currentTimeMillis() - initTime == System.currentTimeMillis()) {
                initTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - initTime > 3000) {
                leftOpened.state = false;
                rightOpened.state = false;
                robot.state = Po.robotState.DEPOSITED;
            }

        }

        // presses left stick button to lower arm (prevents slamming, will remove once we have motion profiling)

        if (robot.state == Po.robotState.DEPOSITED && !robot.arm.lifted.state) {
            robot.state = Po.robotState.NEUTRAL;
        }

        // will set arm and claw position based on the current state

        setArmPosition(robot.state.getArmPos());
        pivotServo.setPosition(robot.state.getPivotPos());
        openClaw(leftOpened.state, rightOpened.state);
        closeClaw(!leftOpened.state, !rightOpened.state);

        telemetry.addData("robot state: ", robot.state);
        telemetry.addData("lifted: ", lifted.state);
        telemetry.addData("left: ", leftOpened.state);
        telemetry.addData("right: ", rightOpened.state);
        telemetry.addData("arm motor: ", armMotor.getCurrentPosition());
    }

    public void openClaw(boolean leftOpened, boolean rightOpened) {
        if (leftOpened) {clawServo1.setPosition(servoposition[1]);}
        if (rightOpened) {clawServo2.setPosition(servoposition[3]);}
    }
    public void closeClaw(boolean leftClosed, boolean rightClosed) {
        if (leftClosed) {clawServo1.setPosition(servoposition[0]);}
        if (rightClosed) {clawServo2.setPosition(servoposition[2]);}
    }
    public void setArmPosition(int targetPos) {
        double armAngle = ((((double) armMotor.getCurrentPosition() - armPosOffset) / encoderResolution) * 2 * Math.PI) / gearRatio;
        double pid = armController.calculate(armMotor.getCurrentPosition(), targetPos, armAngleFeedforward(armAngle));
        armMotor.setPower(this.ArmProfile.evaluate(Math.abs(Range.clip(pid, -1, 1))) * Math.signum(pid));
    }
    public double armAngleFeedforward(double armAngle) {
        if (armAngle <= (Math.PI/2) && armAngle >= 0) {
            return Math.cos(armAngle);
        }
        else if (armAngle >= (Math.PI/2)) {
            return Math.cos(armAngle);
        }
        else if (armAngle < 0 && armMotor.getCurrentPosition() != 0){
            return (Math.cos(armAngle) + 1);
        }
        else {
            return 0;
        }
    }
    @Override
    public void stop() {

    }
}