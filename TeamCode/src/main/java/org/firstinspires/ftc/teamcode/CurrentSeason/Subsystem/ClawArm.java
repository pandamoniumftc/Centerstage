package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Timer;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.opencv.core.Point;

import java.io.IOException;

public class ClawArm extends AbstractSubsystem {
    Po robot;
    public DcMotor armMotor;
    public Servo pivotServo, clawServo1, clawServo2;
    public Toggle leftOpened, rightOpened, lifted, reset;
    public PIDFController armController;
    public CurveSequence ArmProfile;
    private final double[] servoposition = new double[] {0.48, 0.43, 0.07, .12};
    public double armAngle;
    private final double armPosOffset = 100.0, encoderResolution = 537.7, gearRatio = 3;
    public Timer timer;

    public ClawArm(AbstractRobot robot, String am, String ps, String cs1, String cs2, Point[][] ArmCurve) {
        super(robot);
        this.robot = (Po) robot;

        armMotor = robot.hardwareMap.get(DcMotor.class, am);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armController = new PIDFController(0.0006, 0.0003, 0.0001, 0.005, 0.0002, 0.00005);
        armAngle = ((-armPosOffset / encoderResolution) * 2 * Math.PI) / gearRatio;

        pivotServo = robot.hardwareMap.get(Servo.class, ps);
        clawServo1 = robot.hardwareMap.get(Servo.class, cs1);
        clawServo2 = robot.hardwareMap.get(Servo.class, cs2);

        pivotServo.setDirection(Servo.Direction.REVERSE);

        leftOpened = new Toggle(false);
        rightOpened = new Toggle(false);
        lifted = new Toggle(false);
        reset = new Toggle(false);

        timer = new Timer();

        this.ArmProfile = CurveSequence.init(ArmCurve);
    }

    @Override
    public void init() throws IOException {armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

    @Override
    public void start() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driverLoop() {
        setArmPosition(2, robot.state.getLifted());
        reset.updateState(robot.gamepad2.right_stick_button);
        pivotServo.setPosition(robot.state.getPivotPos());
        openClaw(leftOpened.state, rightOpened.state);
        closeClaw(!leftOpened.state, !rightOpened.state);

        if (!reset.state) {
            if (robot.state == Po.robotState.NEUTRAL) {
                if (lifted.state) {
                    leftOpened.state = false;
                    rightOpened.state = false;
                    lifted.state = false;
                }
                leftOpened.updateState(robot.gamepad2.left_bumper);
                rightOpened.updateState(robot.gamepad2.right_bumper);

                // presses left and right bumper to open claw and go intake
                if (leftOpened.state && rightOpened.state && !lifted.state) {robot.state = Po.robotState.INTAKING;}
            }
            else if (robot.state == Po.robotState.INTAKING) {
                leftOpened.updateState(robot.gamepad2.left_bumper);
                rightOpened.updateState(robot.gamepad2.right_bumper);

                // presses left and right bumper again to close claw and lift claw
                if (robot.wait(1.5, !leftOpened.state && !rightOpened.state)) {robot.state = Po.robotState.GRABBED_PIXEL;}
            }
            else if (robot.state == Po.robotState.GRABBED_PIXEL) {
                lifted.updateState(robot.gamepad2.left_stick_button);

                // presses left stick button to lift arm
                if (lifted.state) {robot.state = Po.robotState.DEPOSIT;}
            }
            else if (robot.state == Po.robotState.DEPOSIT) {
                leftOpened.updateState(robot.gamepad2.left_bumper);
                rightOpened.updateState(robot.gamepad2.right_bumper);
                lifted.updateState(robot.gamepad2.left_stick_button);

                // reset arm back to lowered pos
                if (!lifted.state && timer.isComplete && timer.arm_task[2] == 1) {robot.state = Po.robotState.GRABBED_PIXEL;}

                // presses left and right bumper to release pixels
                // once claw is completely open and slides is all the way down, goes on to the next stage
            }
            else if (robot.state == Po.robotState.HANGING) {
                leftOpened.state = false;
                rightOpened.state = false;
            }
        } else {
            // resets to neutral state
            if (robot.state == Po.robotState.DEPOSIT) {
                lifted.state = false;
                robot.state = Po.robotState.GRABBED_PIXEL;
            }
            else if (robot.state == Po.robotState.GRABBED_PIXEL) {
                leftOpened.state = false;
                rightOpened.state = false;
                if (robot.wait(1, timer.isComplete)) {
                    robot.state = Po.robotState.INTAKING;
                }
            }
            else if (robot.state == Po.robotState.INTAKING) {
                leftOpened.state = true;
                rightOpened.state = true;
                if (robot.wait(2, true)) {
                    robot.state = Po.robotState.NEUTRAL;
                }
            }
            else if (robot.state == Po.robotState.NEUTRAL) {
                leftOpened.state = false;
                rightOpened.state = false;
                reset.state = false;
            }
        }

        telemetry.addData("ROBOT STATE: ", robot.state.toString());
        telemetry.addData("ARM POS: ", armMotor.getCurrentPosition());
        telemetry.addData("ARM POS: ", timer.arm_task[0] + ", " + timer.arm_task[1] + ", " + timer.arm_task[2] + ", " + timer.isComplete);
        telemetry.addData("COMPLETED: ", timer.isComplete);
        telemetry.addData("lifted: ", lifted.state);
        telemetry.addData("left: ", leftOpened.state);
        telemetry.addData("right: ", rightOpened.state);
    }

    public void openClaw(boolean leftOpened, boolean rightOpened) {
        if (leftOpened) {clawServo1.setPosition(servoposition[1]);}
        if (rightOpened) {clawServo2.setPosition(servoposition[3]);}
    }
    public void closeClaw(boolean leftClosed, boolean rightClosed) {
        if (leftClosed) {clawServo1.setPosition(servoposition[0]);}
        if (rightClosed) {clawServo2.setPosition(servoposition[2]);}
    }
    public void setArmPosition(int animTime, boolean up) {
        timer.newArmTask(animTime, up);

        if ((up ? 1 : 0) == timer.arm_task[2] && timer.arm_task[0] != 0) {
            double t = (Range.clip(timer.x, 0, 1) - ArmProfile.minX)/(ArmProfile.maxX - ArmProfile.minX);

            double armPosition = ArmProfile.evaluate(timer.arm_task[2] == 1 ? t : 1 - t) [0];
            double armVelocity = ArmProfile.evaluate(timer.arm_task[2] == 1 ? t : 1 - t) [1];
            double armAcceleration = ArmProfile.evaluate(timer.arm_task[2] == 1 ? t : 1 - t) [2];
            double armAngle = ((((double) armMotor.getCurrentPosition() - armPosOffset) / encoderResolution) * 2 * Math.PI) / gearRatio;

            armMotor.setPower(armController.calculate(armMotor.getCurrentPosition(), armPosition, armAngleFeedforward(armAngle), armVelocity, armAcceleration));
        }
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