package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.MotionProfile;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.CurvesPort.CurveSequence;
import org.firstinspires.ftc.teamcode.CurvesPort.VariantDegreeBezier;
import org.opencv.core.Point;

import java.io.IOException;

public class ClawArm extends AbstractSubsystem {
    Po robot;
    public DcMotor armMotor;
    public Servo pivotServo, clawServo1, clawServo2;
    public Toggle leftOpened = new Toggle(false);
    public Toggle rightOpened = new Toggle(false);
    public Toggle lifted = new Toggle(false);
    public Toggle finiteState = new Toggle(false);
    public double armAngle;
    public CurveSequence ArmProfile;
    private final double[] servoposition = new double[] {0.5, 0.44, 0.05, .11};

    public PIDFController armController;
    private final double armPosOffset = 85.0;
    private final double encoderResolution = 537.7;
    private final double gearRatio = 3;

    public ClawArm(AbstractRobot robot, String am, String ps, String cs1, String cs2, Point[] ArmCurve) {
        super(robot);
        this.robot = (Po) robot;

        armMotor = robot.hardwareMap.get(DcMotor.class, am);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armController = new PIDFController(1, 0.0075, 0.75, 0.002);
        armAngle = ((-armPosOffset / encoderResolution) * 2 * Math.PI) / gearRatio;

        //pivotServo = robot.hardwareMap.get(Servo.class, ps);
        clawServo1 = robot.hardwareMap.get(Servo.class, cs1);
        clawServo2 = robot.hardwareMap.get(Servo.class, cs2);

        this.ArmProfile = CurveSequence.init(ArmCurve);
    }

    @Override
    public void init() throws IOException {armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

    @Override
    public void start() {}

    @Override
    public void driverLoop() {
        finiteState.updateState(robot.gamepad2.right_stick_button);
        if (finiteState.state) {
            if (robot.state == Po.robotState.NEUTRAL || robot.state == Po.robotState.INTAKING || robot.state == Po.robotState.WAITING_TO_DEPOSIT) {
                leftOpened.updateState(robot.gamepad2.left_bumper);
                rightOpened.updateState(robot.gamepad2.right_bumper);
            }

            if (robot.state == Po.robotState.GRABBED_PIXEL || robot.state == Po.robotState.DEPOSITED) {
                lifted.updateState(robot.gamepad2.left_stick_button);
            }

            if (robot.state == Po.robotState.NEUTRAL && leftOpened.state && rightOpened.state) {robot.state = Po.robotState.INTAKING;}

            if (robot.state == Po.robotState.INTAKING && !leftOpened.state && !rightOpened.state) {robot.state = Po.robotState.GRABBED_PIXEL;}

            if (robot.state == Po.robotState.GRABBED_PIXEL && lifted.state) {robot.state = Po.robotState.WAITING_TO_DEPOSIT;}

            if (robot.state == Po.robotState.WAITING_TO_DEPOSIT && leftOpened.state && rightOpened.state) {robot.state = Po.robotState.DEPOSITED;}

            if (robot.state == Po.robotState.DEPOSITED && !lifted.state) {robot.state = Po.robotState.NEUTRAL;}

            setArmPosition(robot.state.getArmPos());
            pivotServo.setPosition(robot.state.getPivotPos());
            openClaw(leftOpened.state, rightOpened.state);
            closeClaw(!leftOpened.state, !rightOpened.state);
        }
        else {
            leftOpened.updateState(robot.gamepad2.left_bumper);
            rightOpened.updateState(robot.gamepad2.right_bumper);
            lifted.updateState(robot.gamepad2.left_stick_button);

            setArmPosition(lifted.state ? 200 : 85);
            pivotServo.setPosition(lifted.state ? 0.45 : .5);
            openClaw(leftOpened.state, rightOpened.state);
            closeClaw(!leftOpened.state, !rightOpened.state);
        }


        telemetry.addData("robot state: ", robot.state);
        telemetry.addData("lifted: ", lifted.state);
        telemetry.addData("left: ", leftOpened.state);
        telemetry.addData("right: ", rightOpened.state);
        telemetry.addData("arm motor: ", armMotor.getCurrentPosition());
    }

    public void openClaw(boolean leftOpened, boolean rightOpened) {
        if (leftOpened) {clawServo1.setPosition(servoposition[1]);}
        if (rightOpened) {clawServo1.setPosition(servoposition[1]);}
    }
    public void closeClaw(boolean leftClosed, boolean rightClosed) {
        if (leftClosed) {clawServo1.setPosition(servoposition[0]);}
        if (rightClosed) {clawServo1.setPosition(servoposition[0]);}
    }
    public void setArmPosition(int targetPos) {
        /*double armAngle = ((((double) armMotor.getCurrentPosition() - armPosOffset) / encoderResolution) * 2 * Math.PI) / gearRatio;
        double pid = armController.calculate(armMotor.getCurrentPosition(), targetPos, armAngleFeedforward(armAngle));
        armMotor.setPower(this.ArmProfile.evaluate(Math.abs(Range.clip(pid, -1, 1))) * Math.signum(pid));*/
        double x = MotionProfile.motionProfile_position    (10,10,.5,2);
        double v = MotionProfile.motionProfile_velocity    ();
        double a = MotionProfile.motionProfile_acceleration();
    }
    public double armAngleFeedforward(double armAngle) {
        if (armAngle <= (Math.PI/2) && armAngle >= 0) {
            return Math.cos(armAngle);
        }
        else if (armAngle >= (Math.PI/2)) {
            return -Math.cos(armAngle);
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
