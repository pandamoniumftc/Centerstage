package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import  com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.PixelPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CurrentSeason.Util.PIDFController;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Timer;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp (name="old pidf tuner")
public class PIDFTuner extends OpMode {
    Po robot;
    PIDFController controller;
    public static double kp = 0.004, ki = 0.001, kd = 0.0001, kf = 0.0001;
    public static double Kv = 0.0002, Ka = 0.00005;
    public static int target = 0;
    public static double power;
    public static double pos = Po.robotState.NEUTRAL.getPivotPos();
    public double armAngle;
    public double armPosition, slidePosition;
    public double armVelocity, slideVelocity;
    public double armAcceleration, slideAcceleration;
    public static int animTime;
    double encoderResolution = 537.7;
    double gearRatio = 3;
    public static int lift = 0;
    public Timer timer = new Timer();
    public PixelPipeline pixelPipeline;
    public OpenCvCamera armCamera;

    @Override
    public void init() {
        this.robot = new Po(this);
        //controller = new PIDFController(kp, ki, kd, kf);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //robot.slides.lSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.slides.rSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        armCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam1"), cameraMonitorViewId);

        pixelPipeline = new PixelPipeline();
        armCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                armCamera.setPipeline(pixelPipeline);
                armCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the slidesCamera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(armCamera, 0);
    }

    @Override
    public void start() {
        //robot.slides.lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.slides.rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (lift == 1) {
            robot.mecanum.align(pixelPipeline.getError(), kp);
        }
        //robot.slides.lSlideMotor.setPower(controller.calculate(robot.slides.lSlideMotor.getCurrentPosition(), target, 1));
        //robot.slides.rSlideMotor.setPower(controller.calculate(robot.slides.lSlideMotor.getCurrentPosition(), target, 1));
        /*timer.newSlideTask(animTime, target);

        if ((target > timer.slides_task[4] ? 1 : 0) == timer.slides_task[2] && timer.slides_task[0] != 0) {
            double t = (Range.clip(timer.x, 0, 1) - robot.slides.SlidesProfile.minX)/(robot.slides.SlidesProfile.maxX - robot.slides.SlidesProfile.minX);

            slidePosition = robot.slides.SlidesProfile.evaluate(t) [0];
            slideVelocity = robot.slides.SlidesProfile.evaluate(t) [1];
            slideAcceleration = robot.slides.SlidesProfile.evaluate(t) [2];

            robot.slides.lSlideMotor.setPower(controller.calculate(robot.slides.lSlideMotor.getCurrentPosition(), slidePosition * timer.slides_task[3] + (1.0 - slidePosition) * timer.slides_task[4], 1, slideVelocity, slideAcceleration));
            robot.slides.rSlideMotor.setPower(controller.calculate(robot.slides.lSlideMotor.getCurrentPosition(), slidePosition * timer.slides_task[3] + (1.0 - slidePosition) * timer.slides_task[4], 1, slideVelocity, slideAcceleration));
        }*/

        /*double pid = controller.calculate(robot.arm.armMotor.getCurrentPosition(), target, robot.arm.armAngleFeedforward(armAngle));
        robot.arm.armMotor.setPower(robot.arm.ArmProfile.evaluate(Math.abs(Range.clip(pid, -1, 1)))[0] * Math.signum(pid));*/
        /*boolean up = lift == 1;

        timer.newArmTask(animTime, up);

        if ((up ? 1 : 0) == timer.arm_task[2] && timer.arm_task[0] != 0) {
            double t = (Range.clip(timer.x, 0, 1) - robot.arm.ArmProfile.minX)/(robot.arm.ArmProfile.maxX - robot.arm.ArmProfile.minX);

            double armPosition = robot.arm.ArmProfile.evaluate(timer.arm_task[2] == 1 ? t : 1 - t) [0];
            double armVelocity = robot.arm.ArmProfile.evaluate(timer.arm_task[2] == 1 ? t : 1 - t) [1];
            double armAcceleration = robot.arm.ArmProfile.evaluate(timer.arm_task[2] == 1 ? t : 1 - t) [2];
            double armAngle = ((((double) robot.arm.armMotor.getCurrentPosition() - 100) / encoderResolution) * 2 * Math.PI) / gearRatio;

            robot.arm.armMotor.setPower(controller.calculate(robot.arm.armMotor.getCurrentPosition(), armPosition, robot.arm.armAngleFeedforward(armAngle), armVelocity, armAcceleration));
        }*/

        //robot.arm.armMotor.setTargetPosition(target);
        //robot.arm.armMotor.setPower(power);

        //robot.arm.pivotServo.setPosition(pos);
        //robot.airplane.airplaneservo.setPosition(pos);

        //robot.slides.lSlideMotor.setPower(power);
        //robot.slides.rSlideMotor.setPower(power);

        //telemetry.addData("left pos: ", robot.slides.lSlideMotor.getCurrentPosition());
        //telemetry.addData("right pos: ", robot.slides.rSlideMotor.getCurrentPosition());
        //telemetry.addData("left: ", Lpidf);
        //telemetry.addData("right: ", Rpidf);
        //telemetry.addData("time: ", timer.x);
        //telemetry.addData("pos: ", armPosition);
        //telemetry.addData("velo: ", armVelocity);
        //telemetry.addData("accel: ", armAcceleration);
        //telemetry.addData("arm task: ", timer.arm_task[0] + ", " + timer.arm_task[1] + ", " + timer.arm_task[2]/* + ", " + timer.arm_task[3] + ", " + timer.arm_task[4]*/);
        //telemetry.addData("complete: ", timer.isComplete);
        //telemetry.addData("arm pos: ", robot.arm.armMotor.getCurrentPosition());
        //telemetry.addData("angle: ", armAngle);
        //telemetry.addData("target: ", target);
        //telemetry.addData("pid: ", robot.arm.armMotor.getPower());
        //telemetry.addData("error: ", pixelPipeline.getError(), kp);
        telemetry.addData("correction: ", pixelPipeline.getError() * kp);
        telemetry.update();
        //telemetry.addData("left dir", robot.slides.lSlideMotor.getDirection());
        //telemetry.addData("right dir", robot.slides.rSlideMotor.getDirection());
    }
}