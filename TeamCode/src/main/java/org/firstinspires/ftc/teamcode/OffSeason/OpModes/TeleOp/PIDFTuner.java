package org.firstinspires.ftc.teamcode.OffSeason.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.queue.MotorPowerTask;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.tasks.MotorPositionTask;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.OffSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.OffSeason.Robots.Po;
import org.openftc.easyopencv.OpenCvCamera;

@Config
@TeleOp (name="pidf tuner")
public class PIDFTuner extends OpMode {
    Po robot;
    public static double kp = 0.0001, ki = 0, kd = 0;
    public static double target = 0;

    /**
     *
     */
    @Override
    public void init() {
        this.robot = new Po(this);
        /*int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        slideCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam1"), cameraMonitorViewId);

        teamElementPipeline = new TeamElementPipeline();
        slideCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                slideCamera.setPipeline(teamElementPipeline);
                slideCamera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the slidesCamera could not be opened
                 *
            }
        });

        FtcDashboard.getInstance().startCameraStream(slideCamera, 0);*/
        robot.arm.armMotor.positionController = new MotorPositionController(1.57, robot.arm.armMotor, robot.arm.armEncoder, false);

        robot.arm.armMotor.enablePositionPID(true);
        robot.arm.armMotor.positionController.getPid().setPGain(kp);
        robot.arm.armMotor.positionController.getPid().setIGain(ki);
        robot.arm.armMotor.positionController.getPid().setDGain(kd);
        robot.arm.armMotor.positionController.setScale(3.0);

        robot.queue.addTask(
                new MotorPositionTask(
                        1.57,
                        robot.arm.armMotor,
                        true,
                        0.05F
                )
        );
    }

    /**
     *
     */
    @Override
    public void loop() {
        robot.ctrlHub.pullBulkData();
        robot.expHub.pullBulkData();
        robot.queue.update();
        telemetry.addData("rotation: ", robot.arm.armMotor.positionController.getPosition());
        telemetry.addData("power: ", robot.arm.armMotor.power);
        telemetry.addData("P: ", robot.arm.armMotor.positionController.getPid().getP());
        telemetry.addData("pos: ", robot.arm.armMotor.positionController.getHomedEncoderPosition());
        telemetry.addData("complete: ", robot.arm.armMotor.positionController.isAtGoal(0.05F));
        telemetry.addData("idle: ", robot.queue.getIdle());
        telemetry.update();
    }
}