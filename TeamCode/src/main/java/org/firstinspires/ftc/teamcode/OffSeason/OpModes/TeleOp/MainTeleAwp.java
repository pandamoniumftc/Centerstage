package org.firstinspires.ftc.teamcode.OffSeason.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.OffSeason.Pipelines.PixelPipeline;
import org.firstinspires.ftc.teamcode.OffSeason.Robots.Po;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp (name="main tele op")
public class MainTeleAwp extends AbstractTeleOp {
    Po robot;
    public PixelPipeline pixelPipeline;
    public OpenCvCamera armCamera;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new Po(this);
        /*int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        armCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam1"), cameraMonitorViewId);

        pixelPipeline = new PixelPipeline();*/

        return robot;
    }

    @Override
    public void onInit() {
        /*armCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                armCamera.setPipeline(pixelPipeline);
                armCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the armCamera could not be opened
                 *
            }
        });*/

        robot.deadWheels.localizer.setPos(Po.currentPos);

    }

    @Override
    public void onDriverUpdate() {
        robot.ctrlHub.pullBulkData();
        robot.expHub.pullBulkData();
        robot.queue.update();
    }

    @Override
    public void onStop() {
        //armCamera.stopStreaming();
        //armCamera.closeCameraDevice();
    }

}