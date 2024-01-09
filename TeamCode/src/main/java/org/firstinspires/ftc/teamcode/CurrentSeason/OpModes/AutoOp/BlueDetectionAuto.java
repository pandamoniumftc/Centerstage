package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BlueDetectionAuto extends AbstractAutonomous {
    BaoBao robot;
    OpenCvCamera camera;
    Trajectory forward;
    Trajectory left;
    Trajectory right;
    @Override
    public void onInit() {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        //set pipeline
        camera.setPipeline(new TeamElementPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
    @Override
    public void autonomous() {
        forward = robot.mecanum.drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();
        left = robot.mecanum.drive.trajectoryBuilder(forward.end())
                .strafeLeft(20)
                .build();
        right = robot.mecanum.drive.trajectoryBuilder(forward.end())
                .strafeRight(20)
                .build();

        camera.setPipeline(new TeamElementPipeline());

        waitForStart();

        if (opModeIsActive()) {

            TeamElementPipeline.objectLocation location = TeamElementPipeline.location;

            if (TeamElementPipeline.location == TeamElementPipeline.objectLocation.MIDDLE) {
                robot.mecanum.drive.followTrajectory(forward);
            }
            else if (TeamElementPipeline.location == TeamElementPipeline.objectLocation.LEFT) {
                robot.mecanum.drive.followTrajectory(forward);
                robot.mecanum.drive.followTrajectory(left);
            }
            else {
                robot.mecanum.drive.followTrajectory(forward);
                robot.mecanum.drive.followTrajectory(right);
            }

        }

    }

    @Override
    public AbstractRobot instantiateRobot() {
        return null;
    }
}
