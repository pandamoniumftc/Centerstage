package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous (name="detect auto")
public class DetectionAuto extends AbstractAutonomous {
    BaoBao robot;
    OpenCvCamera slidesCamera;
    Trajectory forward;
    Trajectory left;
    Trajectory right;



    @Override
    public void onInit() {
        /*int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        slidesCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        //set pipeline
        slidesCamera.setPipeline(new TeamElementPipeline());

        slidesCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                slidesCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {
                /*
                 * This will be called if the slidesCamera could not be opened

            }
        });*/

        Pose2d startPos = new Pose2d(65, 12, Math.toRadians(0));

        robot.mecanum.drive.setPoseEstimate(startPos);

    }
    @Override
    public void autonomous() {
        forward = robot.mecanum.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(20)
                .build();
        left = robot.mecanum.drive.trajectoryBuilder(forward.end())
                .strafeLeft(20)
                .build();
        right = robot.mecanum.drive.trajectoryBuilder(forward.end())
                .strafeRight(20)
                .build();

        //slidesCamera.setPipeline(new TeamElementPipeline());

        waitForStart();

        if (opModeIsActive()) {

            robot.mecanum.drive.followTrajectory(forward);

            TeamElementPipeline.objectLocation location = TeamElementPipeline.location;

            if (location == TeamElementPipeline.objectLocation.MIDDLE) {
                robot.mecanum.drive.followTrajectory(forward);
            }
            else if (location == TeamElementPipeline.objectLocation.LEFT) {
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
        robot = new BaoBao(this);
        return robot;
    }
}
