package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name="detect auto")
public class DetectionAuto extends AbstractAutonomous {
    public Po robot;
    public OpenCvCamera slidesCamera;
    public TeamElementPipeline teamElementPipeline;
    Trajectory middle, left, right, backdrop;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new Po(this);

        /*int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        slidesCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);*/

        teamElementPipeline = new TeamElementPipeline();

        return robot;
    }
    
    @Override
    public void onInit() {
        /*slidesCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                slidesCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                slidesCamera.setPipeline(teamElementPipeline);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the slidesCamera could not be opened
                 /
            }
        });*/
    }

    @Override
    public void autonomous() {
        middle = robot.mecanum.drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(20, 10))
                .build();
        left = robot.mecanum.drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(20, 10))
                .build();
        right = robot.mecanum.drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(20, 10))
                .build();
        backdrop = robot.mecanum.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .build();

        waitForStart();

        if (opModeIsActive()) {

            robot.mecanum.drive.followTrajectory(middle);

            TeamElementPipeline.objectLocation location = TeamElementPipeline.location;

            if (location == TeamElementPipeline.objectLocation.MIDDLE) {
                robot.mecanum.drive.followTrajectory(middle);
            } else if (location == TeamElementPipeline.objectLocation.LEFT) {
                robot.mecanum.drive.followTrajectory(left);
            } else {
                robot.mecanum.drive.followTrajectory(right);
            }

        }
    }
}
