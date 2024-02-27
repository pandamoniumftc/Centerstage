package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.PipelineRecordingParameters;

@Autonomous (name="detect auto")
public class DetectionAuto extends AbstractAutonomous {
    public Po robot;
    public static OpenCvCamera slidesCamera;
    public TeamElementPipeline teamElementPipeline;
    Trajectory placeTeamPropPixel, goToWaitingPos, goToBackDrop, placeBackDropPixel, goToParkPos;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new Po(this);

        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        slidesCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        teamElementPipeline = new TeamElementPipeline();
        slidesCamera.setPipeline(teamElementPipeline);
        return robot;
    }

    @Override
    public void onInit() {
        slidesCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                slidesCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the slidesCamera could not be opened
                 /*/
            }
        });

        Po.currentPos = new Pose2d(0, 0, 0);

        robot.arm.closeClaw(true, true);

        robot.arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void autonomous() {
        robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        TeamElementPipeline.pixelLocation location = teamElementPipeline.getLocation();
        TeamElementPipeline.pixelLocation color = teamElementPipeline.getColor();
        TeamElementPipeline.pixelLocation side = teamElementPipeline.getSide();

        placeTeamPropPixel = robot.mecanum.drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(location.getPixelPosition().getX(), location.getPixelPosition().getY() * color.getOffset(), color.getHeading()))
                .addTemporalMarker(0, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.NEUTRAL.getPivotPos());
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.INTAKING.getPivotPos());
                })
                .addTemporalMarker(1, 1, () -> {
                    robot.arm.openClaw(!color.isYellowPixelLeft(), color.isYellowPixelLeft());
                })
                .build();
        goToWaitingPos = robot.mecanum.drive.trajectoryBuilder(placeTeamPropPixel.end())
                .lineToLinearHeading(new Pose2d(location.getBackDropPosition().getX(), location.getBackDropPosition().getY() * color.getOffset(), color.getHeading()))
                .addTemporalMarker(0.5, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.GRABBED_PIXEL.getPivotPos());
                })
                .build();
        goToBackDrop = robot.mecanum.drive.trajectoryBuilder(side == TeamElementPipeline.pixelLocation.AUDIENCE_SIDE ? goToWaitingPos.end() : placeTeamPropPixel.end())
                .lineToLinearHeading(new Pose2d(24, 12 * color.getOffset(), color.getHeading()))
                .addTemporalMarker(0.5, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.GRABBED_PIXEL.getPivotPos());
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.WAITING_TO_DEPOSIT.getPivotPos());
                    robot.arm.setArmPosition(Po.robotState.WAITING_TO_DEPOSIT.getArmPos());
                })
                .build();
        placeBackDropPixel = robot.mecanum.drive.trajectoryBuilder(goToBackDrop.end())
                .lineToLinearHeading(new Pose2d(location.getBackDropPosition().getX(), location.getBackDropPosition().getY() * color.getOffset(), color.getHeading()))
                .addTemporalMarker(1, 1, () -> {
                    robot.arm.openClaw(true, true);
                })
                .addTemporalMarker(1, 3, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.NEUTRAL.getPivotPos());
                    robot.arm.setArmPosition(Po.robotState.NEUTRAL.getArmPos());
                })
                .build();
        goToParkPos = robot.mecanum.drive.trajectoryBuilder(placeBackDropPixel.end())
                .lineToLinearHeading(new Pose2d(side.getParkPosition().getX(), side.getParkPosition().getY() * color.getOffset(), color.getHeading()))
                .build();

        if (opModeIsActive()) {

            telemetry.addData("robot location: ", color.name() + " " + side.name());
            telemetry.addData("pixel location: ", color.name() + " " + location.name());

            telemetry.update();

            robot.mecanum.drive.followTrajectory(placeTeamPropPixel);

            switch (side) {
                case BACKDROP_SIDE:
                    robot.mecanum.drive.followTrajectory(goToBackDrop);
                    robot.mecanum.drive.followTrajectory(placeBackDropPixel);
                    robot.mecanum.drive.followTrajectory(goToParkPos);
                case AUDIENCE_SIDE:
                    robot.mecanum.drive.followTrajectory(goToWaitingPos);
                    robot.mecanum.drive.followTrajectory(goToBackDrop);
                    robot.mecanum.drive.followTrajectory(placeBackDropPixel);
                    robot.mecanum.drive.followTrajectory(goToParkPos);
            }

        }
    }

    @Override
    public void onStop() {
        slidesCamera.stopStreaming();
        slidesCamera.closeCameraDevice();

        Po.currentPos = robot.mecanum.drive.getPoseEstimate();
    }
}