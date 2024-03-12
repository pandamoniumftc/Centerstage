package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.openftc.easyopencv.OpenCvCamera;

@Config
@Autonomous (name="audience auto")
public class AudienceSideAuto extends AbstractAutonomous {
    public Po robot;
    public static OpenCvCamera slidesCamera;
    public TeamElementPipeline teamElementPipeline;
    Trajectory forwardTeamPropPixel, placeTeamPropPixel, centerToWaitingPos, goToBackDrop, placeBackDropPixel, goToParkPos;
    TeamElementPipeline.pixelLocation location, color, side;
    public static int l, c, s;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new Po(this);

        /*int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        slidesCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        teamElementPipeline = new TeamElementPipeline();
        slidesCamera.setPipeline(teamElementPipeline);*/
        return robot;
    }

    @Override
    public void onInit() {
        /*slidesCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                slidesCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the slidesCamera could not be opened
                 /
            }
        });*/

        Po.currentPos = new Pose2d(0, 0, 0);

        robot.arm.closeClaw(true, true);

        robot.arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void autonomous() {
        robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        switch (l) {
            case 0:
                location = TeamElementPipeline.pixelLocation.BACKDROP_TAPE;
                break;
            case 1:
                location = TeamElementPipeline.pixelLocation.CENTER_TAPE;
                break;
            case 2:
                location = TeamElementPipeline.pixelLocation.AUDIENCE_TAPE;
                break;
        }
        switch (c) {
            case 0:
                color = TeamElementPipeline.pixelLocation.BLUE;
                break;
            case 1:
                color = TeamElementPipeline.pixelLocation.RED;
                break;
        }

        side = TeamElementPipeline.pixelLocation.AUDIENCE_SIDE;

        forwardTeamPropPixel = robot.mecanum.drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(location.getPixelPosition().getX())
                .addTemporalMarker(0, 0, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.INTAKING.getPivotPos());
                })
                .build();
        placeTeamPropPixel = robot.mecanum.drive.trajectoryBuilder(forwardTeamPropPixel.end())
                .lineToLinearHeading(new Pose2d(0, location.getPixelPosition().getY(), color.getHeading()))
                .addTemporalMarker(0, 0, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.INTAKING.getPivotPos());
                })
                .addTemporalMarker(1, 1, () -> {
                    robot.arm.openClaw(!(color.isYellowPixelLeft() == side.isYellowPixelLeft()), (color.isYellowPixelLeft() == side.isYellowPixelLeft()));
                })
                .build();
        centerToWaitingPos = robot.mecanum.drive.trajectoryBuilder(placeBackDropPixel.end())
                .lineToLinearHeading(new Pose2d(26, 24 * color.getOffset(), color.getHeading()))
                .addTemporalMarker(0, () -> {
                    robot.arm.openClaw(!(color.isYellowPixelLeft() == side.isYellowPixelLeft()), (color.isYellowPixelLeft() == side.isYellowPixelLeft()));
                })
                .build();
        goToBackDrop = robot.mecanum.drive.trajectoryBuilder(centerToWaitingPos.end())
                .lineToLinearHeading(new Pose2d(side.getBackDropPosition().getX(), side.getBackDropPosition().getY() * color.getOffset(), color.getHeading()))
                .addTemporalMarker(0, () -> {
                    robot.arm.openClaw(!(color.isYellowPixelLeft() == side.isYellowPixelLeft()), (color.isYellowPixelLeft() == side.isYellowPixelLeft()));
                })
                .build();
        placeBackDropPixel = robot.mecanum.drive.trajectoryBuilder(goToBackDrop.end())
                .lineToLinearHeading(new Pose2d(location.getBackDropPosition().getX() + (location.getOffset() * color.getOffset()), (location.getBackDropPosition().getY() + 10) * color.getOffset(), color.getHeading()))
                .addTemporalMarker(0, 0, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.DEPOSIT.getPivotPos());
                    robot.arm.setArmPosition(2, Po.robotState.DEPOSIT.getLifted());
                    robot.slides.setSlidesPosition(200);
                })
                .addTemporalMarker(1, 1, () -> {
                    robot.arm.openClaw(true, true);
                })
                .addTemporalMarker(1, 3, () -> {
                    robot.arm.closeClaw(true, true);
                    robot.arm.pivotServo.setPosition(Po.robotState.NEUTRAL.getPivotPos());
                    robot.arm.setArmPosition(2, Po.robotState.NEUTRAL.getLifted());
                })
                .build();
        goToParkPos = robot.mecanum.drive.trajectoryBuilder(placeBackDropPixel.end())
                .lineToLinearHeading(new Pose2d(0, 0, color.getHeading()))
                .build();

        if (opModeIsActive()) {

            robot.mecanum.drive.followTrajectory(placeTeamPropPixel);
            robot.mecanum.drive.followTrajectory(goToBackDrop);
            robot.mecanum.drive.followTrajectory(placeBackDropPixel);
            robot.mecanum.drive.followTrajectory(goToParkPos);

        }
    }

    private double getPixelPositionY(TeamElementPipeline.pixelLocation location, TeamElementPipeline.pixelLocation color, TeamElementPipeline.pixelLocation side) {
        if (location == TeamElementPipeline.pixelLocation.CENTER_TAPE) {
            return TeamElementPipeline.pixelLocation.CENTER_TAPE.getPixelPosition().getY();
        }
        else if (location == TeamElementPipeline.pixelLocation.BACKDROP_TAPE) {
            return color.getOffset() == side.getOffset() ? TeamElementPipeline.pixelLocation.BACKDROP_TAPE.getPixelPosition().getY() : TeamElementPipeline.pixelLocation.AUDIENCE_TAPE.getPixelPosition().getY() + 2;
        }
        else {
            return color.getOffset() == side.getOffset() ? TeamElementPipeline.pixelLocation.AUDIENCE_TAPE.getPixelPosition().getY() : TeamElementPipeline.pixelLocation.BACKDROP_TAPE.getPixelPosition().getY() - 2;
        }
    }

    @Override
    public void onStop() {
        /*slidesCamera.stopStreaming();
        slidesCamera.closeCameraDevice();*/

        Po.currentPos = robot.mecanum.drive.getPoseEstimate();
    }
}