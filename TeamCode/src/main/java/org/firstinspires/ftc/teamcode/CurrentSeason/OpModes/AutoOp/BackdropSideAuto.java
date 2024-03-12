package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous (name="backdrop auto")
public class BackdropSideAuto extends AbstractAutonomous {
    public Po robot;
    public static OpenCvCamera slidesCamera;
    public TeamElementPipeline teamElementPipeline;
    Trajectory placeTeamPropPixel, goToBackDrop, placeBackDropPixel, backFromBackDrop, strafeToParkPos, forwardToParkPos;
    TeamElementPipeline.pixelLocation location, color, side;
    public static int l, c, s;

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
                 */
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

        /*switch (l) {
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
        }*/
        location = teamElementPipeline.getLocation();
        color = teamElementPipeline.getColor();
        side = TeamElementPipeline.pixelLocation.BACKDROP_SIDE;

        placeTeamPropPixel = robot.mecanum.drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(location.getPixelPosition().getX(), getPixelPositionY(location, color, side) * color.getOffset() * side.getOffset(), color.getHeading() * side.getOffset()))
                .addTemporalMarker(0, () -> {
                    robot.arm.pivotServo.setPosition(Po.robotState.INTAKING.getPivotPos());
                })
                .addTemporalMarker(1, 1, () -> {
                    robot.arm.openClaw(!(color.isYellowPixelLeft() == side.isYellowPixelLeft()), (color.isYellowPixelLeft() == side.isYellowPixelLeft()));
                })
                .build();
        goToBackDrop = robot.mecanum.drive.trajectoryBuilder(placeTeamPropPixel.end())
                .lineToLinearHeading(new Pose2d(side.getBackDropPosition().getX(), side.getBackDropPosition().getY() * color.getOffset(), color.getHeading()))
                .build();
        placeBackDropPixel = robot.mecanum.drive.trajectoryBuilder(goToBackDrop.end())
                .lineToLinearHeading(new Pose2d(side.getBackDropPosition().getX() + (location.getOffset() * color.getOffset()), (side.getBackDropPosition().getY() + 7) * color.getOffset(), color.getHeading()))
                .addTemporalMarker(0, 0, () -> {
                    robot.arm.armMotor.setPower(0.05);
                })
                .addTemporalMarker(0.6, 0, () -> {
                    robot.arm.openClaw(true, true);
                })
                .build();
        backFromBackDrop = robot.mecanum.drive.trajectoryBuilder(placeBackDropPixel.end())
                .back(3)
                .build();
        strafeToParkPos = robot.mecanum.drive.trajectoryBuilder(backFromBackDrop.end())
                .lineToConstantHeading(new Vector2d(0, placeBackDropPixel.end().getY()))
                .build();
        forwardToParkPos = robot.mecanum.drive.trajectoryBuilder(strafeToParkPos.end())
                .forward(15)
                .build();

        if (opModeIsActive()) {
            robot.arm.armMotor.setTargetPosition(830);
            robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mecanum.drive.followTrajectory(placeTeamPropPixel);

            robot.arm.openClaw(!(color.isYellowPixelLeft() == side.isYellowPixelLeft()), (color.isYellowPixelLeft() == side.isYellowPixelLeft()));
            robot.arm.pivotServo.setPosition(Po.robotState.DEPOSIT.getPivotPos());
            robot.arm.armMotor.setPower(0.31);

            robot.mecanum.drive.followTrajectory(goToBackDrop);

            robot.mecanum.drive.followTrajectory(placeBackDropPixel);

            robot.arm.armMotor.setPower(0.1);

            robot.arm.armMotor.setTargetPosition(0);

            robot.mecanum.drive.followTrajectory(strafeToParkPos);

            robot.arm.closeClaw(true, true);

            robot.mecanum.drive.followTrajectory(forwardToParkPos);

            robot.arm.pivotServo.setPosition(Po.robotState.INTAKING.getPivotPos());
        }
    }

    private double getPixelPositionY(TeamElementPipeline.pixelLocation location, TeamElementPipeline.pixelLocation color, TeamElementPipeline.pixelLocation side) {
        if (location == TeamElementPipeline.pixelLocation.CENTER_TAPE) {
            return TeamElementPipeline.pixelLocation.CENTER_TAPE.getPixelPosition().getY();
        }
        else if (location == TeamElementPipeline.pixelLocation.BACKDROP_TAPE) {
            return color.getOffset() == side.getOffset() ? TeamElementPipeline.pixelLocation.BACKDROP_TAPE.getPixelPosition().getY() + 2: TeamElementPipeline.pixelLocation.AUDIENCE_TAPE.getPixelPosition().getY() + 2;
        }
        else {
            return color.getOffset() == side.getOffset() ? TeamElementPipeline.pixelLocation.AUDIENCE_TAPE.getPixelPosition().getY() + 2 : TeamElementPipeline.pixelLocation.BACKDROP_TAPE.getPixelPosition().getY() - 2;
        }
    }

    @Override
    public void onStop() {
        /*slidesCamera.stopStreaming();
        slidesCamera.closeCameraDevice();*/

        Po.currentPos = robot.mecanum.drive.getPoseEstimate();
    }
}