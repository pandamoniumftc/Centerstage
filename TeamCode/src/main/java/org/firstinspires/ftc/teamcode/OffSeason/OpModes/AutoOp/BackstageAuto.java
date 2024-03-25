package org.firstinspires.ftc.teamcode.OffSeason.OpModes.AutoOp;

import static java.lang.Math.PI;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractQueueAuto;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.OffSeason.Pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.OffSeason.Pipelines.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.OffSeason.Robots.Po;
import org.firstinspires.ftc.teamcode.OffSeason.Task.ClawTask;
import org.firstinspires.ftc.teamcode.OffSeason.Task.SwitchPipelineTask;
import org.openftc.easyopencv.OpenCvCamera;

@Config
@Autonomous (name="backstage auto")
public class BackstageAuto extends AbstractQueueAuto {
    public Po robot;
    public static OpenCvCamera slidesCamera;
    public TeamElementPipeline teamElement;
    public AprilTagDetectionPipeline aprilTag;
    private PTPController pointController;
    public TeamElementPipeline.pixelLocation l, c, s;
    private TaskList autoList = new TaskList();

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new Po(this);

        //int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        //slidesCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        //teamElement = new TeamElementPipeline();
        //slidesCamera.setPipeline(teamElement);

        //teamElement.setInfo(2, TeamElementPipeline.pixelLocation.BACK_STAGE);

        return robot;
    }

    @Override
    public void onInit() {
        /*slidesCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                slidesCamera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the slidesCamera could not be opened
                 *
            }
        });*/

        pointController = new PTPController(robot.drive.driveTrain, robot.deadWheels.localizer);

        Po.currentPos = new Pose(0, 0, 0);



        //robot.arm.controlClaw(false, false);
    }

    @Override
    public void detectionLoop() {
        /*l = teamElement.getInfo()[0];
        c = teamElement.getInfo()[1];
        s = teamElement.getInfo()[2];*/
    }

    @Override
    public void addTasks() {
        // move pivot to intake pos and switch pipeline
        /*autoList.addTask(new ForkTask(
                new ServoPresetTask(
                        robot.arm.pivotServo,
                        robot.arm.INTAKE_LEVEL
                ),
                new SwitchPipelineTask(
                        slidesCamera,
                        aprilTag
                )
        ));
        // moves to tape
        autoList.addTask(new PointTask(
                new Waypoint(
                        new Pose(
                                l.getX() * signum(c.getHeading()),
                                l.getY(),
                                l == TeamElementPipeline.pixelLocation.CENTER_TAPE ? Math.PI : c.getHeading()),
                        1.0,
                        0.05,
                        10,
                        true
                ),
                pointController
        ));
        // drop purple pixel
        autoList.addTask(new ClawTask(
                robot.arm.claw,
                !c.isPurplePixelRight(),
                c.isPurplePixelRight()
        ));
        // moves arm and center itself to the backdrop
        autoList.addTask(new ForkTask(
                new SetMotorPositionTask(
                        Math.PI,
                        robot.arm.armController
                ),
                new PointTask(
                        new Waypoint(
                                new Pose(
                                        s.getX() * signum(c.getHeading()),
                                        s.getY(),
                                        c.getHeading()),
                                1.0,
                                0.05,
                                10,
                                true
                        ),
                        pointController
                )
        ));
        // drop yellow pixel
        autoList.addTask(new ClawTask(
                robot.arm.claw,
                c.isPurplePixelRight(),
                !c.isPurplePixelRight()
        ));
        // waits 2 seconds
        autoList.addTask(new DelayTask(2000));
        robot.arm.armController.setPosition(Math.PI);
        autoList.addTask(new SetMotorPositionTask(
                Math.PI,
                robot.arm.armController
        ));
        // parks
        autoList.addTask(new PointTask(
                new Waypoint(
                        new Pose(
                                s.getX() * signum(c.getHeading()),
                                s.getY(),
                                c.getHeading()),
                        1.0,
                        0.05,
                        10,
                        false
                ),
                pointController
        ));*/
        autoList.addTask(new PointTask(
                new Waypoint(
                        new Pose(0, 609.6, PI/2.0),
                        0.6,
                        0.05,
                        10,
                        false
                ),
                pointController
        ));
        robot.queue.addTask(autoList);
    }

    @Override
    public void queueLoop() {
        robot.ctrlHub.pullBulkData();
        robot.expHub.pullBulkData();
        robot.deadWheels.localizer.update();
        robot.queue.update();

        robot.telemetry.addData("CURRENT POS: ", robot.deadWheels.localizer.getPos().getX() + " " + robot.deadWheels.localizer.getPos().getY() + " " + robot.deadWheels.localizer.getPos().getR());
        robot.telemetry.update();
    }

    @Override
    public void onStop() {
        //slidesCamera.stopStreaming();
        //slidesCamera.closeCameraDevice();

        Po.currentPos = robot.deadWheels.localizer.getPos();
    }
}