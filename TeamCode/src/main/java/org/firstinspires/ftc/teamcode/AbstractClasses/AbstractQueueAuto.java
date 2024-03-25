package org.firstinspires.ftc.teamcode.AbstractClasses;

import java.io.IOException;

public abstract class AbstractQueueAuto extends AbstractOpMode {
    public AbstractRobot robot;
    public abstract void detectionLoop();
    public abstract void addTasks();
    public abstract void queueLoop();
    @Override
    public final void runOpMode() {

        super.runOpMode();
        robot = getRobot();

        try {

            robot.init();
            onInit();

            while (!isStarted() && !isStopRequested()) {
                detectionLoop();
            }

            addTasks();

            while(!isStopRequested() && !robot.queue.getIdle()) {
                queueLoop();
            }

            robot.stop();
            super.onStop();
            super.stop();

        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
