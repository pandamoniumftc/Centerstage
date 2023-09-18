package org.firstinspires.ftc.teamcode.AbstractClasses;

import java.io.IOException;

public abstract class AbstractTeleOp extends AbstractOpMode {

    public abstract AbstractRobot instantiateRobot();

    public void onStart() {}

    public void onDriverUpdate(){}

    @Override
    public final void runOpMode() {
        super.runOpMode();

        AbstractRobot robot = getRobot();

        try {
            robot.init();
            onInit();

            while (!isStarted() && !isStopRequested()) {}

            robot.start();
            onStart();

            while (!isStopRequested()) {
                robot.driverLoop();
                onDriverUpdate();
            }

            onStop();
            robot.stop();
        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
