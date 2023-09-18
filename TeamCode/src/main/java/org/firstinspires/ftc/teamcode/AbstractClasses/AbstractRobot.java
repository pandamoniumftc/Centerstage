package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;

public abstract class AbstractRobot {

    public final OpMode opMode;
    public final Telemetry telemetry;
    public final HardwareMap hardwareMap;

    public ArrayList<AbstractSubsystem> subsystems;

    //public RobotTeleOpConfig robotTeleOpConfig;
    //public RobotAutoConfig robotAutoConfig;

    public Gamepad gamepad1, gamepad2;

    //public BHI260IMU imu;

    public AbstractRobot(OpMode opMode) {
        this.opMode =  opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        subsystems = new ArrayList<>();

    }

    public void init() throws IllegalAccessException, IOException {
        Field[] fields = this.getClass().getDeclaredFields();

        //ArrayList<SubsystemTeleOpConfig> subsystemTeleOpConfigs = new ArrayList<>();
        //ArrayList<SubsystemAutoConfig> subsystemAutoConfigs = new ArrayList<>();

        for (Field f : fields) {
            if (AbstractSubsystem.class.isAssignableFrom(f.getType())) {
                Object obj;
                try {
                    obj = f.get(this);
                }
                catch (IllegalAccessException e) {
                    throw new IllegalAccessException("make subsystems public");
                }

                if (obj != null) {
                    subsystems.add((AbstractSubsystem)obj);
                    //TODO: load
                    /* if (((AbstractSubsystem)obj).usesConfig) {
                        ((AbstractSubsystem)obj).buildConfigs();
                        subsystemTeleOpConfigs.add(((AbstractSubsystem)obj).subsystemTeleOpConfigs);
                        subsystemAutoConfigs.add(((AbstractSubsystem)obj).subsystemAutoConfigs);
                    }*/
                }
            }
        }

        //robotTeleOpConfig = new RobotTeleOpConfig((SubsystemTeleOpConfig[])subsystemTeleOpConfigs.toArray());
        //robotAutoConfig = new RobotAutoConfig((SubsystemAutoConfig[])subsystemAutoConfigs.toArray());

        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        /*BHI260IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/
        for (AbstractSubsystem system : subsystems) {
            system.init();
        }
    }

    public final void start() {
        for (AbstractSubsystem system : subsystems) {
            system.start();
            if (system.usesConfig) {
                //TODO: save
            }
        }


    }

    public final void driverLoop() {
        for (AbstractSubsystem system : subsystems) {
            telemetry.addData("", "-------" + system.getClass().getSimpleName() + "------- :");
            system.driverLoop();
        }
        telemetry.update();
    }

    public final void stop() {
        for (AbstractSubsystem system : subsystems) {
            system.stop();
        }
    }



}
