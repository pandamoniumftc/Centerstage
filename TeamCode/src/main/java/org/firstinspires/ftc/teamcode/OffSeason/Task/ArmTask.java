package org.firstinspires.ftc.teamcode.OffSeason.Task;

import com.roboctopi.cuttlefish.queue.Task;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;
import com.roboctopi.cuttlefishftcbridge.tasks.MotorPositionTask;
import com.roboctopi.cuttlefishftcbridge.tasks.ServoPresetTask;

import java.util.ArrayList;

public class ArmTask implements Task {
    MotorPositionTask task1;
    ServoPresetTask task2;
    boolean t1Complete = false;
    boolean t2Complete = false;
    public ArmTask(int index, CuttleMotor motor, CuttleServo servo, ArrayList<Double> preset) {
        task1 = new MotorPositionTask(preset.get(index), motor, true, 0.05F);
        task2 = new ServoPresetTask(servo, index);
    }

    @Override
    public void kill() {

    }

    @Override
    public boolean loop() {
        if(task1.loop()&&!t1Complete)
        {
            t1Complete = true;
        }
        if(task2.loop()&&!t2Complete)
        {
            t2Complete = true;
        }

        return t1Complete&&t2Complete;
    }

    @Override
    public boolean onBegin() {
        task1.onBegin();
        task2.onBegin();
        return true;
    }
}
