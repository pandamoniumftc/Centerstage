package org.firstinspires.ftc.teamcode.OffSeason.Task;

import com.roboctopi.cuttlefish.queue.Task;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class ClawTask implements Task {
    boolean left, right;
    CuttleServo[] claw;
    public ClawTask(CuttleServo[] claw, boolean left, boolean right) {
        this.left = left;
        this.right = right;
        this.claw = claw;
    }

    /**
     *
     */
    @Override
    public void kill() {

    }

    /**
     * @return
     */
    @Override
    public boolean loop() {
        claw[0].goToPreset(left ? 1 : 0);
        claw[1].goToPreset(right ? 1 : 0);
        return true;
    }

    /**
     * @return
     */
    @Override
    public boolean onBegin() {
        return true;
    }
}
