package org.firstinspires.ftc.teamcode.OffSeason.Task;

import com.roboctopi.cuttlefish.queue.Task;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class SwitchPipelineTask implements Task {
    boolean complete = false;
    OpenCvPipeline pipeline;
    OpenCvCamera camera;
    public SwitchPipelineTask(OpenCvPipeline pipe, OpenCvCamera cam) {
        pipeline = pipe;
        camera = cam;
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
        camera.setPipeline(pipeline);
        if (camera.getPipelineTimeMs() > 1000 && camera.getFps() > 0) {
            complete = true;
        }
        return complete;
    }

    /**
     * @return
     */
    @Override
    public boolean onBegin() {
        return true;
    }
}
