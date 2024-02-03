package org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class PixelPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat img) {
        return img;
    }
}
