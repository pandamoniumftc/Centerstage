package org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class TeamElementPipeline extends OpenCvPipeline {
    boolean viewportPaused;
    public static int[] redTeamElementMin = new int[] {0, 0, 120};
    public static int[] redTeamElementMax = new int[] {150, 100, 255};
    public static int[] blueTeamElementMin = new int[] {0, 0, 120};
    public static int[] blueTeamElementMax = new int[] {200, 124, 255};
    public enum objectLocation {
        LEFT(-20),
        MIDDLE(0),
        RIGHT(20);

        private final int offset;

        objectLocation(int offset) {this.offset = offset;}

        public int getOffset() { return offset; }

    }
    public boolean isRedSide;
    public static objectLocation location = objectLocation.MIDDLE;

    @Override
    public Mat processFrame(Mat img) {
        Imgproc.resize(img, img, new Size(80, (int) Math.round((640 / img.size().width) * img.size().height)));
        Mat redTeamElement = new Mat();
        Mat blueTeamElement = new Mat();
        Mat object = new Mat();

        Core.inRange(img, new Scalar(redTeamElementMin[0], redTeamElementMin[1], redTeamElementMin[2]), new Scalar(redTeamElementMax[0], redTeamElementMax[1], redTeamElementMax[2]), redTeamElement);
        Core.inRange(img, new Scalar(blueTeamElementMin[0], blueTeamElementMin[1], blueTeamElementMin[2]), new Scalar(blueTeamElementMax[0], blueTeamElementMax[1], blueTeamElementMax[2]), blueTeamElement);

        Core.bitwise_or(redTeamElement, blueTeamElement, object);

        System.out.println(blueTeamElement.size());

        Mat kernel = Mat.ones(3, 3, CvType.CV_32F);
        Imgproc.morphologyEx(object, redTeamElement, Imgproc.MORPH_OPEN, kernel);

        Mat[] roi = new Mat[] {
                object.submat(new Rect(0, img.height() / 3, img.width() / 4, img.height() * 2 / 3)),
                object.submat(new Rect(img.width() / 4, img.height() / 3, img.width() / 2, img.height() / 3)),
                object.submat(new Rect(img.width() * 3 / 4, img.height() / 3, img.width() / 4, img.height() * 2 / 3))
        };

        double left = 0, middle = 0, right = 0;

        for (int i = 0; i < roi[0].width(); i++) {
            for (int j = 0; j < roi[0].height(); j++) {
                if (roi[0].get(i, j) != null) {
                    left += roi[0].get(i, j)[0];
                }
            }
        }

        for (int i = 0; i < roi[1].width(); i++) {
            for (int j = 0; j < roi[1].height(); j++) {
                if (roi[1].get(i, j) != null) {
                    middle += roi[1].get(i, j)[0];
                }
            }
        }

        for (int i = 0; i < roi[2].width(); i++) {
            for (int j = 0; j < roi[2].height(); j++) {
                if (roi[2].get(i, j) != null) {
                    right += roi[2].get(i, j)[0];
                }
            }
        }

        Imgproc.putText(img, location.toString(), new Point(0, 20), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 255, 255), 1);

        Imgproc.rectangle(img, new Rect(0, img.height() / 3, img.width() / 4, img.height() * 2 / 3), new Scalar(0, 0, 0), 3);
        Imgproc.rectangle(img, new Rect(img.width() / 4, img.height() / 3, img.width() / 2, img.height() / 3), new Scalar(0, 0, 0), 3);
        Imgproc.rectangle(img, new Rect(img.width() * 3 / 4, img.height() / 3, img.width() / 4, img.height() * 2 / 3), new Scalar(0, 0, 0), 3);

        if (left > middle && left > right) {
            location = objectLocation.LEFT;
        }

        if (middle > left && middle > right) {
            location = objectLocation.MIDDLE;
        }

        if (right > middle && right > left) {
            location = objectLocation.RIGHT;
        }

        return img;

    }

    @Override
    public void onViewportTapped()
    {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
         */

        /*viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }*/
    }

}
