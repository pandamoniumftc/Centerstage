package org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class TeamElementPipeline extends OpenCvPipeline {

    int[] pixelIndicatorMin = new int[] {0, 0, 0};
    int[] pixelIndicatorMax = new int[] {255, 255, 255};
    int[] redTeamElementMin = new int[] {0, 0, 0};
    int[] redTeamElementMax = new int[] {255, 255, 255};
    int[] blueTeamElementMin = new int[] {0, 0, 0};
    int[] blueTeamElementMax = new int[] {255, 255, 255};

    public enum objectLocation {
        LEFT(-20),
        MIDDLE(0),
        RIGHT(20);

        private final int offset;

        objectLocation(int offset) {this.offset = offset;}

        public int getOffset() { return offset; }

    }

    public static objectLocation location = objectLocation.MIDDLE;

    @Override
    public Mat processFrame(Mat img) {
        //Imgproc.resize(img, img, );

        Mat kernel = Mat.ones(5, 5, CvType.CV_32F);
        Mat pixelIndicator = new Mat();
        Mat redTeamElement = new Mat();
        Mat blueTeamElement = new Mat();
        Mat object = new Mat();

        Core.inRange(img, new Scalar(pixelIndicatorMin[0], pixelIndicatorMin[1], pixelIndicatorMin[2]), new Scalar(pixelIndicatorMax[0], pixelIndicatorMax[1], pixelIndicatorMax[2]), pixelIndicator);

        Imgproc.cvtColor(object, object, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(img, new Scalar(redTeamElementMin[0], redTeamElementMin[1], redTeamElementMin[2]), new Scalar(redTeamElementMax[0], redTeamElementMax[1], redTeamElementMax[2]), redTeamElement);
        Core.inRange(img, new Scalar(blueTeamElementMin[0], blueTeamElementMin[1], blueTeamElementMin[2]), new Scalar(blueTeamElementMax[0], blueTeamElementMax[1], blueTeamElementMax[2]), blueTeamElement);

        Imgproc.morphologyEx(pixelIndicator, pixelIndicator, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(pixelIndicator, pixelIndicator, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(pixelIndicator, pixelIndicator, Imgproc.MORPH_CLOSE, kernel);

        Core.bitwise_or(pixelIndicator, redTeamElement, object);
        Core.bitwise_or(object, blueTeamElement, object);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        MatOfPoint2f approx = new MatOfPoint2f();

        Imgproc.cvtColor(object, object, Imgproc.COLOR_YCrCb2BGR);
        Imgproc.cvtColor(object, object, Imgproc.COLOR_RGB2GRAY);

        Imgproc.findContours(object, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        final Point imgCenter = new Point(Math.round(img.width() / 2.0), Math.round(img.height() / 2.0));
        Point centroid = new Point();

        for (int i = 0; i < contours.size(); i++) {

            double peri = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);

            double contoursArea = Imgproc.contourArea(contours.get(i));

            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approx, 0.01 * peri, true);

            if (contoursArea > 500 && contoursArea < 50000) {

                final Moments moments = Imgproc.moments(approx);
                centroid.x = Math.round(moments.get_m10() / moments.get_m00());
                centroid.y = Math.round(moments.get_m01() / moments.get_m00());

                if (centroid.x > imgCenter.x) {location = objectLocation.LEFT;}
                if (centroid.x < imgCenter.x) {location = objectLocation.RIGHT;}
                else {location = objectLocation.MIDDLE;}
            }

        }

        return img;
    }
}
