package org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelPipeline extends OpenCvPipeline {
    public static int[] whiteMin = new int[] {160, 0, 0};
    public static int[] whiteMax = new int[] {255, 255, 255};
    public static int[] greenMin = new int[] {0, 0, 0};
    public static int[] greenMax = new int[] {255, 122, 122};
    public static int[] yellowMin = new int[] {0, 100, 130};
    public static int[] yellowMax = new int[] {255, 255, 255};
    public static int[] purpleMin = new int[] {0, 130, 130};
    public static int[] purpleMax = new int[] {255, 255, 255};
    Mat white = new Mat();
    Mat green = new Mat();
    Mat yellow = new Mat();
    Mat purple = new Mat();
    Mat pixel = new Mat();
    public static double strength;
    @Override
    public Mat processFrame(Mat img) {
        Imgproc.resize(img, img, new Size(120, (int) Math.round((120 / img.size().width) * img.size().height)));
        Rect leftROI = new Rect(img.width() * 27 / 256, 0, img.width() * 93 / 256, img.height());
        Rect rightROI = new Rect(img.width() * 17 / 32, 0, img.width() * 93 / 256, img.height());

        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2YUV);

        Core.inRange(img, new Scalar(greenMin[0], greenMin[1], greenMin[2]), new Scalar(greenMax[0], greenMax[1], greenMax[2]), green);

        Core.inRange(img, new Scalar(purpleMin[0], purpleMin[1], purpleMin[2]), new Scalar(purpleMax[0], purpleMax[1], purpleMax[2]), purple);

        Core.inRange(img, new Scalar(whiteMin[0], whiteMin[1], whiteMin[2]), new Scalar(whiteMax[0], whiteMax[1], whiteMax[2]), white);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_YUV2BGR);

        Core.inRange(img, new Scalar(yellowMin[0], yellowMin[1], yellowMin[2]), new Scalar(yellowMax[0], yellowMax[1], yellowMax[2]), yellow);

        Core.bitwise_or(white, green, white);
        Core.bitwise_or(yellow, purple, yellow);
        Core.bitwise_or(white, yellow, pixel);

        Mat kernel = Mat.ones(5, 5, CvType.CV_32F);
        Imgproc.morphologyEx(pixel, pixel, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(pixel, pixel, Imgproc.MORPH_CLOSE, kernel);

        Mat[] roi = new Mat[] {
                pixel.submat(leftROI),
                pixel.submat(rightROI),
        };

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(pixel, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.line(img, new Point(img.width() / 2.0, img.height()), new Point(img.width() / 2.0, img.height() * 3.0 / 4.0), new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(img, leftROI, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(img, rightROI, new Scalar(0, 0, 255), 3);

        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.boundingRect(contours.get(i)).area() > 10000.0) {
                //Rect boundingBox = Imgproc.boundingRect(contours.get(i));
                //Point centroid = new Point(boundingBox.x + (boundingBox.width / 2.0), boundingBox.y + (boundingBox.height / 2.0));
                //Imgproc.putText(img, "area: " + boundingBox.area(), new Point(boundingBox.x, boundingBox.y), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 255, 255));
                //Imgproc.rectangle(img, boundingBox, new Scalar(0, 0, 0), 3);

                double left = 0, right = 0;

                for (int j = 0; j < roi[0].width(); j++) {
                    for (int k = 0; k < roi[0].height(); k++) {
                        if (roi[0].get(j, k) != null) {
                            left += (roi[0].get(j, k)[0] / 255.0);
                        }
                    }
                }

                for (int j = 0; j < roi[1].width(); j++) {
                    for (int k = 0; k < roi[1].height(); k++) {
                        if (roi[1].get(j, k) != null) {
                            right += (roi[1].get(j, k)[0] / 255.0);
                        }
                    }
                }

                // slow down when turning: filtered area / total area = strength, strength * turn input
                strength = 1 - Math.max(left / (leftROI.area() * 0.75), right / (rightROI.area()) * 0.75);

                //Imgproc.putText(img, "left: " + strength, new Point(leftROI.x, img.height()/2.0), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 255, 255));
                //Imgproc.putText(img, "right: " + strength, new Point(rightROI.x, img.height()/2.0), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 255, 255));

                // track pixel when strafing: min distance error = pixel centroid.x - closest ROI centroid.x
            }
        }
        return img;
    }
}