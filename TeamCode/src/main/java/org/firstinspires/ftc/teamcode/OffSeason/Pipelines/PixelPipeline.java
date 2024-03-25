package org.firstinspires.ftc.teamcode.OffSeason.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class PixelPipeline extends OpenCvPipeline {
    private static final int[] greenMin = new int[] {90, 0, 0};
    private static final int[] greenMax = new int[] {120, 255, 255};
    private static final int[] yellowMin = new int[] {0, 0, 0};
    private static final int[] yellowMax = new int[] {60, 255, 255};
    private static final int[] purpleWhiteMin = new int[] {128, 0, 0};
    private static final int[] purpleWhiteMax = new int[] {255, 255, 255};
    private double error;
    Mat yellow = new Mat();
    Mat purpleWhite = new Mat();
    Mat green = new Mat();
    Mat pixel = new Mat();
    @Override
    public Mat processFrame(Mat img) {
        Imgproc.resize(img, img, new Size(80, (int) Math.round((80 / img.size().width) * img.size().height)));

        img.convertTo(img, CvType.CV_32F);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);

        Core.inRange(img, new Scalar(greenMin[0], greenMin[1], greenMin[2]), new Scalar(greenMax[0], greenMax[1], greenMax[2]), green);

        Core.inRange(img, new Scalar(purpleWhiteMin[0], purpleWhiteMin[1], purpleWhiteMin[2]), new Scalar(purpleWhiteMax[0], purpleWhiteMax[1], purpleWhiteMax[2]), purpleWhite);

        Core.inRange(img, new Scalar(yellowMin[0], yellowMin[1], yellowMin[2]), new Scalar(yellowMax[0], yellowMax[1], yellowMax[2]), yellow);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_HSV2BGR);

        Core.bitwise_or(yellow, purpleWhite, yellow);
        Core.bitwise_or(green, yellow, pixel);

        Mat kernel = Mat.ones(5, 5, CvType.CV_32F);
        Imgproc.morphologyEx(pixel, pixel, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(pixel, pixel, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(pixel, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        HashMap<Double, MatOfPoint> distance = new HashMap<>();

        img.convertTo(img, CvType.CV_8UC1);

        for (MatOfPoint contour : contours) {
            if (Imgproc.boundingRect(contour).area() > 250.0) {
                Point centroid = new Point(Imgproc.boundingRect(contour).x + (Imgproc.boundingRect(contour).width / 2.0), Imgproc.boundingRect(contour).y + (Imgproc.boundingRect(contour).height / 2.0));
                double y;

                distance.put(centroid.y, contour);

                y = Collections.max(distance.keySet());
                if (centroid.y == y) {
                    if (centroid.x <= img.width() / 2.0) {
                        error = (img.width() / 5.0) - centroid.x;
                    } else {
                        error = (4.0 * img.width() / 5.0) - centroid.x;
                    }
                    Imgproc.rectangle(img, Imgproc.boundingRect(distance.get(centroid.y)), new Scalar(0, 0, 0), 1);
                    Imgproc.putText(img, "error: " + error, new Point(0, 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.25, new Scalar(255, 255, 255));
                }
            }
        }
        return green;
    }

    public double getError() {
        return error;
    }

}