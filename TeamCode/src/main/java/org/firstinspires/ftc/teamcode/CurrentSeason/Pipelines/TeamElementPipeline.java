package org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp.DetectionAuto;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.Po;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Vector;

public class TeamElementPipeline extends OpenCvPipeline {
    Po robot;
    boolean viewportPaused;
    private static final int[] redTeamElementMin = new int[] {0, 150, 0};
    private static final int[] redTeamElementMax = new int[] {80, 255, 128};
    private static final int[] blueTeamElementMin = new int[] {0, 0, 128};
    private static final int[] blueTeamElementMax = new int[] {200, 128, 255};
    private static final int[] yellowMin = new int[] {25, 120, 120};
    private static final int[] yellowMax = new int[] {50, 255, 255};
    public enum pixelLocation {
        CENTER_TAPE(new Vector2d(36, 0), new Vector2d(36, 0)),
        BACKDROP_TAPE(new Vector2d(24, 12), new Vector2d(36, 0)),
        AUDIENCE_TAPE(new Vector2d(24, -12), new Vector2d(36, 0)),
        BACKDROP_SIDE(new Vector2d(0, 0),true, 1),
        AUDIENCE_SIDE(new Vector2d(0, 0), false, -1),
        RED(false, -Math.PI/2, -1),
        BLUE(true, Math.PI/2, 1);

        private Vector2d pixelPosition, backDropPosition, parkPosition;
        private double heading;
        private int offset;
        private boolean isYellowPixelLeft;

        pixelLocation(Vector2d pixelPos, Vector2d backDropPos) {
            this.pixelPosition = pixelPos;
            this.backDropPosition = backDropPos;
        }
        pixelLocation(boolean yellowPixel, double heading, int offset) {
            this.isYellowPixelLeft = yellowPixel;
            this.heading = heading;
            this.offset = offset;
        }
        pixelLocation(Vector2d parkPos, boolean yellowPixel, int offset) {
            this.parkPosition = parkPos;
            this.isYellowPixelLeft = yellowPixel;
            this.offset = offset;
        }

        public Vector2d getPixelPosition() {
            return pixelPosition;
        }
        public Vector2d getBackDropPosition() {
            return backDropPosition;
        }
        public Vector2d getParkPosition() {
            return parkPosition;
        }
        public int getOffset() {
            return offset;
        }
        public double getHeading() {
            return heading;
        }
        public boolean isYellowPixelLeft() {
            return isYellowPixelLeft;
        }

    }
    Mat redTeamElement = new Mat();
    Mat blueTeamElement = new Mat();
    Mat yellow = new Mat();
    Rect leftROI, middleROI, rightROI, topLeftROI, topRightROI;
    private pixelLocation location, color, side;
    @Override
    public Mat processFrame(Mat img) {
        Imgproc.resize(img, img, new Size(120, (int) Math.round((120 / img.size().width) * img.size().height)));

        leftROI = new Rect(0, img.height() / 3, img.width() / 4, img.height() * 2 / 3);
        middleROI = new Rect(img.width() / 4, img.height() / 3, img.width() / 2, img.height() / 3);
        rightROI = new Rect(img.width() * 3 / 4, img.height() / 3, img.width() / 4, img.height() * 2 / 3);
        topLeftROI = new Rect(0, 0, img.width() / 3, img.height() / 3);
        topRightROI = new Rect(img.width() * 2 / 3, 0, img.width() / 3, img.height() / 3);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2YUV);

        Core.inRange(img, new Scalar(redTeamElementMin[0], redTeamElementMin[1], redTeamElementMin[2]), new Scalar(redTeamElementMax[0], redTeamElementMax[1], redTeamElementMax[2]), redTeamElement);
        Core.inRange(img, new Scalar(blueTeamElementMin[0], blueTeamElementMin[1], blueTeamElementMin[2]), new Scalar(blueTeamElementMax[0], blueTeamElementMax[1], blueTeamElementMax[2]), blueTeamElement);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_YUV2BGR);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);

        Core.inRange(img, new Scalar(yellowMin[0], yellowMin[1], yellowMin[2]), new Scalar(yellowMax[0], yellowMax[1], yellowMax[2]), yellow);

        Mat kernel = Mat.ones(3, 3, CvType.CV_32F);
        Imgproc.morphologyEx(redTeamElement, redTeamElement, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(blueTeamElement, blueTeamElement, Imgproc.MORPH_OPEN, kernel);

        Mat[] roi = new Mat[] {
                redTeamElement.submat(leftROI),
                redTeamElement.submat(middleROI),
                redTeamElement.submat(rightROI),
                blueTeamElement.submat(leftROI),
                blueTeamElement.submat(middleROI),
                blueTeamElement.submat(rightROI),
                yellow.submat(topLeftROI),
                yellow.submat(topRightROI)
        };

        double leftBlue = 0, leftRed = 0, middle = 0, right = 0, yellow = 0;

        for (int i = 0; i < roi[0].width(); i++) {
            for (int j = 0; j < roi[0].height(); j++) {
                if (roi[0].get(i, j) != null) {
                    leftRed += roi[0].get(i, j)[0];
                }
            }
        }
        for (int i = 0; i < roi[3].width(); i++) {
            for (int j = 0; j < roi[3].height(); j++) {
                if (roi[3].get(i, j) != null) {
                    leftBlue += roi[3].get(i, j)[0];
                }
            }
        }

        if (leftRed > leftBlue) {color = pixelLocation.RED;}
        else {color = pixelLocation.BLUE;}

        switch (color) {
            case RED:
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
                for (int i = 0; i < roi[7].width(); i++) {
                    for (int j = 0; j < roi[7].height(); j++) {
                        if (roi[7].get(i, j) != null) {
                            yellow += roi[3].get(i, j)[0];
                        }
                    }
                }
                if (yellow > 0) {side = pixelLocation.AUDIENCE_SIDE;}
                else {side = pixelLocation.BACKDROP_SIDE;}
            case BLUE:
                for (int i = 0; i < roi[4].width(); i++) {
                    for (int j = 0; j < roi[4].height(); j++) {
                        if (roi[4].get(i, j) != null) {
                            middle += roi[4].get(i, j)[0];
                        }
                    }
                }
                for (int i = 0; i < roi[5].width(); i++) {
                    for (int j = 0; j < roi[5].height(); j++) {
                        if (roi[5].get(i, j) != null) {
                            right += roi[5].get(i, j)[0];
                        }
                    }
                }
                for (int i = 0; i < roi[6].width(); i++) {
                    for (int j = 0; j < roi[6].height(); j++) {
                        if (roi[7].get(i, j) != null) {
                            yellow += roi[3].get(i, j)[0];
                        }
                    }
                }
                if (yellow > 0) {side = pixelLocation.AUDIENCE_SIDE;}
                else {side = pixelLocation.BACKDROP_SIDE;}
        }

        Imgproc.rectangle(img, leftROI, new Scalar(0, 0, 0), 1);
        Imgproc.rectangle(img, middleROI, new Scalar(0, 0, 0), 1);
        Imgproc.rectangle(img, rightROI, new Scalar(0, 0, 0), 1);
        Imgproc.rectangle(img, topLeftROI, new Scalar(0, 0, 0), 1);
        Imgproc.rectangle(img, topRightROI, new Scalar(0, 0, 0), 1);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_HSV2BGR);

        if (Math.max(leftRed, leftBlue) > middle && Math.max(leftRed, leftBlue) > right) {
            switch (color) {
                case BLUE:
                    location = pixelLocation.BACKDROP_SIDE;
                case RED:
                    location = pixelLocation.AUDIENCE_SIDE;
            }

        }
        else if (middle > Math.max(leftRed, leftBlue) && middle > right) {
            location = pixelLocation.CENTER_TAPE;
        }
        else if (right > middle && right > Math.max(leftRed, leftBlue)) {
            switch (color) {
                case BLUE:
                    location = pixelLocation.AUDIENCE_TAPE;
                case RED:
                    location = pixelLocation.BACKDROP_TAPE;
            }
        }

        Imgproc.putText(img, location.toString(), new Point(0, 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 255, 255), 1);
        Imgproc.putText(img, color.toString(), new Point(0, 50), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 255, 255), 1);
        Imgproc.putText(img, side.toString(), new Point(0, 80), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 255, 255), 1);

        return img;
    }

    public pixelLocation getLocation() {
        return location;
    }
    public pixelLocation getColor() {
        return color;
    }
    public pixelLocation getSide() {
        return side;
    }
    @Override
    public void onViewportTapped() {
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

        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            DetectionAuto.slidesCamera.pauseViewport();
        }
        else
        {
            DetectionAuto.slidesCamera.resumeViewport();
        }
    }
}