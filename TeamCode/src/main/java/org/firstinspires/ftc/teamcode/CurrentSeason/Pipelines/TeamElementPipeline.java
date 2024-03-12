package org.firstinspires.ftc.teamcode.CurrentSeason.Pipelines;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.AutoOp.BackdropSideAuto;
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

public class TeamElementPipeline extends OpenCvPipeline {
    Po robot;
    boolean viewportPaused;
    private static final int[] redTeamElementMin = new int[] {0, 0, 160};
    private static final int[] redTeamElementMax = new int[] {255, 128, 255};
    private static final int[] blueTeamElementMin = new int[] {0, 160, 0};
    private static final int[] blueTeamElementMax = new int[] {255, 255, 128};
    public enum pixelLocation {
        CENTER_TAPE(new Vector2d(36, 1), 0),
        BACKDROP_TAPE(new Vector2d(30, 20), -6),
        AUDIENCE_TAPE(new Vector2d(30, -3), 6),
        BACKDROP_SIDE(new Vector2d(27, 24),true, 1),
        AUDIENCE_SIDE(new Vector2d(24, 72), false, -1),
        RED(false, -Math.PI/2, -1),
        BLUE(true, Math.PI/2, 1);

        private Vector2d pixelPosition, backDropPosition;
        private double heading;
        private final int offset;
        private boolean isYellowPixelLeft;

        pixelLocation(Vector2d pixelPos, int offset) {
            this.pixelPosition = pixelPos;
            this.offset = offset;
        }
        pixelLocation(boolean yellowPixel, double heading, int offset) {
            this.isYellowPixelLeft = yellowPixel;
            this.heading = heading;
            this.offset = offset;
        }
        pixelLocation(Vector2d backDropPos, boolean yellowPixel, int offset) {
            this.backDropPosition = backDropPos;
            this.isYellowPixelLeft = yellowPixel;
            this.offset = offset;
        }

        public Vector2d getPixelPosition() {
            return pixelPosition;
        }
        public Vector2d getBackDropPosition() {
            return backDropPosition;
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
    Rect leftROI, middleROI, rightROI;
    private pixelLocation location, color;
    @Override
    public Mat processFrame(Mat img) {
        Imgproc.resize(img, img, new Size(80, (int) Math.round((80 / img.size().width) * img.size().height)));

        leftROI = new Rect(0, img.height() / 3, img.width() / 4, img.height() * 2 / 3);
        middleROI = new Rect(img.width() / 4, img.height() / 3, img.width() / 2, img.height() / 3);
        rightROI = new Rect(img.width() * 3 / 4, img.height() / 3, img.width() / 4, img.height() * 2 / 3);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2YUV);

        Core.inRange(img, new Scalar(redTeamElementMin[0], redTeamElementMin[1], redTeamElementMin[2]), new Scalar(redTeamElementMax[0], redTeamElementMax[1], redTeamElementMax[2]), redTeamElement);
        Core.inRange(img, new Scalar(blueTeamElementMin[0], blueTeamElementMin[1], blueTeamElementMin[2]), new Scalar(blueTeamElementMax[0], blueTeamElementMax[1], blueTeamElementMax[2]), blueTeamElement);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_YUV2BGR);

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
        };

        double red = 0, blue = 0, left = 0, middle = 0, right = 0;

        for (int i = 0; i < redTeamElement.width(); i++) {
            for (int j = 0; j < redTeamElement.height(); j++) {
                if (redTeamElement.get(i, j) != null) {
                    red += redTeamElement.get(i, j)[0];
                }
            }
        }
        for (int i = 0; i < blueTeamElement.width(); i++) {
            for (int j = 0; j < blueTeamElement.height(); j++) {
                if (blueTeamElement.get(i, j) != null) {
                    blue += blueTeamElement.get(i, j)[0];
                }
            }
        }

        if (red > blue) {color = pixelLocation.RED;}
        else {color = pixelLocation.BLUE;}

        switch (color) {
            case RED:
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
            case BLUE:
                for (int i = 0; i < roi[3].width(); i++) {
                    for (int j = 0; j < roi[3].height(); j++) {
                        if (roi[3].get(i, j) != null) {
                            left += roi[3].get(i, j)[0];
                        }
                    }
                }
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
        }

        Imgproc.rectangle(img, leftROI, new Scalar(0, 0, 0), 1);
        Imgproc.rectangle(img, middleROI, new Scalar(0, 0, 0), 1);
        Imgproc.rectangle(img, rightROI, new Scalar(0, 0, 0), 1);

        if (left > middle && left > right) {
            switch (color) {
                case BLUE:
                    location = pixelLocation.BACKDROP_SIDE;
                    break;
                case RED:
                    location = pixelLocation.AUDIENCE_SIDE;
                    break;
            }

        }
        else if (middle > left && middle > right) {
            location = pixelLocation.CENTER_TAPE;
        }
        else if (right > middle && right > left) {
            switch (color) {
                case BLUE:
                    location = pixelLocation.AUDIENCE_TAPE;
                    break;
                case RED:
                    location = pixelLocation.BACKDROP_TAPE;
                    break;
            }
        }

        if (location != null) {
            Imgproc.putText(img, location.name(), new Point(0, 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.3, new Scalar(255, 255, 255), 1);
        }
        if (color != null) {
            Imgproc.putText(img, color.name(), new Point(0, 50), Imgproc.FONT_HERSHEY_COMPLEX, 0.3, new Scalar(255, 255, 255), 1);
        }

        return img;
    }

    public pixelLocation getLocation() {
        return location;
    }
    public pixelLocation getColor() {
        return color;
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
            BackdropSideAuto.slidesCamera.pauseViewport();
        }
        else
        {
            BackdropSideAuto.slidesCamera.resumeViewport();
        }
    }
}