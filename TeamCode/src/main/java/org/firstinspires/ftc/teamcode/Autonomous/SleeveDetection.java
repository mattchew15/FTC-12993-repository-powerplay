package org.firstinspires.ftc.teamcode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    /*
    BLUE  = Parking Center
    YELLOW    = Parking Left
    RED = Parking Right
     */

    public enum ParkingPosition {
        CENTER,
        LEFT,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_blue_bounds    = new Scalar(0, 0, 255, 255),
            upper_blue_bounds    = new Scalar(150, 255, 255, 255),
            lower_yellow_bounds  = new Scalar(100, 100, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 150, 255),
            lower_red_bounds     = new Scalar(100, 0, 0, 255),
            upper_red_bounds     = new Scalar(255, 100, 110, 255);

    // Color definitions
    private final Scalar
            BLUE    = new Scalar(0, 0, 255),
            YELLOW  = new Scalar(255, 255, 0),
            RED     = new Scalar(255, 0, 0);

    // Percent and mat definitions
    public double bluPercent, yelPercent, redPercent;
    private Mat bluMat = new Mat(), yelMat = new Mat(), redMat = new Mat(), blurredMat = new Mat();////////

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.CENTER;/////////

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_blue_bounds, upper_blue_bounds, bluMat);
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);

        // Gets color specific values
        bluPercent = Core.countNonZero(bluMat);
        yelPercent = Core.countNonZero(yelMat);
        redPercent = Core.countNonZero(redMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(yelPercent, Math.max(bluPercent, redPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (yelPercent > 1000) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == redPercent && redPercent > 500) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    2
            );
        } else {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BLUE,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        yelMat.release();
        bluMat.release();
        redMat.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}
