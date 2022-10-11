package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


public class TinzenHotTubCam extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location{
        LEFT,
        RIGHT,
        MID
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(426, 720));
    static final Rect MID_ROI = new Rect(
            new Point(426, 0),
            new Point(852, 720));
    static final Rect RIGHT_ROI = new Rect(
            new Point(852, 0),
            new Point(1280, 720));
    // returns function
    public TinzenHotTubCam(Telemetry t) { telemetry = t; }

    static double PERCENTAGE_COLOR_THRESHOLD = 0.4;



    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat mid = mat.submat(MID_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();
        mid.release();

        telemetry.addData( "Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData( "mid raw value", (int) Core.sumElems(mid).val[0]);
        telemetry.addData( "Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData( "Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData( "Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");

        boolean capleft = leftValue > PERCENTAGE_COLOR_THRESHOLD;
        boolean capright = rightValue > PERCENTAGE_COLOR_THRESHOLD;
        boolean capmid = midValue > PERCENTAGE_COLOR_THRESHOLD;

        if (capright){
            // left
            location = location.RIGHT;
            telemetry.addData("Cap Location ", "right");
        }
        else if (capmid){
            // left
            location = location.MID;
            telemetry.addData("Cap Location ", "mid");
        }
        else {
            //right
            location = location.LEFT;
            telemetry.addData("Cap Location ", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorCapStone = new Scalar(23, 50, 70);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorCapStone:colorCapStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorCapStone:colorCapStone);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MID? colorCapStone:colorCapStone);

        return mat;
    }

    public Location getLocation(){

        return location;


    }

}
