package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TinzenBanaraDMPipeline extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public String position = "plCEHOLD3R";
   public TinzenBanaraDMPipeline() {

       // the green M&M has blessed this code, also this in Tinzen and not Noah

   }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(0, 240, 0, 426);
        Mat matMid = workingMatrix.submat(240, 480, 426, 852);
        Mat matRight = workingMatrix.submat(480, 720, 852, 1280);

        Imgproc.rectangle(workingMatrix, new Rect(0, 0, 426, 240), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(240, 426, 426, 240), new Scalar(255, 0, 0));
        Imgproc.rectangle(workingMatrix, new Rect(480, 852, 426, 240), new Scalar(0, 0, 255));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double midTotal = Core.sumElems(matMid).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if (leftTotal > midTotal){
            if (leftTotal > rightTotal) {
                //left is darkest
                position = "LEFT";

            } else {
                //right is darkest
                position = "RIGHT";
            }
    }else {
            if (midTotal > rightTotal){
                // mid is darkest
                position = "MID";
            }else {
                // right is darkest
                position = "RIGHT";
            }
        }
        return workingMatrix;
    }
}
