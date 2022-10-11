package org.firstinspires.ftc.teamcode;

import android.icu.util.Output;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;



@Autonomous

public class OpenCVExample extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new CapstoneDetector());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
    }

    public void loop (){

    }

    class CapstoneDetector extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 255.0, 0.0);


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


            Rect rightRect = new Rect(1, 1, 426, 719);
            Rect leftRect = new Rect(426, 1, 426, 719);
            Rect midRect = new Rect(853, 1, 426, 719);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect,rectColor, 2);
            Imgproc.rectangle(outPut, midRect,rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop,2);
            Core.extractChannel(rightCrop, rightCrop,2);
            Core.extractChannel(midCrop, midCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            if (leftavgfin < rightavgfin && leftavgfin < midavgfin){
                telemetry.addLine("The Capstone Is At Mid");

            }
            else if (rightavgfin < midavgfin && rightavgfin < leftavgfin){
                telemetry.addLine("The Capstone Is At Left");
            }
            else if (midavgfin < rightavgfin && midavgfin < leftavgfin){
                telemetry.addLine("The Capstone is At Right");

            }
            telemetry.addData("leftavgfin", leftavgfin);
            telemetry.addData("rightavgfin", rightavgfin);
            telemetry.addData("midavgfin", midavgfin);
            return(outPut);

        }
    }

}
