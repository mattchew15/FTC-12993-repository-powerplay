package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//---------------------------------------------------------------------------------------------------------------------------

@Autonomous (name = "Tinzen Open Computer Vision Test")
public class TinzenSlideIntoBanarasDms extends LinearOpMode {
    private OpenCvWebcam WebCam;
    private Mat workingMatrix = new Mat();
    public String position = "plCEHOLD3R";

    private TinzenBanaraDMPipeline Detector;

    @Override
    public void runOpMode() {
        WebCam.setPipeline(new TinzenBanaraDMPipeline());


        // init hardware here for later if Tinzen doesn't just Ctrl+c Ctrl+v this into later code

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        WebCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                WebCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
        while (!isStarted()) {
            position = Detector.position;
            telemetry.addData("Position : ", position);
        }

        //other code

    }

}
