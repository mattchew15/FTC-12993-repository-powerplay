package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CamaraTest")
public class CamaraTest extends LinearOpMode {

    AprilTagAutonomousInitDetectionExample aprilTagDetection = new AprilTagAutonomousInitDetectionExample();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    String webcamName = "Webcam 1";
    String robotParkSpot = "Middle";


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(aprilTagDetection.tagsize, aprilTagDetection.fx, aprilTagDetection.fy, aprilTagDetection.cx, aprilTagDetection.cy);

        camera.setPipeline(aprilTagDetection.aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("error");
            }
        });
        telemetry.addData("Parking postion", robotParkSpot);

        waitForStart();
        while (opModeIsActive()) {
            if (aprilTagDetection.tagOfInterest == null || aprilTagDetection.tagOfInterest.id == aprilTagDetection.MIDDLE) {
                robotParkSpot = "Middle";
            } else if (aprilTagDetection.tagOfInterest.id == aprilTagDetection.LEFT) {
                robotParkSpot = "Left";
            } else {
                robotParkSpot = "Right";
            }
            telemetry.addData("Parking position", robotParkSpot);
            if (isStopRequested()) break;
            telemetry.update();

        }
    }
}
