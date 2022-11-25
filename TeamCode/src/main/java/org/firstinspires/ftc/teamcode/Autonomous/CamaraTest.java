/*
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Camera: CameraTest", group = "Camera")

public class CamaraTest extends LinearOpMode {

    private WebcamName CameraRight;
    private WebcamName CameraLeft;

    AprilTagAutonomousInitDetectionExample aprDet = new AprilTagAutonomousInitDetectionExample();
    OpenCvCamera camera;
    String robotParkSpot = "Not detected";


    @Override
    public void runOpMode() {


        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Parking postion", robotParkSpot);

            if (aprDet.tagOfInterest.id == aprDet.MIDDLE) {
                robotParkSpot = "Middle";
            } else if (aprDet.tagOfInterest.id == aprDet.LEFT) {
                robotParkSpot = "Left";
            } else if (aprDet.tagOfInterest.id == aprDet.RIGHT) {
                robotParkSpot = "Right";
            }
            telemetry.addData("Parking position", robotParkSpot);

            telemetry.update();

            if (isStopRequested()) break;

        }
    }
}
*/

