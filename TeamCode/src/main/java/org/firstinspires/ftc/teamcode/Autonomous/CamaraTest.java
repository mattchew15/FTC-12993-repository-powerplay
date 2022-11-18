package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CamaraTest")
public class CamaraTest extends LinearOpMode {

    AprilTagAutonomousInitDetectionExample aprilTagDetection = new AprilTagAutonomousInitDetectionExample();

    String robotParkSpot = "Middle";

    @Override
    public void runOpMode() {

        waitForStart();

        if (aprilTagDetection.tagOfInterest == null || aprilTagDetection.tagOfInterest.id == aprilTagDetection.MIDDLE) {
            robotParkSpot = "Middle";
        }
        else if (aprilTagDetection.tagOfInterest.id == aprilTagDetection.LEFT) {
            robotParkSpot = "Left";
        }
        else {
            robotParkSpot = "Right";
        }
        telemetry.addData("Parking position", robotParkSpot);
    }
}
