package org.firstinspires.ftc.teamcode.Dune.Autonomous.Sensors;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceDetection extends LinearOpMode { //either linear op mode or whatever is needed depending on where you code it

    private DistanceSensor sensorRangeLeft;
    private DistanceSensor sensorRangeRight;

    @Override
    public void runOpMode() {

        sensorRangeRight = hardwareMap.get(DistanceSensor.class, "sensor_rangeRight");
        sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "sensor_rangeLeft");

        Rev2mDistanceSensor sensorTimeOfFlightLeft = (Rev2mDistanceSensor) sensorRangeLeft;
        Rev2mDistanceSensor sensorTimeOfFlightRight = (Rev2mDistanceSensor) sensorRangeRight;

        waitForStart();
        while (opModeIsActive()) {

            double distanceAverage = ((sensorRangeLeft.getDistance(DistanceUnit.INCH)) + (sensorRangeRight.getDistance(DistanceUnit.INCH))) / 2;
            // add DistanceDetection.distanceAverage to the auto or meepmeep or whatever and thats all done
            // then just use it as location finder(i can also do it if i know where)
/*
            telemetry.addData("Left", sensorRangeLeft.getDeviceName());
            telemetry.addData("range", String.format("%.01f cm", sensorRangeLeft.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right", sensorRangeRight.getDeviceName());
            telemetry.addData("range", String.format("%.01f cm", sensorRangeRight.getDistance(DistanceUnit.CM)));


            telemetry.addData("Left", String.format("%x", sensorTimeOfFlightLeft.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlightLeft.didTimeoutOccur()));
            telemetry.addData("Right", String.format("%x", sensorTimeOfFlightRight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlightRight.didTimeoutOccur()));

            telemetry.update();

 */ // add this to the auto (you can also switch it up to be in inches(INCH) instead of Centimeters(CM))
        }
    }
}


