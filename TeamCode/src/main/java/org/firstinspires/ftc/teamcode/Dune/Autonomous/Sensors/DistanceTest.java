package org.firstinspires.ftc.teamcode.Dune.Autonomous.Sensors;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "Sensor: Rev2mDistance", group = "Sensor")

public class DistanceTest extends LinearOpMode {

    private DistanceSensor sensorRangeLeft;
    private DistanceSensor sensorRangeRight;

    @Override
    public void runOpMode() {

        sensorRangeRight = hardwareMap.get(DistanceSensor.class, "sensor_rangeRight");
        sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "sensor_rangeLeft");

        Rev2mDistanceSensor sensorTimeOfFlightLeft = (Rev2mDistanceSensor)sensorRangeLeft;
        Rev2mDistanceSensor sensorTimeOfFlightRight = (Rev2mDistanceSensor)sensorRangeRight;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            /*
            if(sensorRange.getDistance(DistanceUnit.CM) <= 5) {
                //stuff
            }
            else {
                //stuff
            }
            */
            telemetry.addData("Left", sensorRangeLeft.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", sensorRangeLeft.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRangeLeft.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRangeLeft.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f inch", sensorRangeLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Right", sensorRangeRight.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", sensorRangeRight.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRangeRight.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRangeRight.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f inch", sensorRangeRight.getDistance(DistanceUnit.INCH)));

            telemetry.addData("Left", String.format("%x", sensorTimeOfFlightLeft.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlightLeft.didTimeoutOccur()));
            telemetry.addData("Right", String.format("%x", sensorTimeOfFlightRight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlightRight.didTimeoutOccur()));

            telemetry.update();

        }

    }

}
