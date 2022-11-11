package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Sensor: Rev2mDistance", group = "Sensor")

public class DistanceTest extends LinearOpMode {

    private DistanceSensor sensorRange;

    @Override
    public void runOpMode() {

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

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
            telemetry.addData("deviceName", sensorRange.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f inch", sensorRange.getDistance(DistanceUnit.INCH)));

            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();

        }

    }

}
