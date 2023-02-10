package org.firstinspires.ftc.teamcode.Dune.Autonomous.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@Autonomous(name = "Sensor", group = "Sensor")

public class UltrasonicTest extends LinearOpMode {

    private UltrasonicSensor sensorUltrasonicLeft;
    private UltrasonicSensor sensorUltrasonicRight;

    @Override
    public void runOpMode() {

        sensorUltrasonicLeft = hardwareMap.get(UltrasonicSensor.class, "sensor_UltraLeft");
        sensorUltrasonicRight = hardwareMap.get(UltrasonicSensor.class, "sensor_UltraRight");

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            double sensLeft = sensorUltrasonicLeft.getUltrasonicLevel();
            double sensRight = sensorUltrasonicRight.getUltrasonicLevel();

            if(sensLeft > 0) {
                telemetry.addData("Left Ultra", "sensed");
            }
            else {
                telemetry.addData("Left Ultra", "not sensed");
            }

            if(sensRight > 0) {
                telemetry.addData("Right Ultra", "sensed");
            }
            else {
                telemetry.addData("Right Ultra", "not sensed");
            }

            telemetry.update();

        }
    }
}
