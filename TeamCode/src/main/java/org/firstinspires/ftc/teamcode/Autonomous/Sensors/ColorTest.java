package org.firstinspires.ftc.teamcode.Autonomous.Sensors;

import android.graphics.Color;
import android.provider.CalendarContract;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Sensor: ColorSensor", group = "Sensor")

public class ColorTest extends LinearOpMode {

    private ColorSensor sensorColorIntake;

    @Override
    public void runOpMode() {

        sensorColorIntake = hardwareMap.get(ColorSensor.class, "sensor_ColorIntake");

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {



        }
    }
}