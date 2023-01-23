package org.firstinspires.ftc.teamcode.Sensors;

import android.graphics.Color;
import android.provider.CalendarContract;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Autonomous(name = "Sensor: ColorSensor", group = "Sensor")

public class ColorTest extends LinearOpMode {

    NormalizedColorSensor sensorColorIntake;

    @Override
    public void runOpMode() {

        sensorColorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_ColorIntake");


        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            NormalizedRGBA colors = sensorColorIntake.getNormalizedColors();

            telemetry.addLine()
                    .addData("Red", "%.6f", colors.red)
                    .addData("\nBlue", "%.6f", colors.blue);

            telemetry.update();

        }
    }
}