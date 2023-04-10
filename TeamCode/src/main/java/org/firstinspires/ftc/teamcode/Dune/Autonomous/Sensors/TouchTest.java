package org.firstinspires.ftc.teamcode.Dune.Autonomous.Sensors;

import android.text.method.Touch;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Autonomous(name = "Sensor: RevTouchSensor", group = "Sensor")

public class TouchTest extends LinearOpMode {

    private DigitalChannel sensorTouchClaw;

    @Override
    public void runOpMode(){

        sensorTouchClaw = hardwareMap.get(DigitalChannel.class, "sensor_touchClaw");

        sensorTouchClaw.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            if(sensorTouchClaw.getState() == false){
                telemetry.addData("Digital touch", "Is pressed");
            }
            else {
                telemetry.addData("Touch shit", "is not pressed");
            }

            telemetry.update();

        }
    }
}


