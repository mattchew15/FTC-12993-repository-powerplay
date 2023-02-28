package org.firstinspires.ftc.teamcode.Dune.Autonomous.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class AnalogTest extends LinearOpMode {

    //get our analog input from the hardwareMap
    AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "myanaloginput");

    @Override
    public void runOpMode() {

        while(opModeIsActive()) {

            // get the voltage of our analog line
            // divide by 3.3 (the max voltage) to get a value between 0 and 1
            // multiply by 360 to convert it to 0 to 360 degrees
            double position = analogInput.getVoltage() / 3.3 * 360;

            telemetry.addData("Axon pos: ", position);

        }
    }
}
