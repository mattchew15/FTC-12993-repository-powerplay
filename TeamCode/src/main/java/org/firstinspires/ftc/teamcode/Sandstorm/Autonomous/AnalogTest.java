package org.firstinspires.ftc.teamcode.Sandstorm.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
@Disabled
@TeleOp
public class AnalogTest extends LinearOpMode {

    //get our analog input from the hardwareMap


    @Override
    public void runOpMode() {
        AnalogInput analogAxon = hardwareMap.get(AnalogInput.class, "Axon");
        ServoImplEx axon = hardwareMap.get(ServoImplEx.class, "Axon");


        waitForStart();
        while(opModeIsActive()) {

            // get the voltage of our analog line
            // divide by 3.3 (the max voltage) to get a value between 0 and 1
            // multiply by 360 to convert it to 0 to 360 degrees
            double position = analogAxon.getVoltage() / 3.3 * 360;

            axon.setPosition(gamepad1.right_trigger);

            telemetry.addData("Axon pos: ", position);
            telemetry.update();

        }
    }
}

