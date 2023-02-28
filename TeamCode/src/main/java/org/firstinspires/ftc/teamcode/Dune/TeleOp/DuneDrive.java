package org.firstinspires.ftc.teamcode.Dune.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dune.Util;

@TeleOp
public class DuneDrive extends LinearOpMode {

    ElapsedTime stopWatch = new ElapsedTime();
    Util util = new Util();

    boolean manualReset;

    public void runOpMode(){

        while (opModeInInit()){

        }

        while (opModeIsActive()){

            manualReset = false;
            while(!manualReset){
                manualReset = util.EncoderReset(gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_trigger, gamepad2.right_bumper);
            }
        }
    }
}
