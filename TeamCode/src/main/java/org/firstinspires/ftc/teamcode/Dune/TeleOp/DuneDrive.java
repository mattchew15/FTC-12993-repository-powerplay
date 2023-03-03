package org.firstinspires.ftc.teamcode.Dune.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dune.Robot;
import org.firstinspires.ftc.teamcode.Dune.Util;

@TeleOp
public class DuneDrive extends LinearOpMode {

    ElapsedTime stopWatch = new ElapsedTime();

    Robot robot = new Robot();
    Util util = new Util();

    //boolean manualReset;
    //double stickForLift = gamepad2.left_stick_x;
    //double stickForTurret = gamepad2.right_stick_x;
    //float manualLinkage = gamepad2.right_trigger;
    //boolean buttonReset = gamepad2.right_bumper;

    @Override
    public void runOpMode(){

        robot.Init(hardwareMap);

        robot.ClawClose();
        robot.Tilt0();
        robot.LinkageIn();
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()){

        }

        while (opModeIsActive()){


            robot.lift.setPower(gamepad2.left_stick_y * -0.8);
            robot.turret.setPower(gamepad2.right_stick_x * -0.8);
            /*if(buttonReset){
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/
            if(gamepad2.right_trigger <= 0.49 && gamepad2.right_trigger >= 0.05){
                robot.linkage.setPosition(gamepad2.right_trigger);
            }


            /*manualReset = false;
            while(!manualReset){
                manualReset = util.EncoderReset(gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_trigger, gamepad2.right_bumper);
            }*/
        }
    }
}
