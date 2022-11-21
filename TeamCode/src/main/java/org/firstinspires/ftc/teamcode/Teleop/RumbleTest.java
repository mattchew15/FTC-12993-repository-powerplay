package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Rumble: Controllers", group = "Controller")

public class RumbleTest extends LinearOpMode{

    Gamepad.RumbleEffect testRumbleEffect;
    Gamepad.RumbleEffect halfRumbleEffect;
    Gamepad.RumbleEffect endgameRumbleEffect;
    Gamepad.RumbleEffect tenRumbleEffect;
    Gamepad.RumbleEffect onefourthRumbleEffect;
    Gamepad.RumbleEffect girlMode;

    @Override
    public void runOpMode() {

        testRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 250 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();

        onefourthRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 100)
                .build();

        halfRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 750)
                .build();

        endgameRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 250)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 250)
                .build();

        girlMode = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 5000)
                .build();

        tenRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 1000)
                .build();
        waitForStart();
        while(opModeIsActive()) {

            if(gamepad1.a){
                gamepad1.runRumbleEffect(testRumbleEffect);
            }
            else if(gamepad1.b){
                gamepad1.runRumbleEffect(onefourthRumbleEffect);
            }
            else if(gamepad1.x){
                gamepad1.runRumbleEffect(tenRumbleEffect);
            }
            else if(gamepad1.y){
                gamepad1.runRumbleEffect(halfRumbleEffect);
            }
            else if(gamepad1.dpad_up){
                gamepad1.runRumbleEffect(endgameRumbleEffect);
            }
            else if(gamepad1.dpad_down){
                gamepad1.runRumbleEffect(girlMode);
            }

        }
    }
}
