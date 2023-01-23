package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Rumble: Controllers", group = "Controller")

public class RumbleTest extends LinearOpMode{

    //Timing for rumble
    ElapsedTime runtime = new ElapsedTime();

    final double halfTime = 60.0;
    final double endgame = 90.0;
    final double tenTime = 110.0;
    final double oneFourth = 30.0;

    Gamepad.RumbleEffect testRumbleEffect;
    Gamepad.RumbleEffect halfRumbleEffect;
    Gamepad.RumbleEffect endgameRumbleEffect;
    Gamepad.RumbleEffect tenRumbleEffect;
    Gamepad.RumbleEffect onefourthRumbleEffect;
    Gamepad.RumbleEffect girlMode;

    boolean one = true;
    boolean two = false;
    boolean three = false;
    boolean four = false;

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
        runtime.reset();
        while(opModeIsActive()) {

            telemetry.addData(">>", runtime);
            telemetry.update();

            if(gamepad1.a){
                gamepad1.runRumbleEffect(testRumbleEffect);
            }
            if(gamepad1.b || (runtime.seconds() > oneFourth && one)){
                gamepad1.runRumbleEffect(onefourthRumbleEffect);
                one = false;
                two = true;
            }
            if(gamepad1.y || (runtime.seconds() > halfTime) && two){
                gamepad1.runRumbleEffect(halfRumbleEffect);
                two = false;
                three = true;
            }
            if(gamepad1.dpad_up || (runtime.seconds() > endgame) && three){
                gamepad1.runRumbleEffect(endgameRumbleEffect);
                three = false;
                four = true;
            }
            if(gamepad1.x || (runtime.seconds() > tenTime && four)){
                gamepad1.runRumbleEffect(tenRumbleEffect);
                four = false;
            }
            if(gamepad1.dpad_down){
                gamepad1.runRumbleEffect(girlMode);
            }

        }
    }
}
