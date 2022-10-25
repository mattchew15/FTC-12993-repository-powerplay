package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.util.ElapsedTime;

// sensors or gamepad input methods will go in here, that opmodes can access them
public class Inputs {

    ElapsedTime matchTimer = new ElapsedTime();
    final double THIRTY_TO_ENDGAME = 60.0;
    final double TEN_TO_ENDGAME = 80.0;
    final double ENDGAME = 90.0;

    public void resetMatchTimer(){matchTimer.reset();}

    public void gamepadRumbleTimer (){
        if (matchTimer.seconds() > THIRTY_TO_ENDGAME){
            gamepad1.rumbleBlips(1); // this method will only run once when input is true
            gamepad2.rumbleBlips(1); // so this should work
            if (matchTimer.seconds() > TEN_TO_ENDGAME){
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
                if (matchTimer.seconds() > ENDGAME){
                    gamepad1.rumbleBlips(3);
                    gamepad2.rumbleBlips(3);
                }
            }
        }
    }
}
