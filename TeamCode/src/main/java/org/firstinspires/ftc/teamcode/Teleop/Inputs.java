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
    public boolean Toggled;
    public boolean ToggledManualReset;
    public boolean IntakeStackToggleMode;
    public boolean ManualResetToggleMode;
    public boolean IntakeHeightCycleUp;
    public boolean IntakeHeightCycleDown;
    public int IntakeHeightState;

    public void resetMatchTimer(){matchTimer.reset();}

    public void inputsSetup(){ // must run this on init in dune drive
        Toggled = false;
        IntakeStackToggleMode = false;
        IntakeHeightCycleUp = false;
        IntakeHeightCycleDown = false;
        IntakeHeightState = 0; // for fifth cone stack we have 0 height change
        ToggledManualReset = false;
        ManualResetToggleMode = false;
    }

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
    public void intakeStackToggleMode(boolean togglebtn) {
        if (togglebtn) {
            if (!Toggled) { // the first time you first press it it will change stuff, then won't go past this if statement
                if (IntakeStackToggleMode) {
                    IntakeStackToggleMode = false; // will have to access this variable in dune drive
                } else {
                    IntakeStackToggleMode = true;
                }
                Toggled = true;
            }
        }
        else {
            Toggled = false;

        }
    }

    public void manualResetToggleMode(boolean togglebtn) {
        if (togglebtn) {
            if (!ToggledManualReset) { // the first time you first press it it will change stuff, then won't go past this if statement
                if (ManualResetToggleMode) {
                    ManualResetToggleMode = false; // will have to access this variable in dune drive
                } else {
                    ManualResetToggleMode = true;
                }
                ToggledManualReset = true;
            }
        }
        else {
            ToggledManualReset = false;
        }
    }

    public void cycleToggleUp(boolean cyclebtnup){
        if (cyclebtnup) {
            if (!IntakeHeightCycleUp) {
                IntakeHeightCycleUp = true;
                IntakeHeightState = (IntakeHeightState - 1) % 5;
            }
        }
        else {
            IntakeHeightCycleUp = false;
        }
    }
    public void cycleToggleDown (boolean cyclebtndown){
        if (cyclebtndown) {
            if (!IntakeHeightCycleDown) {
                IntakeHeightCycleDown = true;
                IntakeHeightState = (IntakeHeightState + 1) % 5;
            }
        }
        else {
            IntakeHeightCycleDown = false;
        }
    }
}
