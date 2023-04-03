package org.firstinspires.ftc.teamcode.Sandstorm;

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

    public boolean ToggledFlipConeHeight;
    public int FlipConeHeightState;
    public boolean AboveConeHeight;

    public boolean FlipConeToggleMode;
    public boolean FlipConeToggled;

    public boolean IntakeStackToggleMode;
    public boolean IntakeHeightCycleUp;
    public boolean IntakeToggleOut;
    public int IntakeToggleOutState;


    public boolean ManualResetToggleMode;
    public boolean ToggledManualReset;

    public boolean GroundJunctionsToggleMode;
    public boolean ToggledGroundJunctions;

    public boolean DriveToPositionToggleMode;
    public boolean ToggledDriveToPosition;

    public void resetMatchTimer(){matchTimer.reset();}

    public void inputsSetup(){ // must run this on init in dune drive
        Toggled = false;
        IntakeStackToggleMode = false;
        IntakeHeightCycleUp = false;

        IntakeToggleOut = false;
        IntakeToggleOutState = 0; // for fifth cone stack we have 0 height change

        FlipConeToggled = false;
        FlipConeToggleMode = false;

        ToggledFlipConeHeight = false;
        FlipConeHeightState = 0;
        AboveConeHeight = false;

        ToggledManualReset = false;
        ManualResetToggleMode = false;

        ToggledGroundJunctions = false;
        GroundJunctionsToggleMode = false;

        ToggledDriveToPosition = false;
        DriveToPositionToggleMode = false;
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

    public void groundJunctionsToggle(boolean togglebtn) {
        if (togglebtn) {
            if (!ToggledGroundJunctions) { // the first time you first press it it will change stuff, then won't go past this if statement
                if (GroundJunctionsToggleMode) {
                    GroundJunctionsToggleMode = false; // will have to access this variable in dune drive
                } else {
                    GroundJunctionsToggleMode = true;
                }
                ToggledGroundJunctions = true;
            }
        }
        else {
            ToggledGroundJunctions = false;
        }
    }

    public void driveToPositionToggle(boolean togglebtn) {
        if (togglebtn) {
            if (!ToggledDriveToPosition) { // the first time you first press it it will change stuff, then won't go past this if statement
                if (DriveToPositionToggleMode) {
                    DriveToPositionToggleMode = false; // will have to access this variable in other classes
                } else {
                    DriveToPositionToggleMode = true;
                }
                ToggledDriveToPosition = true;
            }
        }
        else {
            ToggledDriveToPosition = false;
        }
    }

    public void SlidesToggleUp(boolean cyclebtnup){
        if (cyclebtnup) {
            if (!IntakeToggleOut) {
                IntakeToggleOut = true;
                IntakeToggleOutState  = (IntakeToggleOutState  - 1) % 3;
            }
        }
        else {
            IntakeToggleOut = false;
        }
    }
    public void IntakeToggleOut (boolean intaketogglebtn){
        if (intaketogglebtn) {
            if (!IntakeToggleOut) {
                IntakeToggleOut = true;
                IntakeToggleOutState  = (IntakeToggleOutState  + 1) % 3;
            }
        }
        else {
            IntakeToggleOut = false;
        }
    }
    public void coneFlipOuttakeDownToggle(boolean togglebtn, boolean heightupbtn) {
        if (togglebtn) {
            if (!ToggledFlipConeHeight) {
                ToggledFlipConeHeight = true;
                FlipConeHeightState  = (FlipConeHeightState + 1) % 2; // as of right now the only states are go above cone, cone down on cone
                if (AboveConeHeight){
                    FlipConeHeightState = 1; // back to down on cone
                    AboveConeHeight = false; // makes it so that it doesn't go to above cone again
                }
            }
        }
        else {
            ToggledFlipConeHeight = false;
        }
        if (heightupbtn) { // at any point in the flipconeheightstate, if right bumper is pressed then it will go to here
            AboveConeHeight = true;
        }
    }

    public void flipConeToggleMode(boolean togglebtn) {
        if (togglebtn) {
            if (!FlipConeToggled) { // the first time you first press it it will change stuff, then won't go past this if statement
                if (FlipConeToggleMode) {
                    FlipConeToggleMode = false; // will have to access this variable in dune drive
                } else {
                    FlipConeToggleMode = true;
                }
                FlipConeToggled = true;
            }
        }
        else {
            FlipConeToggled = false;
        }
    }
    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
    public double angleWrapOtherSide(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians + Math.PI;
    }





}
