package org.firstinspires.ftc.teamcode.Dune;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Util {

    Robot robot = new Robot();

    public void ServoStart(){
        AutoServoStart();
        robot.BarRest();
    }

    public void AutoServoStart() {
        robot.ClawOpen();
        robot.Tilt0();
        robot.LinkageIn();
    }


    //test this
    public boolean EncoderReset(double stickForLift, double stickForTurret, float manualLinkage, boolean buttonReset){
        robot.ClawClose();
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setPower(stickForLift * -0.5);
        robot.turret.setPower(stickForTurret * -0.5);
        if(buttonReset){
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return true;
        }
        else if(manualLinkage <= 0.48 && manualLinkage <= 0.1){
            robot.linkage.setPosition(manualLinkage);
        }
        else{
            robot.LinkageIn();
        }
        return false;
    }



}
