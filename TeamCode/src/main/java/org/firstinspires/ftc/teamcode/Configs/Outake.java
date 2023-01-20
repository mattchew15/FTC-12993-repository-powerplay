package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Outake {

    private ServoImplEx OutakeClawS;
    private ServoImplEx OutakeTiltS;
    private ServoImplEx OutakeBraceS;
    private ServoImplEx OutakeAxelS;
    private DcMotorEx OutakeSlidesM;
    private DcMotorEx TurretM;
    private DigitalChannel OutakeLimitSwitch;

    // final variables
    final double turretthresholdDistance = degreestoTicks(8); // should make the threshold less
    final double turretthresholdDistanceTwo = degreestoTicks(2);
    final double liftthresholdDistance = 60;
    int outSlideTarget;
    int turretTarget;

    //Servo position tune
    public static double ClawOpenPos = 0, ClawClosePos = 0, ClawOpenFullPos = 0, ClawCloseFullPos = 0;
    public static double TiltPickupPos = 0, TiltDepositPos = 0, TiltHoldPos = 0;
    public static double BraceUpPos = 0, BraceDownPos = 0;
    public static double AxelLowPos = 0, AxelMidPos = 0, AxelHighPos = 0, AxelFullHighPos = 0;

    //motor PID tune
    public static double OutakeSlideKp = 0, OutakeSlideKi = 0, OutakeSlideKd = 0, OutakeSlideIntegralSumLimit = 0, OutakeSlideKf = 0;
    public static double TurretKp = 0, TurretKi = 0, TurretKd = 0, TurretIntegralSumLimit = 0, TurretSlideKf = 0;

    //PID variables
    PID outakeSlidePID = new PID(OutakeSlideKp, OutakeSlideKi, OutakeSlideKd, OutakeSlideIntegralSumLimit, OutakeSlideKf);
    PID turretPID = new PID(TurretKp, TurretKi, TurretKd, TurretIntegralSumLimit, TurretSlideKf);

    public void Outake_init(HardwareMap hwMap){

        OutakeClawS = hwMap.get(ServoImplEx.class, "OutakeClawS");
        OutakeTiltS = hwMap.get(ServoImplEx.class, "OutakeTiltS");
        OutakeBraceS = hwMap.get(ServoImplEx.class, "OutakeBraceS");
        OutakeAxelS = hwMap.get(ServoImplEx.class, "OutakeAxelS");

        OutakeSlidesM = hwMap.get(DcMotorEx.class, "OutakeSlideM");
        TurretM = hwMap.get(DcMotorEx.class, "TurretM");

        OutakeLimitSwitch = hwMap.get(DigitalChannel.class, "OutakeLimitSwitch");
        OutakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public boolean outakeTouchPressed() {
        return !OutakeLimitSwitch.getState();
    }

    public void motorsSetup(){

        TurretM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OutakeSlidesM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        TurretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OutakeSlidesM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// reset encoders for start

    }

    public void encodersReset(){

        TurretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OutakeSlidesM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public double turretPos(){
        return TurretM.getCurrentPosition(); //loop times reading encoder?
    }

    public void turretSpinInternalPID(int rotations, double maxSpeed){
        turretTarget = rotations; // variable is public to this class?
        TurretM.setTargetPosition(turretTarget);
        TurretM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurretM.setPower(maxSpeed);
    }

    public boolean turretTargetReachedInteralPID(){
        if (turretPos() < (turretTarget + turretthresholdDistance) && turretPos() > (turretTarget-turretthresholdDistance)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean turretTargetReachedInteralPIDNewThreshold(){
        if (turretPos() < (turretTarget + turretthresholdDistanceTwo) && turretPos() > (turretTarget-turretthresholdDistanceTwo)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean liftTargetReached(){
        if (turretPID.returnError() < liftthresholdDistance){
            return true;
        }
        else{
            return false;
        }
    }

    public void outakeSlidesTo(int rotations, double motorPosition, double maxSpeed){
        outSlideTarget = rotations;
        OutakeSlidesM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = outakeSlidePID.update(outSlideTarget,motorPosition,maxSpeed); //does a lift to x with external PID instead of just regular encoders
        OutakeSlidesM.setPower(output);
    }

    public void outClawClose(){OutakeClawS.setPosition(ClawClosePos);}
    public void outClawOpen(){OutakeClawS.setPosition(ClawOpenPos);}
    public void outClawCloseFull(){OutakeClawS.setPosition(ClawCloseFullPos);}
    public void outClawOpenFull(){OutakeClawS.setPosition(ClawOpenFullPos);}
    public void outClawDisable(){OutakeClawS.setPwmDisable();}

    public void outTiltDepo(){OutakeTiltS.setPosition(TiltDepositPos);}
    public void outTiltHold(){OutakeTiltS.setPosition(TiltHoldPos);}
    public void outTiltPick(){OutakeTiltS.setPosition(TiltPickupPos);}
    public void outTiltDisable(){OutakeTiltS.setPwmDisable();}

    public void outBraceDown(){OutakeBraceS.setPosition(BraceDownPos);}
    public void outBraceUp(){OutakeBraceS.setPosition(BraceUpPos);}
    public void outBraceDisable(){OutakeBraceS.setPwmDisable();}

    public void outAxelFullHigh(){OutakeAxelS.setPosition(AxelFullHighPos);}
    public void outAxelHigh(){OutakeAxelS.setPosition(AxelHighPos);}
    public void outAxelLow(){OutakeAxelS.setPosition(AxelLowPos);}
    public void outAxelMid(){OutakeAxelS.setPosition(AxelMidPos);}
    public void outAxelDisable(){OutakeAxelS.setPwmDisable();}

    public double degreestoTicks(int degrees){
        return degrees * 7.6;
    }
    public double tickstoDegrees(int ticks){
        return ticks / 7.6;
    }

    public double returnPIDLiftError(){return outakeSlidePID.returnError();}
    public double returnPIDLiftOutput(){
        return outakeSlidePID.returnOutput();
    }
    public double returnIntegralSum(){
        return outakeSlidePID.returnIntegralSum();
    }

}
