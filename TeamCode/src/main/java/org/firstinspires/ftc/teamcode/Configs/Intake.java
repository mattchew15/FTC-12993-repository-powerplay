package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Intake {

    private ServoImplEx IntakeClawS;
    private ServoImplEx IntakeTiltS;
    private ServoImplEx IntakeHeightS;
    private DcMotor IntakeSpinM;
    private DcMotorEx IntakeSlidesM;
    private DigitalChannel IntakeLimitSwitch;
    int inSlidesTarget;

    //Servo position tune
    public static double ClawOpenPos = 0, ClawClosePos = 0, ClawOpenFullPos = 0, ClawCloseFullPos = 0;
    public static double TiltPickupPos = 0, TiltDepositPos = 0, TiltHoldPos = 0;
    public static double HeightUpPos = 0, HeightMidPos = 0, HeightDownPos = 0;

    //motor PID tune
    public static double IntakeSlideKp = 0, IntakeSlideKi = 0, IntakeSlideKd = 0, IntakeSlideIntegralSumLimit = 0, IntakeSlideKf = 0;

    //PID variables
    PID intakeSlidePID = new PID(IntakeSlideKp, IntakeSlideKi, IntakeSlideKd, IntakeSlideIntegralSumLimit, IntakeSlideKf);

    public void Intake_init(HardwareMap hwMap){

        IntakeClawS = hwMap.get(ServoImplEx.class, "IntakeClawS");
        IntakeTiltS = hwMap.get(ServoImplEx.class, "IntakeTiltS");
        IntakeHeightS = hwMap.get(ServoImplEx.class, "IntakeHeightS");

        IntakeSpinM = hwMap.get(DcMotorEx.class, "IntakeSpinM");
        IntakeSlidesM = hwMap.get(DcMotorEx.class, "IntakeSlideM");

        IntakeLimitSwitch = hwMap.get(DigitalChannel.class, "IntakeLimitSwitch");
        IntakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public boolean intakeTouchPressed() {
        return !IntakeLimitSwitch.getState();
    }


    public void motorsSetup(){
        IntakeSlidesM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        IntakeSlidesM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset encoders for start
    }

    public void encodersReset(){
        IntakeSlidesM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void outakeSlidesTo(int rotations, double motorPosition, double maxSpeed){
        inSlidesTarget = rotations;
        IntakeSlidesM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = intakeSlidePID.update(inSlidesTarget,motorPosition,maxSpeed); //does a lift to x with external PID instead of just regular encoders
        IntakeSlidesM.setPower(output);
    }

    public void intakeSpin(double speedDirection){
        IntakeSpinM.setPower(speedDirection);
    }

    public void inClawOpen(){IntakeClawS.setPosition(ClawOpenPos);}
    public void inClawClose(){IntakeClawS.setPosition(ClawClosePos);}
    public void inClawOpenFull(){IntakeClawS.setPosition(ClawOpenFullPos);}
    public void inClawCloseFull(){IntakeClawS.setPosition(ClawCloseFullPos);}
    public void inClawDisable(){IntakeClawS.setPwmDisable();}

    public void inTiltPick(){IntakeTiltS.setPosition(TiltPickupPos);}
    public void inTiltDepo(){IntakeTiltS.setPosition(TiltDepositPos);}
    public void inTiltHold(){IntakeTiltS.setPosition(TiltHoldPos);}
    public void inTiltDisable(){IntakeTiltS.setPwmDisable();}

    public void inHeightDown(){IntakeHeightS.setPosition(HeightDownPos);}
    public void inHeightUp(){IntakeHeightS.setPosition(HeightUpPos);}
    public void inHeightMid(){IntakeHeightS.setPosition(HeightMidPos);}
    public void inHeightDisable(){IntakeHeightS.setPwmDisable();}

}