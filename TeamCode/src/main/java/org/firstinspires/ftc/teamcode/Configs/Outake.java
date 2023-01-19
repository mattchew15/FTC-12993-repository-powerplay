package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Outake {

    private ServoImplEx OutakeClawS;
    private ServoImplEx OutakeTiltS;
    private Servo OutakeBraceS;
    private Servo OutakeAxelS;
    private DcMotorEx OutakeSlidesM;
    private DcMotorEx TurretM;
    private DigitalChannel OutakeLimitSwitch;

    //Servo position tune
    public static double ClawOpenPos = 0, ClawClosePos = 0, ClawOpenFullPos = 0, ClawCloseFullPos = 0;
    public static double TiltPickupPos = 0, TiltDepositPos = 0, TiltHoldPos = 0;
    public static double BraceUpPos = 0, BraceDownPos = 0;
    public static double AxelLowPos = 0, AxelMidPos = 0, AxelHighPos = 0, AxelFullHighPos = 0;

    //motor PID tune
    public static double OutakeSlideKp = 0, OutakeSlideKi = 0, OutakeSlideKd = 0, OutakeSlideIntegralSumLimit = 0, OutakeSlideFeedforward = 0;
    public static double TurretKp = 0, TurretKi = 0, TurretKd = 0, TurretIntegralSumLimit = 0, TurretSlideFeedForward = 0;

    //PID variables
    PID outakeSlidePID = new PID(OutakeSlideKp, OutakeSlideKi, OutakeSlideKd, OutakeSlideIntegralSumLimit, OutakeSlideFeedforward);
    PID turretPID = new PID(TurretKp, TurretKi, TurretKd, TurretIntegralSumLimit, TurretSlideFeedForward);

    public void Outake_init(HardwareMap hwMap){

        OutakeClawS = hwMap.get(ServoImplEx.class, "OutakeClawS");
        OutakeTiltS = hwMap.get(ServoImplEx.class, "OutakeTiltS");

        OutakeBraceS = hwMap.get(Servo.class, "OutakeBraceS");
        OutakeAxelS = hwMap.get(Servo.class, "OutakeAxelS");

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

}
