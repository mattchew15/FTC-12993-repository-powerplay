package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;
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
    public static double ClawOpenPos =, ClawClosePos =, ClawOpenFullPos =, ClawCloseFullPos =;
    public static double TiltPickupPos =, TiltDepositPos =, TiltHoldPos =;
    public static double BraceUpPos =, BraceDownPos =;
    public static double AxelLowPos =, AxelMidPos =, AxelHighPos =, AxelFullHighPos;

    //motor PID tune
    public static double OutakeSlideKp =, OutakeSlideKi =, OutakeSlideKd =, OutakeSlideIntegralSumLimit =, OutakeSlideFeedforward =;
    public static double TurretKp =, TurretKi =, TurretKd =, TurretIntegralSumLimit =, TurretSlideFeedForward =;

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

}