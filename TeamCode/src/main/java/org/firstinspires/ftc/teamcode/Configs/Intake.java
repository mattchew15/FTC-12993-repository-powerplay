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
    private Servo IntakeHeightS;
    private DcMotorEx IntakeSpinM;
    private DcMotorEx IntakeSlidesM;
    private DigitalChannel IntakeLimitSwitch;

    //Servo position tune
    public static double ClawOpenPos = 0, ClawClosePos = 0, ClawOpenFullPos = 0, ClawCloseFullPos = 0;
    public static double TiltPickupPos = 0, TiltDepositPos = 0, TiltHoldPos = 0;
    public static double HeightUpPos = 0, HeightMidPos = 0, HeightDownPos = 0;

    //motor PID tune
    public static double IntakeSlideKp = 0, IntakeSlideKi = 0, IntakeSlideKd = 0, IntakeSlideIntegralSumLimit = 0, IntakeSlideFeedforward = 0;

    //PID variables
    PID intakeSlidePID = new PID(IntakeSlideKp, IntakeSlideKi, IntakeSlideKd, IntakeSlideIntegralSumLimit, IntakeSlideFeedforward);

    public void Intake_init(HardwareMap hwMap){

        IntakeClawS = hwMap.get(ServoImplEx.class, "IntakeClawS");
        IntakeTiltS = hwMap.get(ServoImplEx.class, "IntakeTiltS");

        IntakeHeightS = hwMap.get(Servo.class, "IntakeHeightS");

        IntakeSpinM = hwMap.get(DcMotorEx.class, "IntakeSpinM");
        IntakeSlidesM = hwMap.get(DcMotorEx.class, "IntakeSlideM");

        IntakeLimitSwitch = hwMap.get(DigitalChannel.class, "IntakeLimitSwitch");
        IntakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public boolean intakeTouchPressed() {

        return !IntakeLimitSwitch.getState();

    }


    public void motorsSetup(){

        IntakeSpinM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeSlidesM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        IntakeSpinM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeSlidesM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset encoders for start

    }

}