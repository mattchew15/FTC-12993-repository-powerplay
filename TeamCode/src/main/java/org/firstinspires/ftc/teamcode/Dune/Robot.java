package org.firstinspires.ftc.teamcode.Dune;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Robot {

    //servos
    public ServoImplEx claw;
    public ServoImplEx linkage;
    public ServoImplEx tilt;
    public ServoImpl barLeft;
    public ServoImpl barRight;

    //motors
    public DcMotorImplEx lift;
    public DcMotorEx turret;
    public DcMotor intake;
    public DcMotorEx fr;
    public DcMotorEx fl;
    public DcMotorEx br;
    public DcMotorEx bl;

    //others
    public DigitalChannel touchClaw;

    //positions
    public static double clawOpenPos = 0.49, clawClosePos = 0.32, clawDropPos = 0.53, clawGrabPos = 0.4;
    public static double linkageAuto = 0, linkageHalf = 0, linkageOut = 0.05, linkageIn = 0.49, linkageTQ = 0;
    public static double tiltHorizontal = 0.5, tiltHigh = 0.1, tiltMid = 0.2;
    public static double leftRest = 0.28, leftUp = 0.15, leftDown = 0.44;
    public static double rightRest = 0.32, rightUp = 0.46, rightDown = 0.18;



    //getting stuff ready
    public void Init(HardwareMap hwMap) {

        //servos
        claw = hwMap.get(ServoImplEx.class, "ClawS");
        linkage = hwMap.get(ServoImplEx.class, "LinkageS");
        tilt = hwMap.get(ServoImplEx.class, "TiltS");
        barLeft = hwMap.get(ServoImplEx.class, "IntakeLeftS");
        barRight = hwMap.get(ServoImplEx.class, "IntakeRightS");

        //motors
        lift = hwMap.get(DcMotorImplEx.class, "LiftMotor");
        turret = hwMap.get(DcMotorEx.class, "TurretMotor");
        intake = hwMap.get(DcMotor.class, "IntakeMotor");
        fr = hwMap.get(DcMotorEx.class, "FR");
        fl = hwMap.get(DcMotorEx.class, "FL");
        br = hwMap.get(DcMotorEx.class, "BR");
        bl = hwMap.get(DcMotorEx.class, "BL");

        //others
        touchClaw = hwMap.get(DigitalChannel.class, "sensor_touchClaw");
    }



    //setting positions
    public void ClawOpen(){claw.setPosition(clawOpenPos);}
    public void ClawClose(){claw.setPosition(clawClosePos);}
    public void ClawDrop(){claw.setPosition(clawDropPos);}
    public void ClawGrab(){claw.setPosition(clawGrabPos);}

    public void LinkageAuto(){linkage.setPosition(linkageAuto);}
    public void LinkageHalf(){linkage.setPosition(linkageHalf);}
    public void LinkageIn(){linkage.setPosition(linkageIn);}
    public void LinkageOut(){linkage.setPosition(linkageOut);}
    public void LinkageTQ(){linkage.setPosition(linkageTQ);}

    public void Tilt0(){tilt.setPosition(tiltHorizontal);}
    public void TiltHigh(){tilt.setPosition(tiltHigh);}
    public void TiltMid(){tilt.setPosition(tiltMid);}

    public void BarRest(){
        barLeft.setPosition(leftRest);
        barRight.setPosition(rightRest);
    }
    public void BarUp(){
        barLeft.setPosition(leftUp);
        barRight.setPosition(rightUp);
    }
    public void BarDown(){
        barLeft.setPosition(leftDown);
        barRight.setPosition(rightDown);
    }


}
