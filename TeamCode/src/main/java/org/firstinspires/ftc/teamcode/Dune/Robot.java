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
    protected ServoImplEx claw;
    protected ServoImplEx linkage;
    protected ServoImplEx tilt;
    protected ServoImpl barLeft;
    protected ServoImpl barRight;

    //motors
    protected DcMotorImplEx lift;
    protected DcMotorEx turret;
    protected DcMotor intake;
    protected DcMotorEx fr;
    protected DcMotorEx fl;
    protected DcMotorEx br;
    protected DcMotorEx bl;

    //others
    private DigitalChannel touchClaw;



    //getting stuff ready
    public void Init(HardwareMap hwMap) {

        //servos
        claw = hwMap.get(ServoImplEx.class, "Claw");
        linkage = hwMap.get(ServoImplEx.class, "Linkage");
        tilt = hwMap.get(ServoImplEx.class, "Tilt");
        barLeft = hwMap.get(ServoImplEx.class, "Intake Left");
        barRight = hwMap.get(ServoImplEx.class, "Intake Right");

        //motors
        lift = hwMap.get(DcMotorImplEx.class, "Lift");
        turret = hwMap.get(DcMotorEx.class, "Turret");
        intake = hwMap.get(DcMotor.class, "Intake");
        fr = hwMap.get(DcMotorEx.class, "FR");
        fl = hwMap.get(DcMotorEx.class, "FL");
        br = hwMap.get(DcMotorEx.class, "BR");
        bl = hwMap.get(DcMotorEx.class, "BL");

        //others
        touchClaw = hwMap.get(DigitalChannel.class, "Touch Claw");
    }

    //positions
    public static double clawOpenPos = 0, clawClosePos = 0.5, clawDropPos = 0, clawGrabPos = 0;
    public static double linkageAuto = 0, linkageHalf = 0, linkageOut = 0, linkageIn = 0.49, linkageTQ = 0;
    public static double tiltHorizontal = 0, tiltHigh = 0, tiltMid = 0;
    public static double leftRest = 0, leftUp = 0, leftDown = 0;
    public static double rightRest = 0, rightUp = 0, rightDown = 0;

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
