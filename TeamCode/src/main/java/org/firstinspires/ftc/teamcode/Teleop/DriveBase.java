package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config // Allows dashboard to tune
public class DriveBase {  // no constructor for this class

    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor IntakeRMotor;
    private DcMotor IntakeLMotor;
    private Servo IntakeRightServo;
    private Servo IntakeLeftServo;

    //variable for the drivebase speed toggle;
    boolean PowerToggled;
    double PowerBase;
    double PowerBaseTurn;
    double PowerStrafe;

    // config servo position variables
    public static double IntakeRSRaisedPos = 0.765, IntakeLSRaisedPos = 0.145;
    public static double IntakeRSLoweredPos = 0.875, IntakeLSLoweredPos = 0.025;

    public void Drivebase_init(HardwareMap hwMap) {

        FR = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");
        BL = hwMap.get(DcMotor.class, "BL");
        FL = hwMap.get(DcMotor.class, "FL");

        IntakeRMotor = hwMap.get(DcMotor.class, "IntakeR");
        IntakeLMotor = hwMap.get(DcMotor.class, "IntakeL");

        IntakeRightServo = hwMap.get(Servo.class, "IntakeRightS");
        IntakeLeftServo = hwMap.get(Servo.class, "IntakeLeftS");

    }

    public void motorsSetup(){
        // zero brake behavior means when motors aren't powered, they will auto brake
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse correct motors
        FL.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeLMotor.setDirection(DcMotorSimple.Direction.REVERSE); // one intake motor is reversed
    }

    public void Drive(double LY, double LX, double RX) {
        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (LY*PowerBase - LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double backLeftPower = (LY*PowerBase + LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double frontRightPower = (LY*PowerBase + LX*PowerStrafe - RX*PowerBaseTurn) / denominator;
        double backRightPower = (LY*PowerBase - LX*PowerStrafe - RX*PowerBaseTurn) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }
    public void motorDirectionTest(double a, double b, double c, double d){
        FL.setPower(a);
        BL.setPower(b);
        FR.setPower(c);
        BR.setPower(d);
    }
    public int getMotorPosition(){
        return FL.getCurrentPosition();
    }
    public void runtoPositionTest(int Position){
        FL.setTargetPosition(Position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(1);
    }

    public void PowerToggle(boolean toggle) { // toggle code for a slow drive mode for fine adjustment
        if (toggle) {
            if (!PowerToggled) {
                if (PowerBase == 1) {
                    PowerBase = 0.33;
                    PowerBaseTurn = 0.3;
                    PowerStrafe = 0.36;
                } else {
                    //edit these values to change drivecode
                    PowerBase = 1;
                    PowerBaseTurn = 0.65;
                    PowerStrafe = 1.05;
                }
                PowerToggled = true;
            }
        }
        else {
            PowerToggled = false;
        }
    }

    public void intakeBarUp(){
        IntakeLeftServo.setPosition(IntakeLSRaisedPos);
        IntakeRightServo.setPosition(IntakeRSRaisedPos);
    }
    public void intakeBarDown(){
        IntakeLeftServo.setPosition(IntakeLSLoweredPos);
        IntakeRightServo.setPosition(IntakeRSLoweredPos);
    }
    public void intakeSpin(double speedDirection){
        IntakeRMotor.setPower(speedDirection);
        IntakeLMotor.setPower(speedDirection);
    }

}
