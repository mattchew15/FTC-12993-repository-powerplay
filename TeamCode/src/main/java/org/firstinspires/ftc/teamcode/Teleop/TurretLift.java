package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config // Allows dashboard to tune
public class TurretLift {  // no constructor for this class

    private DcMotorEx TurretMotor;
    private DcMotorEx LiftMotor;
    private Servo ClawServo;
    private Servo LinkageServo;
    private Servo TiltServo;

    //config variables can be changed/tuned in dashboard
    public static double ClawOpenPos = 0, ClawClosedPos = 1;
    public static double LinkageFullPos = 0.3, LinkageHalfPos = 0.16, LinkageClosedPos = 0.01;
    public static double TiltUpPos = 0.3, TiltDownPos = 0.85;

    //editable dashboard variables must be public static - PID values for turret and lift that can be tuned
    public static double TurretKp = 0.1, TurretKi = 0.001, TurretKd = 0.05;
    public static double LiftKp = 0.1, LiftKi = 0.001, LiftKd = 0.05;

    // New instance of PID class with editable variables
    PID turretPID = new PID(TurretKp,TurretKi,TurretKd);
    PID liftPID = new PID(LiftKp,LiftKi,LiftKd);


    public void TurretLift_init(HardwareMap hwMap) {
        TurretMotor = hwMap.get(DcMotorEx.class, "TurretMotor");
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");

        ClawServo = hwMap.get(Servo.class, "ClawS");
        LinkageServo = hwMap.get(Servo.class, "LinkageS");
        TiltServo = hwMap.get(Servo.class, "TiltS");

    }

    public void motorsSetup(){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID
    }

    public void liftTo(double rotations, double motorPosition, double maxSpeed){
        double output = liftPID.update(rotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        LiftMotor.setPower(output);
    }

    // returns get current position so that when setting the current state for the PID loop
    // we can do it from this class, and not init the motor in DuneDrive
    public double liftPos(){
        return LiftMotor.getCurrentPosition();
    }

    // instead of using PID class uses the internal run to position on the motor
    public void liftToInternalPID(int rotations, double motorPosition, double maxSpeed){
        LiftMotor.setTargetPosition(rotations);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(maxSpeed);
    }

    public void turretSpin(double targetRotations, double motorPosition, double maxSpeed){
        double output = turretPID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        TurretMotor.setPower(output);
    }

    // returns get current position so that when setting the current state for the PID loop
    // we can do it from this class, and not init the motor in DuneDrive
    public double turretPos(){
        return TurretMotor.getCurrentPosition(); //loop times reading encoder?
    }

    public boolean turretTargetReached(double thresholdDistance){
        if (turretPID.returnError() < thresholdDistance){
            return true;
        }
        else{
            return false;
        }
    }

    // instead of using PID class uses the internal run to position on the motor
    public void turretSpinInternalPID(int rotations, double motorPosition, double maxSpeed){
        TurretMotor.setTargetPosition(rotations);
        TurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurretMotor.setPower(maxSpeed);
    }


    // servo void functions
    public void openClaw(){
        ClawServo.setPosition(ClawOpenPos);
    }
    public void closeClaw(){
        ClawServo.setPosition(ClawOpenPos);
    }
    public void tiltUp(){
        TiltServo.setPosition(TiltUpPos);
    }
    public void tiltReset(){
        TiltServo.setPosition(TiltDownPos);
    }
    public void linkageOut(){
        LinkageServo.setPosition(LinkageFullPos);
    }
    public void linkageOutHalf(){
        LinkageServo.setPosition(LinkageHalfPos);
    }
    public void linkageIn(){
        LinkageServo.setPosition(LinkageClosedPos);
    }

    //state machine sequence functions? finite state machine implimentation
}
