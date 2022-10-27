package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    public static double ClawOpenPos = 0.28, ClawClosedPos = 0.4;
    public static double LinkageFullPos = 0.3, LinkageHalfPos = 0.16, LinkageQuarterPos = 0.1, LinkageClosedPos = 0.01;
    public static double TiltUpPos = 0.3, TiltDownPos = 0.85, TiltHalfPos = 0.48;

    //editable dashboard variables must be public static - PID values for turret and lift that can be tuned
    public static double TurretKp = 0.1, TurretKi = 0.001, TurretKd = 0.05;
    public static double LiftKp = 0.1, LiftKi = 0.001, LiftKd = 0.05;

    // New instance of PID class with editable variables
    PID turretPID = new PID(TurretKp,TurretKi,TurretKd);
    PID liftPID = new PID(LiftKp,LiftKi,LiftKd);

    // final variables
    final double turretthresholdDistance = degreestoTicks(15);
    final double liftthresholdDistance = 60;
    int turretTarget;
    int liftTarget;

    public void TurretLift_init(HardwareMap hwMap) {
        TurretMotor = hwMap.get(DcMotorEx.class, "TurretMotor");
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");

        ClawServo = hwMap.get(Servo.class, "ClawS");
        LinkageServo = hwMap.get(Servo.class, "LinkageS");
        TiltServo = hwMap.get(Servo.class, "TiltS");

    }

    public void motorsSetup(){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // run without encoder is if using external PID
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
    public void liftToInternalPID(int rotations, double maxSpeed){
        liftTarget = rotations;
        //telemetry.addData("lifttarget", liftTarget);
        LiftMotor.setTargetPosition(liftTarget);
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

    public boolean turretTargetReached(){
        if (turretPID.returnError() < turretthresholdDistance){
            return true;
        }
        else{
            return false;
        }
    }
    // instead of using PID class uses the internal run to position on the motor
    public void turretSpinInternalPID(int rotations, double maxSpeed){
        turretTarget = rotations; // variable is public to this class?
        TurretMotor.setTargetPosition(turretTarget);
        TurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurretMotor.setPower(maxSpeed);
    }

    public boolean turretTargetReachedInteralPID(){
        if (Math.abs(turretTarget) == Math.abs(turretPos())){
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

    public boolean liftTargetReachedInternalPID(){ // if it works for lift then do for turret
        if (liftPos() < (liftTarget + liftthresholdDistance) && liftPos() > (liftTarget-liftthresholdDistance)){ //liftthresholdDistance
            return true;
        }
        else{
            return false;
        }
    }




    // servo void functions
    public void openClaw(){
        ClawServo.setPosition(ClawOpenPos);
    }
    public void closeClaw(){
        ClawServo.setPosition(ClawClosedPos);
    }
    public void tiltUp(){
        TiltServo.setPosition(TiltUpPos);
    }
    public void tiltUpHalf(){TiltServo.setPosition(TiltHalfPos);}
    public void tiltReset(){
        TiltServo.setPosition(TiltDownPos);
    }
    public void linkageOut(){
        LinkageServo.setPosition(LinkageFullPos);
    }
    public void linkageOutHalf(){
        LinkageServo.setPosition(LinkageHalfPos);
    }
    public void linkageOutQuarter(){LinkageServo.setPosition(LinkageQuarterPos);}
    public void linkageIn(){
        LinkageServo.setPosition(LinkageClosedPos);
    }

    public void readyServos(){
        openClaw();
        tiltReset();
        linkageIn();
    }

    public double degreestoTicks(int degrees){
        return degrees * 9;
    }
    public double tickstoDegrees(int ticks){
        return ticks / 9;
    }


    //state machine sequence functions? finite state machine implimentation
}
