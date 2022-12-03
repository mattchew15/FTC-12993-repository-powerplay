package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config // Allows dashboard to tune
public class TurretLift {  // no constructor for this class

    private DcMotorEx TurretMotor;
    private DcMotorEx LiftMotor;
    private ServoImplEx ClawServo;
    private ServoImplEx LinkageServo;
    private Servo TiltServo;
    private DigitalChannel sensorTouchClaw;
    AnalogInput linkagePosition;

    //config variables can be changed/tuned in dashboard
    public static double ClawOpenPos = 0.52, ClawClosedPos = 0.405, ClawCloseSoftPos = 0.45, ClawOpenHardPos = 0.61;
    public static double LinkageFullPos = 0, LinkageHalfPos = 0.16, LinkageQuarterPos = 0.26, LinkageClosedPos = 0.49, LinkageNearlyOutPos = 0.06;
    public static double TiltUpPos = 0.3, TiltDownPos = 0.87, TiltHalfPos = 0.48;

    //editable dashboard variables must be public static - PID values for turret and lift that can be tuned
    public static double TurretKp = 0.005, TurretKi = 0.001, TurretKd = 0.05, TurretIntegralSumLimit = 1, TurretFeedforward = 0.3;
    public static double LiftKp = 0.005, LiftKi = 0.01, LiftKd = 0.05, LiftIntegralSumLimit = 10, LiftKf = 0;

    // New instance of PID class with editable variables
    PID turretPID = new PID(TurretKp,TurretKi,TurretKd,TurretIntegralSumLimit,TurretFeedforward);
    PID liftPID = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);

    // final variables
    final double turretthresholdDistance = degreestoTicks(8); // should make the threshold less
    final double liftthresholdDistance = 60;
    int turretTarget;
    int liftTarget;


    public void TurretLift_init(HardwareMap hwMap) {
        TurretMotor = hwMap.get(DcMotorEx.class, "TurretMotor");
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");

        ClawServo = hwMap.get(ServoImplEx.class, "ClawS");
        LinkageServo = hwMap.get(ServoImplEx.class, "LinkageS");
        TiltServo = hwMap.get(Servo.class, "TiltS");
        sensorTouchClaw = hwMap.get(DigitalChannel.class, "sensor_touchClaw");
        sensorTouchClaw.setMode(DigitalChannel.Mode.INPUT);
        linkagePosition = hwMap.get(AnalogInput.class, "linkageEncoder");

    }


    public void motorsSetup(){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // run without encoder is if using external PID
        sensorTouchClaw.setMode(DigitalChannel.Mode.INPUT);
    }

    public void encodersReset(){
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // run without encoder is if using external PID
    }

    public void liftMotorRawControl(double manualcontrollift){
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setPower(manualcontrollift * -0.5);
    }

    public void turretMotorRawControl(double manualcontrolturret){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretMotor.setPower(manualcontrolturret * 0.7);
    }


    public void liftTo(int rotations, double motorPosition, double maxSpeed){
        liftTarget = rotations;
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
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

    public double getLiftVoltage (){
        return LiftMotor.getCurrent(CurrentUnit.AMPS);
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

    public boolean intakeTouchPressed() {
        return !sensorTouchClaw.getState();
    }
    // instead of using PID class uses the internal run to position on the motor

    public double getLinkagePosition(){
        double position = linkagePosition.getVoltage() / 3.3 * 360;
        return position;
    }

    public void turretSpinInternalPID(int rotations, double maxSpeed){
        turretTarget = rotations; // variable is public to this class?
        TurretMotor.setTargetPosition(turretTarget);
        TurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurretMotor.setPower(maxSpeed);
    }

    public boolean turretTargetReachedInteralPID(){
        if (turretPos() < (turretTarget + turretthresholdDistance) && turretPos() > (turretTarget-turretthresholdDistance)){
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
    public void releaseClaw(){
        ClawServo.setPwmDisable();
    }
    public void openClaw(){
        ClawServo.setPosition(ClawOpenPos);
    }
    public void closeClaw(){
        ClawServo.setPosition(ClawClosedPos);
    }
    public void closeClawSoft() {ClawServo.setPosition(ClawCloseSoftPos);}
    public void openClawHard(){
        ClawServo.setPosition(ClawOpenHardPos);}
    public void tiltUp(){
        TiltServo.setPosition(TiltUpPos);
    }
    public void tiltUpHalf(){TiltServo.setPosition(TiltHalfPos);}
    public void tiltReset(){
        TiltServo.setPosition(TiltDownPos);
    }
    public void linkageRelease(){
        LinkageServo.setPwmDisable();
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
    public void linkageNearlyOut(){LinkageServo.setPosition(LinkageNearlyOutPos);}

    public void readyServos(){
        openClaw();
        tiltReset();
        linkageIn();
    }

    public double degreestoTicks(int degrees){
        return degrees * 7.6;
    }
    public double tickstoDegrees(int ticks){
        return ticks / 7.6;
    }

    public double returnPIDLiftError(){
        return liftPID.returnError();
    }
    public double returnPIDLiftOutput(){
        return liftPID.returnOutput();
    }
    public double returnIntegralSum(){
        return liftPID.returnIntegralSum();
    }

    //state machine sequence functions? finite state machine implimentation
}
