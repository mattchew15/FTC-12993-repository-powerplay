package org.firstinspires.ftc.teamcode.Sandstorm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Dune.PID;

@Config // Allows dashboard to tune
public class Outtake {  // no constructor for this class

    // remove all double ups of reading and writing from hubs

    private DcMotorEx TurretMotor;
    private DcMotorEx LiftMotor;
    private DcMotorEx IntakeSlideMotor;
    // outtake servos
    private ServoImplEx OuttakeClawServo;
    private ServoImplEx OuttakeArmServo;
    private ServoImplEx OuttakeSlideServo;
    private ServoImplEx OuttakeBraceServo;

    private ServoImplEx IntakeArmServo;
    private ServoImplEx IntakeSlideServo;
    private ServoImplEx IntakeClawServo;

    private ServoImplEx ConeArmServo;
    private ServoImplEx IntakeClipServo;

    private DigitalChannel IntakeClawTouch;
    private DigitalChannel OuttakeClawTouch;
    AnalogInput IntakeArmPosition;
    AnalogInput IntakeLiftPosition;

    //Servo Positions for outtake
    public static double OuttakeClawOpenPos = 0.58, OuttakeClawClosedPos = 0.45, OuttakeClawOpenHardPos = 0.7;
    public static double OuttakeArmReadyPos = 0.732, OuttakeArmDepositPos = 0.095, OuttakeArmPickupPos = 0.095, OuttakeArmScorePos = 0.22, OuttakeArmScoreAutoPos = 0.26, OuttakeArmSlightlyTiltedUpPos = 0.3, OuttakeArmUprightPos = 0.57;
    public static double BraceReadyPos = 0.55, BraceActivePos = 0.72, BraceActivePosAuto = 0.64, BraceTuckedPos = 0, BraceFlipConePos = 0.6;
    public static double OuttakeSlideReadyPos = 0.03, OuttakeSlideScorePos = 0.03, OuttakeSlideScoreDropPos = 0.16, OuttakeSlideGroundPos =  0.305, OuttakeSlideConeFlipPos = 0.18, OuttakeSlideAboveConePos = 0.245;

    // Servo Position for ConeArm
    public static double ConeArmReadyPos = 0.15, ConeArmAboveConePos = 0.53, ConeArmDownOnConePos = 0.575;
    public static double IntakeClipHold = 0.712, IntakeClipOpen = 0.575;

    //Servo Positions for Intake
    public static double IntakeClawOpenPos = 0.675, IntakeClawClosedPos = 0.73, IntakeClawOpenHardPos = 0.57;
    public static double IntakeArmReadyPos = 0.908, IntakeArmTransferPos = 0.448, IntakeArmCOneHoldForTransferPos = 0.6;
    public static double IntakeLiftReadyPos = 0.43, IntakeLiftTransferPos = 0.215;

    //Servo Positions for Stack Height
    public static double IntakeHeight5 = 0.145, IntakeHeight4 = 0.241, IntakeHeight3 = 0.32, IntakeHeight2 = 0.39, IntakeHeight1 = 0.435;

    //editable dashboard variables must be public static - PID values for turret and lift that can be tuned
    public static double TurretKp = 0.012, TurretKi = 0.000, TurretKd = 0.0004, TurretIntegralSumLimit = 1, TurretFeedforward = 0.3;
    public static double LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00035, LiftIntegralSumLimit = 10, LiftKf = 0;
    public static double intakeSlideKp = 0.015, intakeSlideKi = 0.00, intakeSlideKd = 0.0005, intakeSlideIntegralSumLimit = 10, intakeSlideKf = 0;

    // New instance of PID class with editable variables
    PID turretPID = new PID(TurretKp,TurretKi,TurretKd,TurretIntegralSumLimit,TurretFeedforward);
    PID liftPID = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    PID intakeSlidePID = new PID(intakeSlideKp,intakeSlideKi,intakeSlideKd,intakeSlideIntegralSumLimit,intakeSlideKf);

    // final variables
    final double turretthresholdDistance = degreestoTicks(8); // should make the threshold less
    final double turretthresholdDistanceTwo = degreestoTicks(2);

    final double liftthresholdDistance = 20;
    final double intakeSlidethresholdDistance = 20;
    final double intakeSlidethresholdDistanceNewThreshold = 4;

    double turretTarget;
    int liftTarget;
    int intakeSlideTarget;

    public double turretPosition;
    public double liftPosition;
    public double intakeSlidePosition;
    public double intakeArmPosition;
    public double intakeLiftPosition;


    public void Outtake_init(HardwareMap hwMap) {
        TurretMotor = hwMap.get(DcMotorEx.class, "TurretMotor");
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");
        IntakeSlideMotor = hwMap.get(DcMotorEx.class, "IntakeSlideMotor");

        OuttakeClawServo = hwMap.get(ServoImplEx.class, "OutClawS");
        OuttakeArmServo = hwMap.get(ServoImplEx.class, "OutArmS");
        OuttakeSlideServo = hwMap.get(ServoImplEx.class, "OutSlideS");
        OuttakeBraceServo = hwMap.get(ServoImplEx.class, "OutBraceS");
        IntakeArmServo = hwMap.get(ServoImplEx.class, "InArmS");
        IntakeSlideServo = hwMap.get(ServoImplEx.class, "InLiftS");
        IntakeClawServo = hwMap.get(ServoImplEx.class, "InClawS");
        ConeArmServo = hwMap.get(ServoImplEx.class, "ConeArmS");
        IntakeClipServo = hwMap.get(ServoImplEx.class, "InClipS");

        IntakeClawTouch = hwMap.get(DigitalChannel.class, "InTouch");
        //OuttakeClawTouch = hwMap.get(DigitalChannel.class, "OuttakeTouch");

        IntakeArmPosition = hwMap.get(AnalogInput.class, "InSEncoder");
        //OuttakeArmPosition = hwMap.get(AnalogInput.class, "OutSEncoder");
        IntakeLiftPosition = hwMap.get(AnalogInput.class, "InLiftSEncoder");
    }

    public void hardwareSetup(){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // this should make it so that we don't use a pid on the main loop and take up power
        IntakeClawTouch.setMode(DigitalChannel.Mode.INPUT);
        //OuttakeArmServo.setDirection(Servo.Direction.REVERSE);

        //turretPosition = 0; // these need to be initialized on setup or you will get null error
        //liftPosition = 0;
        //intakeSlidePosition = 0;
    }
    public void outtakeReads(){
        turretPosition = turretPos();
        liftPosition = liftPos();
        intakeSlidePosition = IntakeSlidePos();
        intakeLiftPosition = getIntakeLiftPos();
        intakeArmPosition = getIntakeArmPos();
    }

    public void encodersReset(){
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftMotorRawControl(double manualcontrollift){
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is a write that is not needed
        LiftMotor.setPower(manualcontrollift * 0.7);
    }

    public void turretMotorRawControl(double manualcontrolturret){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretMotor.setPower(manualcontrolturret * -0.3);
    }
    public void intakeSlideMotorRawControl(double manualcontrolintakeslide){ // shouldn't have to do this - will be too slow
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeSlideMotor.setPower(manualcontrolintakeslide * -0.6);
    }

    public void liftTo(int rotations, double motorPosition, double maxSpeed){
        liftTarget = rotations;
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        LiftMotor.setPower(output);
    }
    /*
    public void liftToDeacceleration(int rotations, double motorPosition, double maxSpeed, double accelerationRate, double deaccelerationRate, double accelerationDistance, double deccelerationDistance){
        liftTarget = rotations;
        double liftPosition = liftPosition - startPosition;
        double output;

        if (liftPosition < accelerationDistance) {
            output = liftPID.update(liftTarget,motorPosition,(maxSpeed/accelerationDistance)*liftPosition);

        } else if (liftPosition > rotations - deccelerationDistance) {
            output = liftPID.update(liftTarget,motorPosition,(maxSpeed/deccelerationDistance)*(rotations-liftPosition));

        } else {
            output = liftPID.update(liftTarget,motorPosition,maxSpeed);
        }

        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        LiftMotor.setPower(output);
    }

     */

    public void turretSpin(double targetRotations, double motorPosition, double maxSpeed){
        turretTarget = targetRotations;
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = turretPID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        TurretMotor.setPower(output);
    }
    public void IntakeSlideTo(int targetRotations, double motorPosition, double maxSpeed){
        intakeSlideTarget = targetRotations;
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = intakeSlidePID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        IntakeSlideMotor.setPower(output);
    }

    // returns get current position so that when setting the current state for the PID loop
    public double liftPos(){
        return LiftMotor.getCurrentPosition();
    }
    public double getLiftVoltage (){
        return LiftMotor.getCurrent(CurrentUnit.AMPS);
    }
    public double turretPos(){return TurretMotor.getCurrentPosition();}
    public double IntakeSlidePos(){return IntakeSlideMotor.getCurrentPosition();}
    public double getIntakeSlideVoltage (){return LiftMotor.getCurrent(CurrentUnit.AMPS);}

    public double intakeSlideError(){
        return intakeSlidePID.returnError();
    }
    public double liftError(){
        return liftPID.returnError();
    }

    public boolean turretTargetReached(){
        if (turretPosition < (turretTarget + turretthresholdDistance) && turretPosition > (turretTarget-turretthresholdDistance)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean turretTargetReachedNewThreshold(){
        if (turretPosition < (turretTarget + turretthresholdDistanceTwo) && turretPosition > (turretTarget-turretthresholdDistanceTwo)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean liftTargetReached(){
        if (liftPosition > (liftTarget - liftthresholdDistance) && liftPosition < (liftTarget+liftthresholdDistance)){ //liftthresholdDistance
            return true;
        }
        else{
            return false;
        }
    }

    public boolean intakeSlideTargetReached(){
        if (IntakeSlidePos() > (intakeSlideTarget - intakeSlidethresholdDistance) && IntakeSlidePos() < (intakeSlideTarget + intakeSlidethresholdDistance)){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean intakeSlideTargetReachedSmallerThreshold(){
        if (IntakeSlidePos() > (intakeSlideTarget - intakeSlidethresholdDistanceNewThreshold) && IntakeSlidePos() < (intakeSlideTarget + intakeSlidethresholdDistanceNewThreshold)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean intakeClawTouchPressed() {
        return IntakeClawTouch.getState();
    }
  /*
    public boolean outtakeClawTouchPressed() {
        return !OuttakeClawTouch.getState();
    }
  */
    // instead of using PID class uses the internal run to position on the motor

    public double getIntakeArmPos(){ // does work just needs to plugged in correctly
        double position = IntakeArmPosition.getVoltage() / 3.3 * 360;
        return position;
    }

    public double getIntakeLiftPos(){ // does work just needs to plugged in correctly
        double position = IntakeLiftPosition.getVoltage() / 3.3 * 360;
        return position;
    }

    // servo void functions
    public void ConeArmReady(){ConeArmServo.setPosition(ConeArmReadyPos);}
    public void ConeArmAboveCone(){ConeArmServo.setPosition(ConeArmAboveConePos);}
    public void ConeArmDownOnCone(){ConeArmServo.setPosition(ConeArmDownOnConePos);}
    public void IntakeClipOpen(){IntakeClipServo.setPosition(IntakeClipOpen);}
    public void IntakeClipHold(){IntakeClipServo.setPosition(IntakeClipHold);}

    public void IntakeClawOpen(){
        IntakeClawServo.setPosition(IntakeClawOpenPos);
    }
    public void IntakeClawClose(){
        IntakeClawServo.setPosition(IntakeClawClosedPos);
    }
    public void IntakeClawOpenHard(){
        IntakeClawServo.setPosition(IntakeClawOpenHardPos);
    }
    public double getIntakeClawPosition(){return IntakeClawServo.getPosition();}

    public void IntakeArmReady(){IntakeArmServo.setPosition(IntakeArmReadyPos);}
    public void IntakeArmTransfer(){
        IntakeArmServo.setPosition(IntakeArmTransferPos);
    }
    public void IntakeArmConeHoldForTransfer(){IntakeArmServo.setPosition(IntakeArmCOneHoldForTransferPos);}

    public void IntakeLiftReady(){IntakeSlideServo.setPosition(IntakeLiftReadyPos);}
    public void IntakeLiftTransfer(){IntakeSlideServo.setPosition(IntakeLiftTransferPos);}

    public void IntakeLift5(){IntakeSlideServo.setPosition(IntakeHeight5);}
    public void IntakeLift4(){IntakeSlideServo.setPosition(IntakeHeight4);}
    public void IntakeLift3(){IntakeSlideServo.setPosition(IntakeHeight3);}
    public void IntakeLift2(){IntakeSlideServo.setPosition(IntakeHeight2);}
    public void IntakeLift1(){IntakeSlideServo.setPosition(IntakeHeight1);}

    public void OuttakeClawOpen(){
        OuttakeClawServo.setPosition(OuttakeClawOpenPos);
    }
    public void OuttakeClawClose() {OuttakeClawServo.setPosition(OuttakeClawClosedPos);;}
    public void OuttakeClawOpenHard() {
        OuttakeClawServo.setPosition(OuttakeClawOpenHardPos);
    }

    public void OuttakeArmReady(){OuttakeArmServo.setPosition(OuttakeArmReadyPos);}
    public void OuttakeArmUpright(){OuttakeArmServo.setPosition(OuttakeArmUprightPos);}
    public void OuttakeArmDeposit(){OuttakeArmServo.setPosition(OuttakeArmDepositPos);}
    public void OuttakeArmPickup(){OuttakeArmServo.setPosition(OuttakeArmPickupPos);}
    public void OuttakeArmScore(){OuttakeArmServo.setPosition(OuttakeArmScorePos);}
    public void OuttakeArmTiltUpSlightly(){OuttakeArmServo.setPosition(OuttakeArmSlightlyTiltedUpPos);}
    public void OuttakeArmScoreAuto(){OuttakeArmServo.setPosition(OuttakeArmScoreAutoPos);}
   // public void GetOuttakeArmPosition(){OuttakeArmServo.getPosition();}

    public void BraceReady(){OuttakeBraceServo.setPosition(BraceReadyPos);}
    public void BraceActive(){OuttakeBraceServo.setPosition(BraceActivePos);}
    public void BraceTucked(){OuttakeBraceServo.setPosition(BraceTuckedPos);}
    public void BraceActiveAuto(){OuttakeBraceServo.setPosition(BraceActivePosAuto);}
    public void BracePickupCones(){OuttakeBraceServo.setPosition(BraceFlipConePos);}

    public void OuttakeSlideReady(){OuttakeSlideServo.setPosition(OuttakeSlideReadyPos);}
    public void OuttakeSlideScore(){OuttakeSlideServo.setPosition(OuttakeSlideScorePos);}
    public void OuttakeSlideScoreDrop(){OuttakeSlideServo.setPosition(OuttakeSlideScoreDropPos);}
    public void OuttakeSlideGround(){OuttakeSlideServo.setPosition(OuttakeSlideGroundPos);}
    public void OuttakeSlidePickupCones(){OuttakeSlideServo.setPosition(OuttakeSlideGroundPos);}
    public void OuttakeSlideAboveCones(){OuttakeSlideServo.setPosition(OuttakeSlideAboveConePos);}
    public void GetOuttakeSlidePosition(){OuttakeSlideServo.getPosition();}


    public double degreestoTicks(int degrees){
        return degrees * 11;
    }
    public double tickstoDegrees(int ticks){
        return ticks / 11;
    }

    public double returnPIDLiftOutput(){return liftPID.returnOutput();}
    public double returnPIDIntakeSlideOutput(){
        return intakeSlidePID.returnOutput();
    }
    public double returnPIDTurretOutput(){
        return turretPID.returnOutput();
    }

    // instead of using PID class uses the internal run to position on the motor
    public void liftToInternalPID(int rotations, double maxSpeed){
        liftTarget = rotations;
        //telemetry.addData("lifttarget", liftTarget);
        LiftMotor.setTargetPosition(liftTarget);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(maxSpeed);
    }
    public void turretSpinInternalPID(int rotations, double maxSpeed){
        turretTarget = (int)Math.round(degreestoTicks(rotations)); // rounds whatever the double is to a whole number to make it an int
        TurretMotor.setTargetPosition((int)Math.round(turretTarget));
        TurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurretMotor.setPower(maxSpeed);
    }
    public void IntakeSlideInternalPID(int rotations, double maxSpeed){
        intakeSlideTarget = rotations; // variable is public to this class?
        IntakeSlideMotor.setTargetPosition(intakeSlideTarget);
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeSlideMotor.setPower(maxSpeed);
    }
}
