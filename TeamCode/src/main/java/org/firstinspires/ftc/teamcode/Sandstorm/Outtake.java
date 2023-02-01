package org.firstinspires.ftc.teamcode.Sandstorm;

import com.acmerobotics.dashboard.config.Config;
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
    private DigitalChannel IntakeClawTouch;
    private DigitalChannel OuttakeClawTouch;
    AnalogInput IntakeArmPosition;

    //Servo Positions for outtake
    public static double OuttakeClawOpenPos = 0.52, OuttakeClawClosedPos = 0.405, OuttakeClawOpenHardPos = 0.61;
    public static double OuttakeArmReadyPos = 0, OuttakeArmDepositPos = 0, OuttakeArmPickupPos = 0;
    public static double BraceReadyPos = 0, BraceActivePos = 0, BraceTuckedPos = 0, BraceFlipConePos = 0;
    public static double OuttakeSlideReadyPos = 0, OuttakeSlideScorePos = 0, OuttakeSlideScoreDropPos = 0, OuttakeSlideGroundPos = 0, OuttakeSlideConeFlipPos = 0;

    //Servo Positions for Intake
    public static double IntakeClawOpenPos = 0.52, IntakeClawClosedPos = 0.405, IntakeClawOpenHardPos = 0.61;
    public static double IntakeArmReadyPos = 0, IntakeArmDepositPos = 0, IntakeArmPickupPos = 0;
    public static double IntakeSlideReadyPos = 0, IntakeSlideTransferPos = 0;

    //Servo Positions for Stack Height
    public static double IntakeHeight5 = 0, IntakeHeight4 = 0, IntakeHeight3 = 0, IntakeHeight2 = 0, IntakeHeight1 = 0;

    //editable dashboard variables must be public static - PID values for turret and lift that can be tuned
    public static double TurretKp = 0.005, TurretKi = 0.001, TurretKd = 0.05, TurretIntegralSumLimit = 1, TurretFeedforward = 0.3;
    public static double LiftKp = 0.005, LiftKi = 0.01, LiftKd = 0.05, LiftIntegralSumLimit = 10, LiftKf = 0;
    public static double intakeSlideKp = 0.005, intakeSlideKi = 0.01, intakeSlideKd = 0.05, intakeSlideIntegralSumLimit = 10, intakeSlideKf = 0;

    // New instance of PID class with editable variables
    PID turretPID = new PID(TurretKp,TurretKi,TurretKd,TurretIntegralSumLimit,TurretFeedforward);
    PID liftPID = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    PID intakeSlidePID = new PID(intakeSlideKp,intakeSlideKi,intakeSlideKd,intakeSlideIntegralSumLimit,intakeSlideKf);

    // final variables
    final double turretthresholdDistance = degreestoTicks(8); // should make the threshold less
    final double turretthresholdDistanceTwo = degreestoTicks(2);

    final double liftthresholdDistance = 60;
    final double intakeSlidethresholdDistance = 60;

    int turretTarget;
    int liftTarget;
    int intakeSlideTarget;


    public void Outtake_init(HardwareMap hwMap) {
        TurretMotor = hwMap.get(DcMotorEx.class, "TurretMotor");
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");
        IntakeSlideMotor = hwMap.get(DcMotorEx.class, "IntakeSlideMotor");

        OuttakeClawServo = hwMap.get(ServoImplEx.class, "OutClawS");
        OuttakeArmServo = hwMap.get(ServoImplEx.class, "OutArmS");
        OuttakeSlideServo = hwMap.get(ServoImplEx.class, "OutSlideS");
        OuttakeBraceServo = hwMap.get(ServoImplEx.class, "OutBraceS");
        IntakeArmServo = hwMap.get(ServoImplEx.class, "InArmS");
        IntakeSlideServo = hwMap.get(ServoImplEx.class, "InSlideS");
        IntakeClawServo = hwMap.get(ServoImplEx.class, "InClawS");

        IntakeClawTouch = hwMap.get(DigitalChannel.class, "IntakeTouch");
        OuttakeClawTouch = hwMap.get(DigitalChannel.class, "OuttakeTouch");

        IntakeArmPosition = hwMap.get(AnalogInput.class, "linkageEncoder");
    }


    public void hardwareSetup(){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeClawTouch.setMode(DigitalChannel.Mode.INPUT);
        OuttakeClawTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void encodersReset(){
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftMotorRawControl(double manualcontrollift){
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setPower(manualcontrollift * -0.5);
    }

    public void turretMotorRawControl(double manualcontrolturret){
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretMotor.setPower(manualcontrolturret * -0.3);
    }
    public void intakeSlideMotorRawControl(double manualcontrolintakeslide){ // shouldn't have to do this - will be too slow
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretMotor.setPower(manualcontrolintakeslide * -0.3);
    }

    public void liftTo(int rotations, double motorPosition, double maxSpeed){
        liftTarget = rotations;
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        LiftMotor.setPower(output);
    }
    public void turretSpin(double targetRotations, double motorPosition, double maxSpeed){
        double output = turretPID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        TurretMotor.setPower(output);
    }
    public void IntakeSlideTo(double targetRotations, double motorPosition, double maxSpeed){
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

    public boolean turretTargetReached(){
        if (turretPos() < (turretTarget + turretthresholdDistance) && turretPos() > (turretTarget-turretthresholdDistance)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean turretTargetReachedNewThreshold(){
        if (turretPos() < (turretTarget + turretthresholdDistanceTwo) && turretPos() > (turretTarget-turretthresholdDistanceTwo)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean liftTargetReached(){
        if (liftPos() < (liftTarget + liftthresholdDistance) && liftPos() > (liftTarget-liftthresholdDistance)){ //liftthresholdDistance
            return true;
        }
        else{
            return false;
        }
    }

    public boolean intakeSlideTargetReached(){
        if (IntakeSlidePos() < (intakeSlideTarget + intakeSlidethresholdDistance) && IntakeSlidePos() > (intakeSlideTarget-intakeSlidethresholdDistance)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean intakeClawTouchPressed() {
        return !IntakeClawTouch.getState();
    }
    public boolean outtakeClawTouchPressed() {
        return !OuttakeClawTouch.getState();
    }
    // instead of using PID class uses the internal run to position on the motor

    public double getIntakeArmPos(){ // doesn't work
        double position = IntakeArmPosition.getVoltage() / 3.3 * 360;
        return position;
    }

    // servo void functions
    public void IntakeClawOpen(){
        IntakeClawServo.setPosition(IntakeClawOpenPos);
    }
    public void IntakeClawClose(){
        IntakeClawServo.setPosition(IntakeClawClosedPos);
    }
    public void IntakeClawOpenHard(){
        IntakeClawServo.setPosition(IntakeClawOpenHardPos);
    }

    public void IntakeArmReady(){IntakeClawServo.setPosition(IntakeClawOpenPos);}
    public void IntakeArmTransfer(){
        IntakeClawServo.setPosition(IntakeClawClosedPos);
    }

    public void IntakeSlideReady(){IntakeClawServo.setPosition(IntakeClawOpenHardPos);}
    public void IntakeSlideTransfer(){IntakeClawServo.setPosition(IntakeClawOpenHardPos);}

    public void IntakeSlide5(){IntakeClawServo.setPosition(IntakeHeight5);}
    public void IntakeSlide4(){IntakeClawServo.setPosition(IntakeHeight4);}
    public void IntakeSlide3(){IntakeClawServo.setPosition(IntakeHeight3);}
    public void IntakeSlide2(){IntakeClawServo.setPosition(IntakeHeight2);}
    public void IntakeSlide1(){IntakeClawServo.setPosition(IntakeHeight1);}

    public void OuttakeClawOpen(){
        OuttakeClawServo.setPosition(OuttakeClawOpenPos);
    }
    public void OuttakeClawClose() {OuttakeClawServo.setPosition(OuttakeClawClosedPos);;}
    public void OuttakeClawOpenHard() {
        OuttakeClawServo.setPosition(OuttakeClawOpenHardPos);
    }

    public void OuttakeArmReady(){OuttakeArmServo.setPosition(OuttakeArmReadyPos);}
    public void OuttakeArmDeposit(){OuttakeArmServo.setPosition(OuttakeArmDepositPos);}
    public void OuttakeArmPickup(){OuttakeArmServo.setPosition(OuttakeArmPickupPos);}

    public void BraceReady(){OuttakeBraceServo.setPosition(BraceReadyPos);}
    public void BraceActive(){OuttakeBraceServo.setPosition(BraceActivePos);}
    public void BraceTucked(){OuttakeBraceServo.setPosition(BraceTuckedPos);}
    public void BracePickupCones(){OuttakeBraceServo.setPosition(BraceFlipConePos);}

    public void OuttakeSlideReady(){OuttakeSlideServo.setPosition(OuttakeSlideReadyPos);}
    public void OuttakeSlideScore(){OuttakeSlideServo.setPosition(OuttakeSlideScorePos);}
    public void OuttakeSlideScoreDrop(){OuttakeSlideServo.setPosition(OuttakeSlideScoreDropPos);}
    public void OuttakeSlideGround(){OuttakeSlideServo.setPosition(OuttakeSlideReadyPos);}
    public void OuttakeSlidePickupCones(){OuttakeSlideServo.setPosition(OuttakeSlideConeFlipPos);}


    public double degreestoTicks(int degrees){
        return degrees * 7.6;
    }
    public double tickstoDegrees(int ticks){
        return ticks / 7.6;
    }

    public double returnPIDLiftOutput(){
        return liftPID.returnOutput();
    }
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
        turretTarget = rotations; // variable is public to this class?
        TurretMotor.setTargetPosition(turretTarget);
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
