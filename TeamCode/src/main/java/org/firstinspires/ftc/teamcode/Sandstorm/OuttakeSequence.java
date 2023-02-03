package org.firstinspires.ftc.teamcode.Sandstorm;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dune.DuneDrive;
import org.firstinspires.ftc.teamcode.Dune.TurretLift;

public class OuttakeSequence {

    // might have to have a function to ensure everything else goes to idle when one thing is happening

    final double IntakeSlideOutTicks = 500;

    // creating instances of each class
    ElapsedTime GlobalTimer;
    double OuttakeTimer;
    double ConeDepositTimer;
    double FlipConeTimer;
    double OuttakePickupTimer;
    double IntakeOutTimer;

    boolean DepositReady;
    boolean FlipConeReady;
    boolean OuttakePickupReady;

    boolean DepositIdle;
    boolean FlipConeIdle;
    boolean OuttakePickupIdle;

    boolean IntakeReady; // might not need this - it just goes true when the intake is ready

    // these variables are for the target positions for the lift and stuff, they may change throughout the program sequence
    int liftTargetPosition;
    int turretTargetPosition;
    int intakeSlideTargetPosition;

    DriveBase drivebase = new DriveBase();
    Inputs inputs = new Inputs();
    Outtake outtake = new Outtake();

    enum OuttakeState { // main outtake state - other fsm's will do smaller actions
        READY,
        PICKUP_EXTENDED,
        INTAKE,
        LIFT_CONE,
        CLAW_GRIP_TRANSFER,
        OUTTAKE_DEPOSIT,
        RETURN
    }
    enum ConeDepositState {
        READY,
        CONE_DROP,
        BRACE_RETRACT,
        ARM_RESET,
        IDLE
    }
    enum FlipConesState {
        OUTTAKE_FLIP_CONE_EXECUTE, // should be able to transition between up and down on cone
        IDLE,
        RETURN
    }
    enum OuttakePickupState {
        READY,
        READY_TO_OUTTAKE,
        GRAB_OUTTAKE,
        HEIGHT_OUTTAKE,
        RETURN,
        IDLE
    }
    enum IntakeOut {
        READY,
        INTAKE_INITIAL_LIFT,
        INTAKE_SHOOT_OUT,
        INTAKE_TO_TRANSFER,
        RETURN,
        IDLE
    }

    // instance of enum
    OuttakeState outtakeState = OuttakeState.READY;
    ConeDepositState coneDepositState = ConeDepositState.IDLE;
    FlipConesState flipConesState = FlipConesState.IDLE;
    OuttakePickupState outtakePickupState = OuttakePickupState.IDLE;
    IntakeOut intakeout = IntakeOut.IDLE;

    // runs on init (not setup function)
    public void OuttakeHardware(){
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap);
    }

    // this functions runs in the setup function of the opmode (runs once instantly)
    public void OuttakeSetup(){
        // seperate timers for each state machine so no interfering (asynchronous)
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();

        OuttakeTimer = 0; // this variable is just a normal double however it is stored as the global timer
        IntakeOutTimer = 0;
        FlipConeTimer = 0;
        ConeDepositTimer = 0;

        // hardware setup
        drivebase.motorsSetup();
        outtake.hardwareSetup();
        inputs.inputsSetup(); // this needs to be chnaged - changes toggle variables and stuff to false

        // sets the first case for the fsm to be in
        outtakeState = OuttakeState.READY;
        coneDepositState = ConeDepositState.IDLE;
        flipConesState = FlipConesState.IDLE;
        outtakePickupState = OuttakePickupState.IDLE;
        intakeout = IntakeOut.READY;

        turretTargetPosition = 0;
        liftTargetPosition = 0;
        intakeSlideTargetPosition = 0;
    }

    public void IntakeShootOut() {
        switch (intakeout) {
            case READY:
                outtake.IntakeArmReady();
                outtake.IntakeLiftReady();
                if (gamepad1.a || inputs.IntakeToggleOutState == 2) { // make sure this function isn't called in a case that it start unintentially
                    intakeout = IntakeOut.INTAKE_INITIAL_LIFT; // this will start the timer and the inputs function will also go to 2 at the same time
                    IntakeOutTimer = GlobalTimer.milliseconds();
                }
                break;
            case INTAKE_INITIAL_LIFT:
                outtake.IntakeLiftTransfer();
                if (GlobalTimer.milliseconds() - IntakeOutTimer > 200) {
                    if (inputs.IntakeToggleOutState == 2){
                        intakeout = IntakeOut.INTAKE_SHOOT_OUT;
                    }
                }
                break;
            case INTAKE_SHOOT_OUT:
                outtake.IntakeSlideTo(IntakeSlideOutTicks, outtake.IntakeSlidePos(), 1);
                if (outtake.IntakeSlidePos() > 100) {
                    outtake.IntakeClawOpenHard();
                    outtake.IntakeLiftReady();
                }
                if (inputs.IntakeToggleOutState == 3){
                    outtake.IntakeClawClose();
                    IntakeOutTimer = GlobalTimer.milliseconds();
                    intakeout = IntakeOut.INTAKE_TO_TRANSFER;
                }
                    break;
            case INTAKE_TO_TRANSFER:
                outtake.IntakeClawClose();
                if (GlobalTimer.milliseconds() - IntakeOutTimer > 300){ // wait for the claw to grav
                    if (inputs.IntakeToggleOutState == 3){ // 4/3 is zero remainder 3, therefore it goes from 0 to 3
                        intakeExtendedToTransfer();
                        outtakeState = OuttakeState.CLAW_GRIP_TRANSFER; // main state machine takes it from here
                        intakeout = IntakeOut.IDLE; // main state machine must set everything to ready in a function
                    }
                }
                break;
            case IDLE:
                // nothing happens - when other processes are in use put to idle
                break;

        }
        inputs.IntakeToggleOut(gamepad1.left_bumper); // ONLY RUN THIS FUNCTION IN PLACES WHERE YOU WANT BUTTON SPAM, DO NOT RUN THIS FUNCTION OTHERWISE INTAKE THING WILL CHANGE STATES WITH BUTTON SPAMS
        // PUT TO IDLE WHEN SWITCHING STATES so the code runs once and this function doesn't re-run with a different intake toggleoutstate
        // put the inputs Toggle out state back to 0 in a function
        if (inputs.IntakeToggleOutState == 0 && intakeout != IntakeOut.IDLE){ // can break if you spam button fix
            intakeout = IntakeOut.READY;
        }
        else if (inputs.IntakeToggleOutState == 1 && intakeout != IntakeOut.IDLE){
            intakeout = IntakeOut.INTAKE_INITIAL_LIFT;
        }
        // 4/3 is zero remainder 3, therefore it goes from 0 to 3
    }


    public void ConeDepositSequence(){

        switch (coneDepositState) {
            case READY: // not being used at the moment

                break;
            case CONE_DROP:
                coneDepositState = ConeDepositState.CONE_DROP; // basically when it goes into cone drop it instantly goes to next state to start the timer
                ConeDepositTimer = GlobalTimer.milliseconds(); // reset timer
                break;

            case BRACE_RETRACT:
                outtake.OuttakeSlideScore();
                if (GlobalTimer.milliseconds() - ConeDepositTimer > 200){
                    outtake.BraceReady(); // might need a new position for this
                    outtake.OuttakeArmScore();
                    if (GlobalTimer.milliseconds() - ConeDepositTimer > 250){
                        outtake.OuttakeClawOpenHard();
                        if (GlobalTimer.milliseconds() - ConeDepositTimer > 300){
                            coneDepositState = ConeDepositState.ARM_RESET;
                        }
                    }
                }
                break;
            case ARM_RESET:
                outtake.BraceReady();
                outtake.OuttakeClawOpen();
                outtake.OuttakeArmReady();
                outtake.OuttakeSlideReady();
                break;

            case IDLE:

                break;
        }
    }
    public void FlipConeSequence(){
        switch (flipConesState) {
            case OUTTAKE_FLIP_CONE_EXECUTE: // this case is basically when we want the command in toggle, otherwise we set it to idle case
                inputs.flipConeToggleMode(gamepad2.left_stick_button);
                if (inputs.FlipConeToggleMode){
                    inputs.coneFlipOuttakeDownToggle(gamepad2.left_bumper, gamepad2.right_bumper);
                    // use if statements for this, state machine not needed
                    if (inputs.AboveConeHeight || inputs.FlipConeHeightState == 0){ // this is an if statement rather than if else, because if its pressed we want it to overide everything else
                        outtake.OuttakeSlideAboveCones();
                        outtake.BracePickupCones();
                        outtake.OuttakeArmPickup();
                        outtake.OuttakeClawOpenHard();
                    } else if (inputs.FlipConeHeightState == 1){
                        outtake.OuttakeSlidePickupCones();
                        outtake.BracePickupCones();
                        outtake.OuttakeArmPickup();
                        outtake.OuttakeClawOpenHard();
                        FlipConeTimer = GlobalTimer.milliseconds(); // being reset the whole time until goes to next stage?
                    } else if (inputs.FlipConeHeightState == 2){
                        if (GlobalTimer.milliseconds() - FlipConeTimer > 300) {
                            outtake.BraceTucked();
                            if (GlobalTimer.milliseconds() - FlipConeTimer > 400) {
                                outtake.OuttakeSlidePickupCones();
                                outtake.OuttakeArmPickup();
                                outtake.OuttakeClawOpenHard();
                                flipConesState = FlipConesState.IDLE;
                                outtakePickupState = OuttakePickupState.GRAB_OUTTAKE; // this puts into a state of waiting idly for me to press the pickup button
                            }
                        }
                        else{
                            outtake.OuttakeArmTiltUpSlightly();
                            outtake.OuttakeSlideAboveCones();
                        }
                    }
                }
                break;
            case IDLE:
                inputs.FlipConeToggleMode = false; // ensures when we return out of idle state brace and arm wont immediately try and flip cones
                // does nothing
                break;
        }

    }
    public void OuttakePickupSequence(){
        switch (outtakePickupState) {
            case READY: // not being used at the moment
                outtakePickupState = OuttakePickupState.READY_TO_OUTTAKE; // basically when it goes into cone drop it instantly goes to next state to start the timer
                OuttakePickupTimer = GlobalTimer.milliseconds(); // reset timer
                break;
            case READY_TO_OUTTAKE:
                outtake.OuttakeArmPickup();
                if (GlobalTimer.milliseconds() - OuttakePickupTimer > 450){
                    outtake.BraceTucked();
                    outtake.OuttakeClawOpenHard();
                    if (GlobalTimer.milliseconds() - OuttakePickupTimer > 600){
                        outtake.OuttakeSlidePickupCones();
                        outtakePickupState = OuttakePickupState.GRAB_OUTTAKE;
                    }
                }
                break;
            case GRAB_OUTTAKE:
                if (gamepad1.left_bumper){
                    outtake.OuttakeClawClose();
                    OuttakePickupTimer = GlobalTimer.milliseconds(); // reset timer
                }
            case HEIGHT_OUTTAKE:
                if (GlobalTimer.milliseconds() - OuttakePickupTimer > 200){ // wait 200 ms before the claw closes
                    outtake.OuttakeSlideScore();
                    outtake.liftTo(liftTargetPosition, outtake.liftPos(),1);
                    if (GlobalTimer.milliseconds() - OuttakePickupTimer > 250){ // just wait a teensy weensy bit before arm tilts
                        outtake.OuttakeArmTiltUpSlightly();
                        outtake.BraceActive();
                        outtakePickupState = OuttakePickupState.IDLE;
                        outtakeState = OuttakeState.OUTTAKE_DEPOSIT;
                    }
                }
                break;
            case IDLE:
                // yeah sets it to idle so no multiple commands can happen at once
                break;
            }
        }

    public void intakeExtendedToTransfer(){
        if (outtake.IntakeSlidePos() > 100){
            outtake.IntakeLiftTransfer();
            outtake.IntakeArmTransfer();
            outtake.IntakeSlideTo(0, outtake.IntakeSlidePos(), 1);
        } else {
            IntakeReady = true;
        }
    }
}
