package org.firstinspires.ftc.teamcode.Sandstorm;

// Old imports, some not needed
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "StormDrive")
public class StormDrive extends LinearOpMode {
    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    final double IntakeSlideOutTicks = -400;

    final int LiftHighPosition = 800;
    final int LiftMidPosition = 400;
    final int LiftLowPosition = 0;

    final int TurretLeftPosition = -30;
    final int TurretRightposition = 30;

    // creating instances of each class
    ElapsedTime GlobalTimer;
    double OuttakeTimer;
    double ConeDepositTimer;
    double FlipConeTimer;
    double OuttakePickupTimer;
    double IntakeOutTimer;

    boolean IntakeReady; // might not need this - it just goes true when the intake is ready

    // these variables are for the target positions for the lift and stuff, they may change throughout the program sequence
    int liftTargetPosition;
    int turretTargetPosition;
    int intakeSlideTargetPosition;

    DriveBase drivebase = new DriveBase();
    Inputs inputs = new Inputs();
    Outtake outtake = new Outtake();

    enum OuttakeState { // main outtake state - other fsm's will do smaller actions
        READY, // this is the state that everything will return to when b is pressed.
        PICKUP_EXTENDED, // add the functions for the other state machines (intake out etc) in these cases later
        INTAKE,
        LIFT_CONE,
        CLAW_GRIP_TRANSFER,
        HEIGHT_CHANGE_OUTTAKE_DEPOSIT,
        DROP,
        SUBSYSTEMS_SET_RETURN,
        RETURN,
        IDLE
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
        RETURN_SET_TIMER,
        RETURN,
        IDLE
    }

    // instance of enum
    OuttakeState outtakeState; // do i have to set a state here to idle or something?
    ConeDepositState coneDepositState;
    FlipConesState flipConesState;
    OuttakePickupState outtakePickupState;
    IntakeOut intakeout;

    // runs on init (not setup function)
    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
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
        inputs.IntakeToggleOutState = 0; // for intake shoot out function

        // sets the first case for the fsm to be in
        outtakeState = OuttakeState.SUBSYSTEMS_SET_RETURN; // tghis is return so all the other states start in the right thing
        coneDepositState = ConeDepositState.IDLE;
        flipConesState = FlipConesState.IDLE;
        outtakePickupState = OuttakePickupState.IDLE;
        intakeout = IntakeOut.READY;

        turretTargetPosition = 0;
        liftTargetPosition = 50; // if no height is specified, then the lift needs to go out a bit for the transfer
        intakeSlideTargetPosition = 0;

    }

    @Override
    public void runOpMode() {

        // this is basically init, all setup, hardware classes etc get initialized here
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap);

        // waits for user to press start on driverhub
        waitForStart();
        if (opModeIsActive()) {
            // runs setup function before main loop
            Setup();

            while (opModeIsActive()) {

                drivebase.Drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                //drivebase.motorDirectionTest(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x,gamepad1.right_stick_y);
                drivebase.PowerToggle(gamepad1.x);
                //inputs.gamepadRumbleTimer();
                telemetry.addData("LiftMotorPosition", outtake.liftPos());
                telemetry.addData("turretPosition", outtake.tickstoDegrees((int)Math.round(outtake.turretPos()))); // might be in the wrong degrees/other
                telemetry.addData("turret raw position", outtake.turretPos());
                telemetry.addData("lift motor current draw", outtake.getLiftVoltage());
                telemetry.addData("intakeSlideMotor", outtake.IntakeSlidePos());

               // outtakeSequence(); // if gamepad things don't work here then need to pass them in as parameters of this function

                telemetry.addData("Main Outtake State", outtakeState);
                telemetry.addData("Intake Out State", outtakeState);
                telemetry.addData("Cone Deposit State", coneDepositState);
                telemetry.addData("Flip Cone State", flipConesState);
                telemetry.addData("Outtake Pickup state", outtakePickupState);

                telemetry.addData("powerbase", drivebase.PowerBase);
                telemetry.update();
            }
        }
    }
    public void outtakeSequence(){ // make sure to go back and add when each statemachine command should be run
        switch (outtakeState) {
            case READY:
                outtake.IntakeClawOpen();
                outtake.IntakeArmReady();
                outtake.IntakeLiftReady();
                liftTargetPosition = 0; // if lift target position is zero, then the transfer won't happen

                resetAllMotors(); // might break something

                outtake.OuttakeSlideReady();
                outtake.OuttakeClawOpen();
                outtake.OuttakeArmReady();
                outtake.BraceReady();

                if (gamepad2.right_bumper || gamepad1.right_bumper){
                    outtakeState = OuttakeState.INTAKE;
                }
                IntakeShootOut();
                OuttakePickupSequence();
                FlipConeSequence();

                break;
            case INTAKE:
                drivebase.intakeSpin(1); // spin the intake
                resetAllMotors();
                // no need to put ready stuff on because there will be nothing conflicting with it
                if (outtake.intakeClawTouchPressed() || gamepad2.right_bumper){
                    outtakeState = OuttakeState.LIFT_CONE;
                    OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                    outtake.IntakeClawClose();
                }
                break;
            case LIFT_CONE:
                outtake.IntakeClawClose();
                resetAllMotors();
                if (GlobalTimer.milliseconds() - OuttakeTimer > 200){
                    outtake.IntakeLiftTransfer();
                    if (GlobalTimer.milliseconds() - OuttakeTimer > 400){
                        outtake.IntakeArmTransfer();
                        if ((GlobalTimer.milliseconds() - OuttakeTimer > 650) ||outtake.outtakeClawTouchPressed()){
                            outtakeState = OuttakeState.CLAW_GRIP_TRANSFER;
                            OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                        }
                    }
                }
                break;
            case CLAW_GRIP_TRANSFER:
                resetAllMotors();
                if (outtake.liftTargetReached() && (liftTargetPosition != 0)){ // make sure height is selected before transferring
                    outtake.OuttakeClawClose();
                    if (GlobalTimer.milliseconds() - OuttakeTimer > 150){
                        outtake.IntakeClawOpen();
                        if (GlobalTimer.milliseconds() - OuttakeTimer > 200){
                            outtake.liftTo(50, outtake.liftPos(), 1);
                            outtake.OuttakeArmScore();
                            if (GlobalTimer.milliseconds()-OuttakeTimer > 230){
                                outtake.BraceActive(); // this should happen at the same time as the outtake arm is going out so that its always parrallel to the ground
                                outtake.IntakeArmReady();
                                if (GlobalTimer.milliseconds()-OuttakeTimer > 300){
                                    outtake.IntakeLiftReady();
                                }
                            }
                        }
                    }
                }
                break;
            case HEIGHT_CHANGE_OUTTAKE_DEPOSIT:
                outtake.turretSpin(turretTargetPosition,outtake.turretPos(),1);
                outtake.liftTo(liftTargetPosition, outtake.liftPos(),1);
                if (gamepad1.right_bumper){
                    outtakeState = OuttakeState.DROP;
                    coneDepositState = ConeDepositState.CONE_DROP; // should run the whole drop sequence here
                    OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                }
                break;
            case DROP:
                ConeDepositSequence(); // this runs constantly to make it happen
                // run the intake shoot out
                IntakeShootOut();
                if (coneDepositState == ConeDepositState.IDLE){ // if the whole sequence has finished
                    outtakeState = OuttakeState.SUBSYSTEMS_SET_RETURN;
                    OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                }
                break;
            case SUBSYSTEMS_SET_RETURN: // set the cases for return
                IntakeShootOut();
                intakeout = IntakeOut.RETURN_SET_TIMER;// sets the intake to return state - will return no matter its state
                outtakeState = OuttakeState.RETURN;
                OuttakeTimer = GlobalTimer.milliseconds(); // reset timer

                // makes all the subsystems states go back to default
                inputs.FlipConeHeightState = 0; // makes the flip cone state go back to zero so it goes to the first state in the modulo rather than a random stage of picking up cones
                flipConesState = FlipConesState.OUTTAKE_FLIP_CONE_EXECUTE;
                // puts it back to ready so we can use it in ready state of the main state machine
                outtakePickupState = OuttakePickupState.READY;

                break;
            case RETURN:
                // make sure to go to arm reset state for all subsystems
                outtake.liftTo(0, outtake.liftPos(),1);
                outtake.turretSpin(0,outtake.turretPos(),1);
                IntakeShootOut(); // runs the return case for this
                if (IntakeReady) { // outtake lift target reached might be needed but the whole point is so that we can use the intake while the outtake is resetting
                    outtakeState = OuttakeState.READY; // need the arm to reset as well

                }
                outtake.OuttakeSlideReady();
                outtake.OuttakeClawOpen();
                if (GlobalTimer.milliseconds() - OuttakeTimer > 200){ // this might not return if b is pressed when its picking up cones or something - need to test
                    outtake.BraceReady();
                    if (GlobalTimer.milliseconds() - OuttakeTimer > 300){
                        outtake.OuttakeArmReady();
                    }
                }
                break;
            case IDLE:
                // puts the state into idle when other actions are happening
                break;
        }
        if ((gamepad2.b && outtakeState != OuttakeState.READY) || (gamepad1.b && outtakeState != OuttakeState.READY)){
            outtakeState = OuttakeState.SUBSYSTEMS_SET_RETURN; // if b is pressed at any state then return to ready
        }

        turretPositionChange();
        liftPositionChange();
    }
    public void IntakeShootOut() {
        switch (intakeout) {
            case READY:
                outtake.IntakeArmReady();
                outtake.IntakeLiftReady();
                IntakeReady = true; // use this variable somewhere else
                if (gamepad1.left_bumper || inputs.IntakeToggleOutState == 2) { // make sure this function isn't called in a case that it start unintentially
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
                drivebase.intakeSpin(0.4); // helps the slides go out
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
            case RETURN_SET_TIMER: //this runs a timer to set for the return state
                intakeout = IntakeOut.RETURN;
                IntakeOutTimer = GlobalTimer.milliseconds();
                break;
            case RETURN:
                if (outtake.IntakeSlidePos() > 100){
                    if (GlobalTimer.milliseconds() - IntakeOutTimer > 350){
                        outtake.IntakeSlideTo(0, outtake.IntakeSlidePos(),1);
                        if (outtake.intakeSlideTargetReached()){
                            outtake.IntakeLiftReady();
                            intakeout = IntakeOut.READY;
                            inputs.IntakeToggleOutState = 0;
                        }
                        else{
                            outtake.IntakeLiftTransfer(); // when slide is coming in make sure servos are up
                            outtake.IntakeArmReady();
                            drivebase.intakeSpin(-0.4); // spin in when its coming in
                        }
                    } else{
                        outtake.IntakeLiftTransfer();
                        outtake.IntakeArmReady();
                    }
                } else {
                    outtake.IntakeLiftReady();
                    outtake.IntakeArmReady();
                    intakeout = IntakeOut.READY;
                    inputs.IntakeToggleOutState = 0; // makes sure when it goes back to ready it won't autonomatically go back to previous state
                }
                break;

        }
        inputs.IntakeToggleOut(gamepad1.left_bumper); // ONLY RUN THIS FUNCTION IN PLACES WHERE YOU WANT BUTTON SPAM, DO NOT RUN THIS FUNCTION OTHERWISE INTAKE THING WILL CHANGE STATES WITH BUTTON SPAMS
        // PUT TO IDLE WHEN SWITCHING STATES so the code runs once and this function doesn't re-run with a different intake toggleoutstate
        // put the inputs Toggle out state back to 0 in a function
        if (inputs.IntakeToggleOutState == 0 && intakeout != IntakeOut.IDLE && intakeout != IntakeOut.RETURN&& intakeout != IntakeOut.RETURN_SET_TIMER){ // can break if you spam button fix
            intakeout = IntakeOut.READY; // make sure these inputs don't count when this is happening
        }
        else if (inputs.IntakeToggleOutState == 1 && intakeout != IntakeOut.IDLE && intakeout != IntakeOut.RETURN&& intakeout != IntakeOut.RETURN_SET_TIMER){
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
                outtake.OuttakeSlideScore(); // drops down on pole a bit
                if (GlobalTimer.milliseconds() - ConeDepositTimer > 200){
                    outtake.BraceReady(); // might need a new position for this
                    outtake.OuttakeArmScore();
                    if (GlobalTimer.milliseconds() - ConeDepositTimer > 250){
                        outtake.OuttakeClawOpenHard();
                        if (GlobalTimer.milliseconds() - ConeDepositTimer > 350){
                            outtake.liftTo(0, outtake.liftPos(), 1);
                            outtake.turretSpin(0, outtake.turretPos(),1);
                            if (GlobalTimer.milliseconds() - ConeDepositTimer > 450){
                                coneDepositState = ConeDepositState.ARM_RESET;
                            }
                        }
                    }
                }
                break;
            case ARM_RESET:
                outtake.liftTo(0, outtake.liftPos(), 1);
                outtake.turretSpin(0, outtake.turretPos(),1);
                outtake.BraceReady();
                outtake.OuttakeClawOpen();
                outtake.OuttakeArmReady();
                outtake.OuttakeSlideReady();
                coneDepositState = ConeDepositState.IDLE; // goes to idle to make sure it only runs once when we want it
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
    public void OuttakePickupSequence(){ // this basically uses the dual intake to pickup
        switch (outtakePickupState) {
            case READY:
                if (gamepad1.dpad_up || gamepad2.dpad_up){
                    outtakePickupState = OuttakePickupState.READY_TO_OUTTAKE;
                    OuttakePickupTimer = GlobalTimer.milliseconds(); // reset timer
                }
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
                if (gamepad1.left_bumper || gamepad2.left_bumper){
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
                        outtakeState = OuttakeState.HEIGHT_CHANGE_OUTTAKE_DEPOSIT; // this goes to the deposit stage of the main state machine
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
            drivebase.intakeSpin(-0.4);
        } else {
            IntakeReady = true;
        }
    }
    public void liftPositionChange(){ // if gamepad inputs don't work in this class then pass them through as parameters in the function
        if (gamepad1.y || gamepad2.y){
            liftTargetPosition = LiftHighPosition; // add servo change here if its different for each height
        } else if (gamepad1.x || gamepad2.x){
            liftTargetPosition = LiftMidPosition;
        } else if (gamepad1.a || gamepad2.a){
            liftTargetPosition = LiftLowPosition;
        }
    }
    public void turretPositionChange(){
        if (gamepad1.dpad_up || gamepad2.dpad_up){
            turretTargetPosition = 0;
        } else if (gamepad1.dpad_right || gamepad2.dpad_right){
            turretTargetPosition = TurretRightposition;
        } else if (gamepad1.dpad_left || gamepad2.dpad_left){
            turretTargetPosition = TurretLeftPosition;
        }
    }
    public void resetAllMotors(){
        outtake.IntakeSlideTo(0, outtake.IntakeSlidePos(),1); // might break something
        outtake.liftTo(0, outtake.liftPos(),1);
        outtake.turretSpin(0,outtake.turretPos(),1);
    }
}


