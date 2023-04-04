package org.firstinspires.ftc.teamcode.Sandstorm;

// Old imports, some not needed
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Dune.DuneDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


@TeleOp(name = "StormDrive")
public class StormDrive extends LinearOpMode {

    //Rumble
    Gamepad.RumbleEffect halfRumbleEffect;
    Gamepad.RumbleEffect endgameRumbleEffect;
    Gamepad.RumbleEffect tenRumbleEffect;
    Gamepad.RumbleEffect oneFourthRumbleEffect;
    Gamepad.RumbleEffect switchRumbleEffect;

    //Timing for rumble
    ElapsedTime runtime = new ElapsedTime();

    final double halfTime = 60.0;
    final double endgame = 90.0;
    final double tenTime = 110.0;
    final double oneFourth = 30.0;

    final double xTarget = 0;
    final double yTarget = 0;
    final double headingTarget = 0;

    final double distanceFromPointThreshold = 3; // this would have to be changed

    double xPosition;
    double yPosition;
    double headingPosition;
    double correctedHeading;

    boolean one = true;
    boolean two = false;
    boolean three = false;
    boolean four = false;

    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    final int IntakeSlideOutTicks = -700;

    final int LiftHighPosition = -752;
    final int LiftMidPosition = -414;
    final int LiftLowPosition = -40;
    final int LiftGroundPosition = -30;

    final int LiftUpperLimit = -810;

    final int TurretLeftPosition = 8;
    final int TurretRightposition = -8;

    // creating instances of each class
    ElapsedTime GlobalTimer;
    double OuttakeTimer;
    double ConeDepositTimer;
    double FlipConeArmTimer;
    double OuttakePickupTimer;
    double IntakeOutTimer;
    double GroundJunctionDepositTimer;
    double IntakeLeftSequenceTimer;


    boolean IntakeReady; // might not need this - it just goes true when the intake is ready
    boolean BeaconScore; // if this variable is true the intake will not reset but it will hold the cone
    boolean SlidesToggleUp;
    boolean SlidesToggleDown;

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
        CLAW_GRIP_TRANSFER_START,
        CLAW_GRIP_TRANSFER_END,
        HEIGHT_CHANGE_OUTTAKE_DEPOSIT,
        PICKUP_CONE_AND_BEACON,
        DROP,
        SUBSYSTEMS_SET_RETURN,
        RETURN,
        MANUAL_ENCODER_RESET,
        IDLE
    }
    enum ConeDepositState {
        READY,
        CONE_DROP,
        BRACE_RETRACT,
        ARM_RESET,
        IDLE
    }
    enum FlipConeArmState {
        OUTTAKE_FLIP_CONE_EXECUTE, // should be able to transition between up and down on cone
        IDLE,
        RETURN
    }
    enum OuttakePickupState {
        READY,
        READY_TO_OUTTAKE_START,
        READY_TO_OUTTAKE_END,
        GRAB_OUTTAKE,
        HEIGHT_OUTTAKE,
        RETURN,
        IDLE
    }
    enum GroundJunctionDeposit {
        READY,
        READY_TO_OUTTAKE_START,
        READY_TO_OUTTAKE_END,
        DROP_TO_GROUND,
        IDLE
    }
    enum IntakeLiftSequence {
        READY,
        INTAKE,
        LIFT_CONE,
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
    FlipConeArmState flipConeArmState;
    OuttakePickupState outtakePickupState;
    GroundJunctionDeposit groundJunctionsDeposit;
    IntakeOut intakeout;
    IntakeLiftSequence intakeLiftSequence;

    Pose2d resetPose = new Pose2d(0, 0, Math.toRadians(0));

    // runs on init (not setup function)
    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();

        OuttakeTimer = 0; // this variable is just a normal double however it is stored as the global timer
        IntakeOutTimer = 0;
        FlipConeArmTimer = 0;
        ConeDepositTimer = 0;
        GroundJunctionDepositTimer = 0;
        IntakeLeftSequenceTimer = 0;
        SlidesToggleUp = false;
        SlidesToggleDown = false;

        // hardware setup
        drivebase.motorsSetup();
        outtake.hardwareSetup();
        outtake.encodersReset();
        inputs.inputsSetup(); // this needs to be chnaged - changes toggle variables and stuff to false
        inputs.IntakeToggleOutState = 0; // for intake shoot out function

        // sets the first case for the fsm to be in
        outtakeState = OuttakeState.SUBSYSTEMS_SET_RETURN; // tghis is return so all the other states start in the right thing
        coneDepositState = ConeDepositState.IDLE;
        flipConeArmState = FlipConeArmState.IDLE;
        outtakePickupState = OuttakePickupState.IDLE;
        intakeout = IntakeOut.READY;
        groundJunctionsDeposit = GroundJunctionDeposit.READY;
        intakeLiftSequence = IntakeLiftSequence.IDLE;

        turretTargetPosition = 0;
        liftTargetPosition = 50; // if no height is specified, then the lift needs to go out a bit for the transfer
        intakeSlideTargetPosition = 0;
        BeaconScore = false;
    }

    @Override
    public void runOpMode() {

        // this is basically init, all setup, hardware classes etc get initialized here
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot read or write to the same motor mor ethan once or it will issues multiple bulk reads
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //
        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap);

        // waits for user to press start on driverhub
        waitForStart();
        if (opModeIsActive()) {
            // runs setup function before main loop
            Setup();
            // reset the rumble timer before start is pressed - this will be slightly off because we start the program before teleop starts
            runtime.reset();
            location.setPoseEstimate(PoseStorage.currentPose); // this gets sets the current position to what it ended in auto

            while (opModeIsActive()) {
                //rumble();
                location.update();
                // Retrieve your pose
                Pose2d poseEstimate = location.getPoseEstimate();

                xPosition = poseEstimate.getX();
                yPosition = poseEstimate.getY();
                headingPosition = poseEstimate.getHeading();
                correctedHeading = inputs.angleWrap(headingPosition);

                telemetry.addData("x", xPosition);
                telemetry.addData("y", yPosition);
                telemetry.addData("heading", headingPosition);
                telemetry.addData("corrected heading", correctedHeading);

                if (!inputs.DriveToPositionToggleMode){
                    drivebase.Drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                }
                if (gamepad1.dpad_left){
                    location.setPoseEstimate(resetPose);
                }

                //drivebase.motorDirectionTest(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x,gamepad1.right_stick_y);
                //drivebase.PowerToggle(gamepad1.left_stick_button);
                //inputs.gamepadRumbleTimer();
                telemetry.addData("LiftMotorPosition", outtake.liftPos());
                telemetry.addData("turretPosition", outtake.tickstoDegrees((int)Math.round(outtake.turretPos()))); // might be in the wrong degrees/other
                inputs.manualResetToggleMode(gamepad2.left_stick_button);
                telemetry.addData("intakeSlideMotor", outtake.IntakeSlidePos());
                outtakeSequence(); // if gamepad things don't work here then need to pass them in as parameters of this function
                telemetry.addData("Main Outtake State", outtakeState);
                telemetry.addData("Intake Out State", intakeout);
                telemetry.addData("Cone Deposit State", coneDepositState);
                telemetry.addData("Flip Cone State", flipConeArmState);
                telemetry.addData("Outtake Pickup state", outtakePickupState);
                telemetry.addData("Ground Junctions deposit state", groundJunctionsDeposit);
                telemetry.addData("IntakeLiftSequence", intakeLiftSequence);
                telemetry.addData("GroundJunctionsToggleMode", inputs.GroundJunctionsToggleMode);
                telemetry.addData("intakeTouch", outtake.intakeClawTouchPressed());
                telemetry.addData("gamepad2RightStickYPastDeadZone", gamepad2RightStickYPastDeadZone());
                telemetry.addData("intake lift encoder", outtake.getIntakeLiftPos());
                telemetry.addData("intake arm encoder", outtake.getIntakeArmPos());
                telemetry.addData("intake slide motor battery draw", outtake.getIntakeSlideVoltage());
                telemetry.addData("lift motor battery draw", outtake.getLiftVoltage());

                telemetry.update();
            }
        }
    }
    public void outtakeSequence(){ // make sure to go back and add when each statemachine command should be run
        switch (outtakeState) {
            case READY:
                BeaconScore = false; // this is fine because it never reaches the ready stage when scoring beacon
                driveToPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,correctedHeading, 1,1, headingPosition);
                if (intakeout == IntakeOut.READY && !BeaconScore){ // this should reduce any conflicting commands with the intakeshoot out thing
                    outtake.IntakeClawClose();
                    outtake.IntakeArmReady();
                    outtake.IntakeLiftReady();
                } else if (BeaconScore){ // this technicaly isn't needed
                    outtake.IntakeClawClose();
                    outtake.IntakeArmConeHoldForTransfer();
                    outtake.IntakeArmConeHoldForTransfer();
                }

                if (outtakePickupState == OuttakePickupState.READY){ // so it doesn't conflict when lift has to lift up to move counterspringing arm
                    outtake.liftTo(0, outtake.liftPos(),1); // this might reduce the jitters at the start from kd - I Have no clue why
                    liftTargetPosition = 0; // if lift target position is zero, then the transfer won't happen
                    inputs.GroundJunctionsToggleMode = false; // same with the ground junctions
                }
                //outtake.turretSpin(0,outtake.turretPos(),1); // not this because it takes up battery

                if (outtakePickupState == OuttakePickupState.READY){ // so that flip cone thing can be used without conflicting commands and pickup cones
                 // this runs the return sequence for the outtake so that the it can intake as soon as the intake as ready
                    outtake.OuttakeSlideReady();
                    outtake.OuttakeClawOpen();
                    if (GlobalTimer.milliseconds() - OuttakeTimer > 20){ // this might not return if b is pressed when its picking up cones or something - need to test
                        outtake.BraceReady();
                        if (GlobalTimer.milliseconds() - OuttakeTimer > 500){
                            outtake.OuttakeArmReady();
                        }
                    }
                }

                if (gamepad1.b || gamepad2.b){
                    drivebase.intakeSpin(-0.8);
                } else {
                    drivebase.intakeSpin(0);
                }

                if ((gamepad2.right_bumper || gamepad1.right_bumper) && !gamepadRightTriggersDown()){ // if the triggers are used for the cone arm dont go into intake
                    if (outtakePickupState == OuttakePickupState.READY){
                        outtakeState = OuttakeState.INTAKE;
                    }
                }
                if (!gamepadRightTriggersDown()){
                    IntakeShootOut();
                }
                OuttakePickupSequence();
                FlipConeArmSequence();

                break;
            case INTAKE:
                drivebase.intakeSpin(1); // spin the intake
                outtake.IntakeClawOpen();
                outtake.liftTo(0, outtake.liftPos(),1);
                // no need to put ready stuff on because there will be nothing conflicting with it
                if (outtake.intakeClawTouchPressed() || gamepad2.left_bumper){
                    outtakeState = OuttakeState.LIFT_CONE;
                    OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                    outtake.IntakeClawClose();
                }
                intakeClipHoldorNotHold();
                break;
            case LIFT_CONE:
                outtake.IntakeClawClose();
                drivebase.intakeSpin(0);
                outtake.liftTo(0, outtake.liftPos(),1);
                outtake.turretSpinInternalPID(0,1);
                if (GlobalTimer.milliseconds() - OuttakeTimer > 170){
                    outtake.IntakeLiftTransfer();
                    if (outtake.getIntakeLiftPos() > 254){ // this is actually the intake lift
                        outtake.IntakeArmTransfer();
                        if ((outtake.getIntakeArmPos() > 192)){ // put the or statement here for the arm being in the right position
                            if (outtake.liftTargetReached()){ // make sure height is selected before transferring - add intkae slides t hing
                                outtake.OuttakeClawClose();
                                outtakeState = OuttakeState.CLAW_GRIP_TRANSFER_START;
                                OuttakeTimer = GlobalTimer.milliseconds(); // basically grabs in this state
                            }
                        }
                    }
                }
                intakeClipHoldorNotHold();
                break;
            case CLAW_GRIP_TRANSFER_START: // fix so if i don't go
                intakeClipHoldorNotHold();
                if ((outtake.getIntakeArmPos() > 192) && outtake.liftTargetReached() && GlobalTimer.milliseconds() - OuttakeTimer > 80){  // this is actually used as if you are transferring from intake out pickup there is a inbuilt delay - this is an old comment for when there was global timer = 0 - check if intake slidse are all the way in
                    outtake.OuttakeClawClose();
                    outtakeState = OuttakeState.CLAW_GRIP_TRANSFER_END;
                    OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                } else {
                    outtake.liftTo(3, outtake.liftPos(),1);
                    outtake.turretSpin(0,outtake.turretPos(),1);
                    //outtake.IntakeSlideInternalPID(1,1);
                }
                break;
            case CLAW_GRIP_TRANSFER_END: // fix so if i don't go
                intakeClipHoldorNotHold();
                if (GlobalTimer.milliseconds() - OuttakeTimer > 200 || outtake.getIntakeClawPosition() == outtake.IntakeClawOpenHardPos){ // this is so that if the claw has already released don't waste time waiting
                    outtake.IntakeClawOpenHard();
                    if ((GlobalTimer.milliseconds() - OuttakeTimer > 300 || outtake.getIntakeClawPosition() == outtake.IntakeClawOpenHardPos) && ((liftTargetPosition != 0) || inputs.GroundJunctionsToggleMode)){ // make sure height is selected before transferring
                        outtake.liftTo(-50, outtake.liftPos(), 1);
                        outtake.OuttakeArmScore();
                        outtakeState = OuttakeState.HEIGHT_CHANGE_OUTTAKE_DEPOSIT;
                        intakeLiftSequence = IntakeLiftSequence.READY;
                        OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                        if (inputs.GroundJunctionsToggleMode){
                            groundJunctionsDeposit = GroundJunctionDeposit.READY;
                        }
                    }
                }
                break;
            case HEIGHT_CHANGE_OUTTAKE_DEPOSIT:
                if (inputs.GroundJunctionsToggleMode){ // this runs the different sequence for ground junctions
                    GroundJunctions();
                    if (groundJunctionsDeposit == GroundJunctionDeposit.IDLE){ // if the sequence to drop to ground has finished
                        if (!gamepad2RightStickYPastDeadZone()){
                            outtake.liftTo(liftTargetPosition, outtake.liftPos(),1);
                            SlidesUpToggle(gamepad2.right_bumper);
                            SlidesDownToggle(gamepad2.left_bumper);
                        } else {
                            liftFineAdjust();
                        }
                        if (gamepad1.right_bumper) {
                            outtakeState = OuttakeState.DROP;
                        }
                    }
                } else {
                    groundJunctionsDeposit = GroundJunctionDeposit.READY;
                    outtake.turretSpin(turretTargetPosition,outtake.turretPos(),1);
                    if (!gamepad2RightStickYPastDeadZone()){
                        outtake.liftTo(liftTargetPosition, outtake.liftPos(),1);
                        SlidesUpToggle(gamepad2.right_bumper);
                        SlidesDownToggle(gamepad2.left_bumper);
                    } else {
                        liftFineAdjust();
                    }
                    outtake.OuttakeSlideReady(); // outtake slide to top
                    if (gamepad1.right_bumper){
                        outtakeState = OuttakeState.DROP;
                        coneDepositState = ConeDepositState.CONE_DROP; // should run the whole drop sequence here
                        OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                    }
                    outtake.OuttakeArmScore(); // so if you switch back from ground junction mode the arm will go out
                    if (GlobalTimer.milliseconds()-OuttakeTimer > 120){
                        outtake.BraceActive(); // this should happen at the same time as the outtake arm is going out so that its always parrallel to the ground
                    }

                    }
                if (!BeaconScore){
                    if (GlobalTimer.milliseconds()-OuttakeTimer > 150){ // This is returning the intake
                        outtake.IntakeArmReady();
                        if (GlobalTimer.milliseconds()-OuttakeTimer > 150){
                            outtake.IntakeClawClose(); // so it doesn't hit the brushes
                            if (GlobalTimer.milliseconds()-OuttakeTimer > 500){
                                outtake.IntakeLiftReady();
                            }
                        }
                    }
                } else {
                    IntakeConeLiftSequence();// this should be happening when the other side is still outtaking
                }
                if (gamepad2.left_trigger > 0.2){
                    BeaconScore = true; // sets beacon score to true
                } // doesn't set it to false until returning
                break;
            case PICKUP_CONE_AND_BEACON:

                break;
            case DROP:
                if (inputs.GroundJunctionsToggleMode){
                    outtake.OuttakeClawOpenHard();
                    if (GlobalTimer.milliseconds()-OuttakeTimer > 400){
                        outtake.OuttakeSlideReady();
                        if (GlobalTimer.milliseconds()-OuttakeTimer > 600){
                            outtakeState = OuttakeState.SUBSYSTEMS_SET_RETURN;
                            OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                        }
                    }
                } else{
                    ConeDepositSequence(); // this runs constantly to make it happen
                    // run the intake shoot out
                    IntakeShootOut();
                    if (coneDepositState == ConeDepositState.IDLE){ // if the whole sequence has finished
                        outtakeState = OuttakeState.SUBSYSTEMS_SET_RETURN;
                        OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                    }
                }
                break;
            case SUBSYSTEMS_SET_RETURN: // set the cases for return
                IntakeShootOut();
                drivebase.intakeSpin(-1);
                intakeout = IntakeOut.RETURN_SET_TIMER;// sets the intake to return state - will return no matter its state
                outtakeState = OuttakeState.RETURN;
                OuttakeTimer = GlobalTimer.milliseconds(); // reset timer
                // makes all the subsystems states go back to default
                inputs.FlipConeHeightState = 0; // makes the flip cone state go back to zero so it goes to the first state in the modulo rather than a random stage of picking up cones
                flipConeArmState = FlipConeArmState.OUTTAKE_FLIP_CONE_EXECUTE;
                // puts it back to ready so we can use it in ready state of the main state machine
                outtakePickupState = OuttakePickupState.READY;

                break;
            case RETURN:
                // make sure to go to arm reset state for all subsystems
                outtake.liftTo(0, outtake.liftPos(),1);
                outtake.turretSpinInternalPID(0,1); // this has to be internal because we don't hold pos in main thing
                drivebase.intakeSpin(-1);
                IntakeShootOut(); // runs the return case for this
                if (IntakeReady) { // outtake lift target reached might be needed but the whole point is so that we can use the intake while the outtake is resetting
                    outtakeState = OuttakeState.READY; // need the arm to reset as well
                    OuttakeTimer = GlobalTimer.milliseconds(); // reset timer so that the intake claw won't open instantly when its in ready
                }
                outtake.OuttakeSlideReady();
                outtake.OuttakeClawOpen();
                break;

            case MANUAL_ENCODER_RESET: // manual reset in case anything happens
                outtake.liftMotorRawControl(gamepad2.right_stick_y);
                outtake.turretMotorRawControl(gamepad2.left_stick_x);
                outtake.intakeSlideMotorRawControl(gamepad2.left_trigger-gamepad2.right_trigger);
                outtake.IntakeClawClose();
                outtake.OuttakeClawOpenHard(); // shouuld be fine
                outtake.IntakeLiftTransfer();
                outtake.IntakeClipOpen();
                drivebase.intakeSpin(0);
                if (gamepad2.right_bumper){
                    outtake.encodersReset();
                    outtakeState = OuttakeState.READY;
                    inputs.ManualResetToggleMode = false; // this should exit this state once right bumper is pressed
                }
                break;
            case IDLE:
                // puts the state into idle when other actions are happening
                break;
        }
        if ((gamepad2.b && (outtakeState != OuttakeState.READY || intakeout != IntakeOut.READY || groundJunctionsDeposit != GroundJunctionDeposit.IDLE)) || (gamepad1.b && (outtakeState != OuttakeState.READY || intakeout != IntakeOut.READY || groundJunctionsDeposit != GroundJunctionDeposit.IDLE ))){ // so that it returns when intake is out
            outtakeState = OuttakeState.SUBSYSTEMS_SET_RETURN; // if b is pressed at any state then return to ready
            drivebase.intakeSpin(-1);
            BeaconScore = false; // if return do not do the beacon thing
        }
        if (inputs.ManualResetToggleMode){
            outtakeState = OuttakeState.MANUAL_ENCODER_RESET;
        }

        turretPositionChange();
        liftPositionChange();
    }
    public void IntakeShootOut() {
        switch (intakeout) {
            case READY:
                //drivebase.intakeSpin(0);
                intakeClipHoldorNotHold();
                //outtake.IntakeSlideTo(0, outtake.liftPos(), 1);
                outtake.IntakeArmReady();
                outtake.IntakeLiftReady(); // this is in ready as well
                IntakeReady = true; // use this variable somewhere else
                if (gamepad1.left_bumper || inputs.IntakeToggleOutState == 1) { // make sure this function isn't called in a case that it start unintentially
                    intakeout = IntakeOut.INTAKE_SHOOT_OUT; // this will start the timer and the inputs function will also go to 2 at the same time
                    IntakeOutTimer = GlobalTimer.milliseconds();
                }
                break;

                /*
            case INTAKE_INITIAL_LIFT:
                outtake.IntakeLiftTransfer();
                if (GlobalTimer.milliseconds() - IntakeOutTimer > 200) {
                    if (inputs.IntakeToggleOutState == 2){
                        intakeout = IntakeOut.INTAKE_SHOOT_OUT;
                    }
                }
                break;

                 */
            case INTAKE_SHOOT_OUT:
                outtake.IntakeClipOpen(); // time for intake clip to open
                if (GlobalTimer.milliseconds() - IntakeOutTimer > 150){
                    outtake.IntakeSlideTo(IntakeSlideOutTicks, outtake.IntakeSlidePos(), 1);
                    // drivebase.intakeSpin(-0.4); // helps the slides go out
                    if (outtake.IntakeSlidePos() < -100) {
                        outtake.IntakeClawOpenHard();
                        outtake.IntakeLiftReady();
                        outtake.IntakeArmReady();
                    }
                    if (inputs.IntakeToggleOutState == 2){ // if gamepad2.leftbumper is pressed
                        outtake.IntakeClawClose();
                        IntakeOutTimer = GlobalTimer.milliseconds();
                        intakeout = IntakeOut.INTAKE_TO_TRANSFER;
                    }
                }
                break;
            case INTAKE_TO_TRANSFER:
                outtake.IntakeClawClose();
                if (GlobalTimer.milliseconds() - IntakeOutTimer > 200){ // wait for the claw to grab
                    outtake.IntakeLiftTransfer();
                    intakeClipHoldorNotHold();
                    if (GlobalTimer.milliseconds() - IntakeOutTimer > 260){
                        outtake.IntakeArmTransfer();
                    }
                    if (outtake.IntakeSlidePos() > -5 && outtake.getIntakeArmPos() > 192) {
                        IntakeReady = true;
                        if (IntakeReady){ // if the slides are all the way in
                            outtakeState = OuttakeState.CLAW_GRIP_TRANSFER_START; // main state machine takes it from here
                            OuttakeTimer = GlobalTimer.milliseconds() + 200; //offsets the timer
                            outtake.IntakeArmTransfer();
                            //OuttakeTimer = GlobalTimer.milliseconds() - 500; // basically grabs in this state
                            intakeout = IntakeOut.IDLE; // main state machine must set everything to ready in a function
                        }
                    }
                } else {
                    outtake.IntakeClipOpen();
                }
                break;
            case IDLE:
                // nothing happens - when other processes are in use put to idle
                break;
            case RETURN_SET_TIMER: //this runs a timer to set for the return state
                intakeout = IntakeOut.RETURN;
                IntakeOutTimer = GlobalTimer.milliseconds();
                liftTargetPosition = LiftLowPosition;
                break;
            case RETURN:
                if (!BeaconScore){
                    intakeClipHoldorNotHold();
                    if (outtake.IntakeSlidePos() < -100){ // if the intake slides aren't in yet
                        outtake.IntakeSlideTo(0, outtake.IntakeSlidePos(),1); // should conflict as its going to ready too
                        outtake.IntakeLiftReady();
                        outtake.IntakeArmReady();
                        outtake.IntakeClawClose();
                        if (outtake.intakeSlideTargetReached()){
                            intakeout = IntakeOut.READY;
                            inputs.IntakeToggleOutState = 0;
                        } else {
                            drivebase.intakeSpin(0.8); // spin in when its coming in
                        }
                    } else { // if they are in - technically this isnt needed
                        outtake.IntakeLiftReady();
                        outtake.IntakeArmReady();
                        outtake.IntakeClawClose();
                        intakeout = IntakeOut.READY;
                        inputs.IntakeToggleOutState = 0; // makes sure when it goes back to ready it won't autonomatically go back to previous state
                    }
                } else if (BeaconScore) { // doesn't reset the arm if its holding another cone
                    outtakeState = OuttakeState.LIFT_CONE; // goes back to transfer
                    OuttakeTimer = GlobalTimer.milliseconds() + 450; // offsets timer in the main state machine to give outtake time to return
                }
                break;
        }
        inputs.IntakeToggleOut(gamepad1.left_bumper); // ONLY RUN THIS FUNCTION IN PLACES WHERE YOU WANT BUTTON SPAM, DO NOT RUN THIS FUNCTION OTHERWISE INTAKE THING WILL CHANGE STATES WITH BUTTON SPAMS
        // PUT TO IDLE WHEN SWITCHING STATES so the code runs once and this function doesn't re-run with a different intake toggleoutstate
        // put the inputs Toggle out state back to 0 in a function
        if (inputs.IntakeToggleOutState == 0 && intakeout != IntakeOut.IDLE && intakeout != IntakeOut.RETURN && intakeout != IntakeOut.RETURN_SET_TIMER){ // can break if you spam button fix
            intakeout = IntakeOut.READY; // make sure these inputs don't count when this is happening
        }
        else if (inputs.IntakeToggleOutState == 1 && intakeout != IntakeOut.IDLE && intakeout != IntakeOut.RETURN&& intakeout != IntakeOut.RETURN_SET_TIMER){
            intakeout = IntakeOut.INTAKE_SHOOT_OUT;
        }
        if (outtake.intakeClawTouchPressed() && gamepad1.left_trigger < 0.2){ // if the left trigger is down it won't use the limit switch
            inputs.IntakeToggleOutState = 2;
        }
        // 4/3 is zero remainder 3, therefore it goes from 0 to 3
    }


    public void ConeDepositSequence(){ // this doesn't happen in ready state
        switch (coneDepositState) {
            case READY: // not being used at the moment

                break;
            case CONE_DROP:
                coneDepositState = ConeDepositState.BRACE_RETRACT; // basically when it goes into cone drop it instantly goes to next state to start the timer
                ConeDepositTimer = GlobalTimer.milliseconds(); // reset timer
                break;

            case BRACE_RETRACT:
                outtake.OuttakeSlideScoreDrop(); // drops down on pole a bit
                outtake.OuttakeArmDeposit();
                if (GlobalTimer.milliseconds() - ConeDepositTimer > 100){
                    outtake.OuttakeClawOpenHard();
                    if (GlobalTimer.milliseconds() - ConeDepositTimer > 100){
                        outtake.BraceReady(); // might need a new position for this
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
    public void FlipConeArmSequence(){
        switch (flipConeArmState) {
            case OUTTAKE_FLIP_CONE_EXECUTE: // this case is basically when we want the command in toggle, otherwise we set it to idle case
                if (gamepadRightTriggersDown()){ // if the trigger is pressed
                    inputs.coneFlipOuttakeDownToggle(gamepad2.left_bumper||gamepad1.left_bumper, gamepad2.right_bumper||gamepad1.right_bumper);
                    // use if statements for this, state machine not needed
                    if (inputs.AboveConeHeight || inputs.FlipConeHeightState == 0){ // above cone height means if right bumper is pressed at any time return to this
                        outtake.ConeArmAboveCone();
                    } else if (inputs.FlipConeHeightState == 1){
                        outtake.ConeArmDownOnCone();
                    }
                } else {
                    outtake.ConeArmReady();
                    inputs.FlipConeHeightState = 0;
                }
                break;
            case IDLE:
                // does nothing
                break;
        }

    }
    public void OuttakePickupSequence(){ // this basically uses the dual intake to pickup
        switch (outtakePickupState) {
            case READY:
                if (gamepad1.dpad_up || gamepad2.dpad_up){
                    outtakePickupState = OuttakePickupState.READY_TO_OUTTAKE_START;
                    OuttakePickupTimer = GlobalTimer.milliseconds(); // reset timer
                }
                break;
            case READY_TO_OUTTAKE_START:
                if (GlobalTimer.milliseconds() - OuttakePickupTimer > 650){
                    outtake.BraceTucked();
                    outtake.OuttakeClawOpenHard();
                    outtakePickupState = OuttakePickupState.READY_TO_OUTTAKE_END;
                    OuttakePickupTimer = GlobalTimer.milliseconds(); // reset timer
                } else {
                    outtake.OuttakeArmTiltUpSlightly(); // so that there is no conflicting commands
                    outtake.OuttakeSlideReady();
                    outtake.liftTo(-100,outtake.liftPos(),1);
                }
                break;
            case READY_TO_OUTTAKE_END:
                if (GlobalTimer.milliseconds() - OuttakePickupTimer > 300){ // this is the time it takes for the brace to tuck under
                    outtakePickupState = OuttakePickupState.GRAB_OUTTAKE;
                    liftTargetPosition = LiftGroundPosition;
                    outtake.OuttakeClawOpenHard();
                }
                break;
            case GRAB_OUTTAKE: // waiting to for drivers to grab the cone

                outtake.OuttakeArmPickup();
                if (!gamepad2RightStickYPastDeadZone()){
                    SlidesUpToggleFiner(gamepad2.right_bumper);
                    SlidesDownToggleFiner(gamepad2.left_bumper);
                    outtake.liftTo(liftTargetPosition, outtake.liftPos(),1); // bumps should be able to be used here to fine adjust
                } else {
                    liftFineAdjust();
                }
                if (gamepad1.right_bumper){
                    outtake.OuttakeClawClose();
                    inputs.GroundJunctionsToggleMode = true; // so that it won't go up automatically
                    liftTargetPosition = 0; // this is so that the lift will not lift up and topple any cones , you can still use bumpers of joystick to fine adjust, and switch to score using a,x,y
                    OuttakePickupTimer = GlobalTimer.milliseconds(); // reset timer
                    outtake.OuttakeSlideReady(); // lift up regardless
                    outtakePickupState = OuttakePickupState.HEIGHT_OUTTAKE;
                } else {
                    outtake.OuttakeSlidePickupCones();
                }
                break;
            case HEIGHT_OUTTAKE:
                if (GlobalTimer.milliseconds() - OuttakePickupTimer > 240){ // wait 200 ms before the claw closes
                    if (inputs.GroundJunctionsToggleMode){
                        outtake.OuttakeSlideReady(); // lift up regardless
                        outtake.OuttakeArmPickup();
                        if (!gamepad2RightStickYPastDeadZone()){
                            outtake.liftTo(liftTargetPosition, outtake.liftPos(),1);
                            SlidesUpToggle(gamepad2.right_bumper);
                            SlidesDownToggle(gamepad2.left_bumper);
                        } else {
                            liftFineAdjust();
                        }
                        if (gamepad1.right_bumper){
                            outtakeState = OuttakeState.DROP;
                        }
                    } else {
                        outtake.OuttakeSlideReady();
                        outtake.liftTo(liftTargetPosition, outtake.liftPos(),1);
                        if (GlobalTimer.milliseconds() - OuttakePickupTimer > 260){ // just wait a teensy weensy bit before arm tilts
                            outtake.OuttakeArmTiltUpSlightly();
                            outtake.BraceActive();
                            outtakePickupState = OuttakePickupState.IDLE;
                            liftTargetPosition = LiftLowPosition;
                            groundJunctionsDeposit = GroundJunctionDeposit.READY;
                            outtakeState = OuttakeState.HEIGHT_CHANGE_OUTTAKE_DEPOSIT; // this goes to the deposit stage of the main state machine
                        }
                    }
                }
                break;
            case IDLE:
                // yeah sets it to idle so no multiple commands can happen at once
                break;
        }
    }
    public void GroundJunctions(){ // this basically uses the dual intake to pickup
        switch (groundJunctionsDeposit) {
            case READY:
                GroundJunctionDepositTimer = GlobalTimer.milliseconds(); // reset timer
                groundJunctionsDeposit = GroundJunctionDeposit.READY_TO_OUTTAKE_START;
                break;
            case READY_TO_OUTTAKE_START:
                outtake.OuttakeClawClose();
                if (GlobalTimer.milliseconds() - OuttakePickupTimer > 650){
                    outtake.BraceTucked();
                    groundJunctionsDeposit = GroundJunctionDeposit.READY_TO_OUTTAKE_END;
                    GroundJunctionDepositTimer = GlobalTimer.milliseconds(); // reset timer
                } else {
                    outtake.OuttakeArmTiltUpSlightly(); // so that there is no conflicting commands
                    outtake.OuttakeSlideReady();
                    outtake.liftTo(-100,outtake.liftPos(),1);
                }
                break;
            case READY_TO_OUTTAKE_END:
                if (GlobalTimer.milliseconds() - GroundJunctionDepositTimer > 300){ // this is the time it takes for the brace to tuck under
                    groundJunctionsDeposit = GroundJunctionDeposit.DROP_TO_GROUND;
                }
                break;
            case DROP_TO_GROUND: // waiting to for drivers to grab the cone
                outtake.OuttakeSlidePickupCones();
                outtake.OuttakeArmPickup();
                outtake.liftToInternalPID(LiftGroundPosition,0.8);
                if (GlobalTimer.milliseconds() - GroundJunctionDepositTimer > 300){
                    groundJunctionsDeposit = GroundJunctionDeposit.IDLE;
                }
                break;
            case IDLE:
                // yeah sets it to idle so no multiple commands can happen at once
                break;
        }
    }
    public void IntakeConeLiftSequence(){
        switch (intakeLiftSequence) {
            case READY: // this case is basically when we want the command in toggle, otherwise we set it to idle case
                // reseting happens here so no conflicting writes, should return when going height linkage
                if (GlobalTimer.milliseconds()-OuttakeTimer > 150){ // This is returning the intake
                    outtake.IntakeArmReady();
                    if (GlobalTimer.milliseconds()-OuttakeTimer > 150){
                        outtake.IntakeClawClose(); // so it doesn't hit the brushes
                        if (GlobalTimer.milliseconds()-OuttakeTimer > 500){
                            outtake.IntakeLiftReady();
                            if (gamepad2.right_bumper && !gamepadRightTriggersDown()){ // if the triggers are used for the cone arm dont go into intake
                                intakeLiftSequence = IntakeLiftSequence.INTAKE;
                            }
                        }
                    }
                }
                break;
            case INTAKE:
                drivebase.intakeSpin(1); // spin the intake
                outtake.IntakeClawOpen();
                // no need to put ready stuff on because there will be nothing conflicting with it
                if (outtake.intakeClawTouchPressed() || gamepad2.left_bumper){
                    intakeLiftSequence = IntakeLiftSequence.LIFT_CONE;
                    IntakeLeftSequenceTimer = GlobalTimer.milliseconds(); // reset timer
                    outtake.IntakeClawClose();
                }
                break;
            case LIFT_CONE:
                outtake.IntakeClawClose();
                if (GlobalTimer.milliseconds() - IntakeLeftSequenceTimer > 170){
                    outtake.IntakeLiftTransfer();
                    if (outtake.getIntakeLiftPos() > 254){ // this is actually the intake lift
                        outtake.IntakeArmConeHoldForTransfer();
                        BeaconScore = true;
                        }
                    }
                break;
            case IDLE:
                // does nothing
                break;
        }
    }

    public void liftPositionChange(){ // if gamepad inputs don't work in this class then pass them through as parameters in the function
        if (gamepad2.y || gamepad1.y){
            liftTargetPosition = LiftHighPosition; // add servo change here if its different for each height
            //GroundJunctions = false;
        } else if (gamepad2.x || gamepad1.x){
            liftTargetPosition = LiftMidPosition;
            //GroundJunctions = false; // the way setting a variable works is that it won't change until you change it back to false
            // therefore, if we want to switch off ground junction mode we need to be able to switch it off true
        } else if (gamepad2.a || gamepad1.a || gamepad2.right_trigger > 0.2){ // if the trigger is pressed then select a height, so that the simaltaneous scoring can still commence
            liftTargetPosition = LiftLowPosition;
            //GroundJunctions = false;
        } else if (gamepad1.dpad_down || gamepad2.dpad_down){
           liftTargetPosition = LiftGroundPosition;
        }
        inputs.groundJunctionsToggle(gamepad2.dpad_down || gamepad1.dpad_down);
    }
    public void turretPositionChange(){
        if (gamepad1.dpad_up || gamepad2.dpad_up){
            turretTargetPosition = 0;
        } else if (gamepad1.dpad_right || gamepad2.dpad_right){
           // turretTargetPosition = TurretRightposition;
        } else if (gamepad1.dpad_left || gamepad2.dpad_left){
            //turretTargetPosition = TurretLeftPosition;
        }
    }
    public void resetAllMotors(){
        outtake.IntakeSlideTo(0, outtake.IntakeSlidePos(),1); // might break something
        outtake.liftTo(0, outtake.liftPos(),1);
        outtake.turretSpin(0,outtake.turretPos(),1);
    }
    public void liftFineAdjust(){ // won't work with the fine adjust from the bumpers, need to make sure the target positions are set on return??
        if (gamepad2RightStickYPastDeadZone()) { // makes the deadzone for the controller more
            if (outtake.liftPos() > LiftUpperLimit && outtake.liftPos() < 1){ // this is so that you can't fine adjust so much that the slide just keeps going and breaks something
                outtake.liftMotorRawControl(gamepad2.right_stick_y); // this is the fine adjust - the encoder should still keep its position in this
            }
        }
    }
    public boolean gamepad2RightStickYPastDeadZone(){
        if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2) { // makes the deadzone for the controller more
            return true;
        } else {
            return false;
        }
    }
    public boolean gamepadRightTriggersDown(){
        if (gamepad2.right_trigger < 0.2 && gamepad1.right_trigger < 0.2){
            return false;
        } else{
            return true;
        }
    }
    public void driveToPosition(double xTarget, double yTarget, double headingTarget, double xPosition, double yPosition, double correctedHeading, double maxTranslationalSpeed, double maxRotationalSpeed, double rawHeading){
        inputs.driveToPositionToggle(gamepad1.dpad_right);
        if (inputs.DriveToPositionToggleMode){
            drivebase.DriveToPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,correctedHeading, maxTranslationalSpeed,maxRotationalSpeed); // last values are translationalspeed, and rotational speed
            if ((drivebase.getDistanceFromPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,rawHeading) < distanceFromPointThreshold) && drivebase.getHeadingError() < Math.toRadians(Math.abs(3))){ // have to deal with the heading here, read telemetry for heading angle
                if (inputs.IntakeToggleOutState == 0){ //so that it will only happen once
                    inputs.IntakeToggleOutState = 1;
                }
                telemetry.addLine("DRIVE HAS REACHED POSITION");
            }
        }
        if (gamepad1SticksBeingUsed()){
            inputs.DriveToPositionToggleMode = false; // if the gamepad sticks are moved then stop the automatic driving

        }
    }
    public boolean gamepad1SticksBeingUsed(){
        double gamepadThresholdDistance = 0.13;
        if (gamepad1.right_stick_x > gamepadThresholdDistance || gamepad1.right_stick_x < -gamepadThresholdDistance || gamepad1.left_stick_y > gamepadThresholdDistance || gamepad1.left_stick_y < -gamepadThresholdDistance || gamepad1.left_stick_x > gamepadThresholdDistance || gamepad1.left_stick_x < -gamepadThresholdDistance) { // makes the deadzone for the controller more
            return true;
        } else {
            return false;
        }
    }

    public void rumbleSetup() {
        oneFourthRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 100)
                .build();

        halfRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 750)
                .build();

        endgameRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 250)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 250)
                .build();

        tenRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 1000)
                .build();

        switchRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 500)
                .build();
    }

    public void rumble(){ // this runs the logic for when and what rumbles will go off at what time
        if(runtime.seconds() > oneFourth && one){
            gamepad1.runRumbleEffect(oneFourthRumbleEffect);
            gamepad2.runRumbleEffect(oneFourthRumbleEffect);
            one = false;
            two = true;
        }
        if(runtime.seconds() > halfTime && two){
            gamepad1.runRumbleEffect(halfRumbleEffect);
            gamepad2.runRumbleEffect(halfRumbleEffect);
            two = false;
            three = true;
        }
        if(runtime.seconds() > endgame && three){
            gamepad1.runRumbleEffect(endgameRumbleEffect);
            gamepad2.runRumbleEffect(endgameRumbleEffect);
            three = false;
            four = true;
        }
        if(runtime.seconds() > tenTime && four) {
            gamepad1.runRumbleEffect(tenRumbleEffect);
            gamepad2.runRumbleEffect(tenRumbleEffect);
            four = false;
        }
    }
    public void SlidesUpToggle (boolean SlidesUpBtn){
        if (SlidesUpBtn) {
            if (!SlidesToggleUp) {
                SlidesToggleUp = true;
                if (liftTargetPosition > LiftUpperLimit + 85){
                    liftTargetPosition -= 85; // increases by 50 every time
                }
            }
        }
        else {
            SlidesToggleUp = false;
        }
    }
    public void SlidesDownToggle (boolean SlidesDownBtn){
        if (SlidesDownBtn) {
            if (!SlidesToggleDown) {
                SlidesToggleDown = true;
                if (liftTargetPosition < -80){
                    liftTargetPosition += 85; // decreases by 50 every time
                }
            }
        }
        else {
            SlidesToggleDown = false;
        }
    }
    public void SlidesUpToggleFiner (boolean SlidesUpBtn){
        if (SlidesUpBtn) {
            if (!SlidesToggleUp) {
                SlidesToggleUp = true;
                if (liftTargetPosition > LiftUpperLimit){
                    liftTargetPosition -= 50; // increases by 50 every time
                }
            }
        }
        else {
            SlidesToggleUp = false;
        }
    }
    public void SlidesDownToggleFiner (boolean SlidesDownBtn){
        if (SlidesDownBtn) {
            if (!SlidesToggleDown) {
                SlidesToggleDown = true;
                if (liftTargetPosition < 0){
                    liftTargetPosition += 50; // decreases by 50 every time
                }
            }
        }
        else {
            SlidesToggleDown = false;
        }
    }

    public void intakeClipHoldorNotHold(){
        if (outtake.IntakeSlidePos() > -2) {
            outtake.IntakeClipHold(); // turn the intake slide pid running to pos off to save battery draw
            outtake.intakeSlideMotorRawControl(0);
        } else {
            outtake.IntakeClipOpen(); // this might break something when as the intake slides won't go in, but stops jittering
            outtake.IntakeSlideTo(3, outtake.IntakeSlidePos(),1);
        }
    }

}


