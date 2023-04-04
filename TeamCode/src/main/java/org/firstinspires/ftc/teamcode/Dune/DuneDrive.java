package org.firstinspires.ftc.teamcode.Dune;

// Old imports, some not needed
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name = "DuneDrive")
public class DuneDrive extends LinearOpMode {


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

    boolean one = true;
    boolean two = false;
    boolean three = false;
    boolean four = false;

    //create new instances of class
    DriveBase drivebase = new DriveBase();
    TurretLift turretlift = new TurretLift();
    Inputs inputs = new Inputs();


    // uses the Orientation class from the SDK to create variable angles
    Orientation angles;
    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    ElapsedTime GlobalTimer;

    int intaketype; // have to initialize here instead of inside method
    double turretpositiontype;
    int liftpositiontype;
    double outakesequencetimer; // change these into the class, optimize object oriented code its kinda clunky rn
    int liftHighPosition = 850;
    int liftMidPosition = 650;

    // states for asynchronus sequences
    enum OutakeState {
        READY, // ready for pickup
        INTAKE, // either normal, or bar up and down
        PICKUP_STACK, // have the linkage go out to pick up from the stack in teleop - mika presses left and right bumper
        // to go up and down from 5 to 0, then presses other pickup dpad pickup and height adjust like normal
        // mika toggles left button to enter and exit mode, gamepad rumble feedback occurs when switched mode for both drivers
        GRAB, // grip claw
        HEIGHT_LINKAGE, // linkage extends once nearly finished turning - stability. Lift goes to correct level simaltaneously
        FINE_ADJUST, // gamepad2 controls fine adjust
        DROP, // gamepad1.rightbumper
        RETURN, // calculate the fastest way for everything to be moving simaltaneously
        RETURN_PICKUP_STACK,
        MANUAL_ENCODER_RESET // USE VERY CAREFULLY
    }

    // create instance of OutakeState Enum and set it to ready
    OutakeState outakestate = OutakeState.READY;

    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        inputs.resetMatchTimer();
        drivebase.motorsSetup();
        //turretlift.motorsSetup();
        inputs.inputsSetup(); // sets toggle variables to false
        outakestate = OutakeState.READY;
        outakesequencetimer = 0;
        //liftpositiontype = 350;
    }


    @Override
    public void runOpMode() {

        rumbleSetup();

        // this is basically init, all setup, hardware classes etc get initialized here
        drivebase.Drivebase_init(hardwareMap);
        turretlift.TurretLift_init(hardwareMap);

        while (!isStarted()) {
            turretlift.releaseClaw();
        }

        // waits for user to press start on driverhub
        waitForStart();
        if (opModeIsActive()) {
            runtime.reset();
            // runs setup function before main loop
            Setup();

            while (opModeIsActive()) {

                rumble();
                // Main loop. Run class methods here to do stuff
                drivebase.Drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                //drivebase.motorDirectionTest(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x,gamepad1.right_stick_y);
                drivebase.PowerToggle(gamepad1.a);
                /*liftSequence();
                inputs.intakeStackToggleMode(gamepad2.right_stick_button);
                inputs.manualResetToggleMode(gamepad2.left_stick_button);
                inputs.cycleToggleUp(gamepad2.right_bumper);
                inputs.cycleToggleDown(gamepad2.left_bumper);*/
                //inputs.gamepadRumbleTimer();
                telemetry.addData("LiftMotorPosition", turretlift.liftPos());
                telemetry.addData("turretPosition", turretlift.tickstoDegrees((int)Math.round(turretlift.turretPos())));
                telemetry.addData("turret raw position", turretlift.turretPos());
                telemetry.addData("lift motor current draw", turretlift.getLiftVoltage());
                telemetry.addData("sequence state", outakestate);
                //telemetry.addData("linkageposition:", turretlift.getLinkagePosition());
                telemetry.addData("INTAKE STACK TOGGLE MODE", inputs.IntakeStackToggleMode);
                telemetry.addData("INTAKE TOUCH", turretlift.intakeTouchPressed());
               // telemetry.addData("")
                telemetry.update();
            }
        }
    }

    // sequence functions go here;
    public void liftSequence(){
        switch (outakestate) {
            case READY:
                turretlift.liftToInternalPID(0,1);
                turretlift.turretSpinInternalPID(0, 1);
                turretlift.readyServos();
                drivebase.intakeBarUp();
                if ((gamepad2.right_bumper || gamepad1.left_bumper) && !inputs.IntakeStackToggleMode) { // not statement to ensure it stays in toggle if you press gamepad
                    outakestate = OutakeState.INTAKE;
                    intaketype = 0; // normal orientated cone
                }
                else if (gamepad2.left_bumper && !inputs.IntakeStackToggleMode) {
                    outakestate = OutakeState.INTAKE;
                    intaketype = 1; // sideways bar cone
                }
                else if (inputs.IntakeStackToggleMode){ // this will go to this case
                    outakestate = OutakeState.PICKUP_STACK;
                }
                /*
                else if (gamepad2.x){
                    outakestate = OutakeState.INTAKE;
                    intaketype = 2; // will be lip facing towards robot
                }
                */
                break;

            case PICKUP_STACK:
                drivebase.intakeBarDown();
                turretlift.turretSpinInternalPID(0, 1);
                turretlift.liftTo(185 - inputs.IntakeHeightState * 35,turretlift.liftPos(), 1); // 20 controls imcrement amount
                if (turretlift.liftPos() > 80){ // greater then the height it hits the bar
                    turretlift.linkageOutHalf(); // should be driving to touch tthe cone, not extending to touch the cone
                    turretlift.openClawHard(); // makes alligning easier
                    intakesequencetransition(); // limit switch might be annoying
                    /*if (!inputs.IntakeStackToggleMode){

                    } */
                }
                else{
                    turretlift.linkageIn();
                }
                break;

            case INTAKE:
                turretlift.liftToInternalPID(0,1);
                turretlift.turretSpinInternalPID(0, 1);
                turretlift.readyServos();
                if (intaketype == 0){ // normal intake case
                    drivebase.intakeSpin(1);
                    drivebase.intakeBarUp();
                    intakesequencetransition(); // this is the dpad or limit switch thing
                }
                else if (intaketype == 1 || intaketype == 2) { // second intake case
                    drivebase.intakeSpin(1);
                    drivebase.intakeBarDown();
                    intakesequencetransition();
                }

                break;

            case GRAB:
                drivebase.intakeSpin(0);
                turretlift.closeClaw();
                drivebase.intakeBarDown();
                turretPositionChange(); // allows drivers to change the turret position in this state
                if (GlobalTimer.milliseconds() - outakesequencetimer > 200){
                    telemetry.addLine("past this stageeeeeeeeeeeeeeeeeeeeee");
                    turretlift.closeClaw(); // not needed, servo is already going to position?
                    turretlift.liftToInternalPID(liftpositiontype,1);
                    //turretlift.linkageIn(); // this should make it so that the linkage doesnt go in when we are picking up from stack
                    turretlift.tiltUpHalf();
                    if (turretlift.liftPos() > 250){ // change this if it doesn't work
                        turretlift.linkageIn();
                        telemetry.addLine("lift is up");
                        turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype), 1); //
                        outakestate = OutakeState.HEIGHT_LINKAGE; // it goes to height linkage thats why the lift goes up by default
                    }
                }
                else { // linkage in/out depending on if we intake stack
                    if (inputs.IntakeStackToggleMode){
                        turretlift.linkageNearlyOutHalf();
                    }
                    else{
                        turretlift.linkageIn();
                    }
                }
                break;

            case HEIGHT_LINKAGE:
                turretlift.closeClaw();
                turretlift.liftToInternalPID(liftpositiontype,1);
                turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype), 1);
                if (turretlift.turretTargetReachedInteralPID()){ //  && liftpositiontype == liftHighPosition
                    outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
                    outakestate = OutakeState.DROP;
                    liftservoPositionChange();
                }
                else{
                    turretlift.tiltUpHalf();
                }
                break;

            case DROP:
                turretlift.liftToInternalPID(liftpositiontype,1); // makes the lift drop before it returns
                turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype), 1);
                liftservoPositionChange();
                if (gamepad1.right_bumper){ //
                    turretlift.openClaw();
                    outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
                    outakestate = OutakeState.RETURN;
                }

                break;

            case RETURN: // could add it so it goes straight to linkage out for intaking stack
                if (GlobalTimer.milliseconds() - outakesequencetimer > 100){
                    turretlift.linkageIn();
                    if (GlobalTimer.milliseconds() - outakesequencetimer > 300){ // make faster wait for drop
                        turretlift.turretSpinInternalPID(0, 1);
                        turretlift.tiltReset();
                        turretlift.linkageIn();
                        turretlift.closeClawHard();
                        drivebase.intakeSpin(-0.7);
                        if (turretlift.turretTargetReachedInteralPIDNewThreshold()){
                            //telemetry.addData("turret return target reached?", true);
                            telemetry.addLine("turret return target reached");
                            turretlift.liftToInternalPID(0,0.8); // slower so nothing breaks
                            if (turretlift.liftPos() < 30){
                                turretlift.openClaw();
                                outakestate = OutakeState.READY;
                            }
                        } else {
                            turretlift.liftToInternalPID(350,1); // going down while turning
                        }
                    }
                }
                break;

            case RETURN_PICKUP_STACK: // could add it so it goes straight to linkage out for intaking stack
                if (GlobalTimer.milliseconds() - outakesequencetimer > 300){ // make faster
                    turretlift.turretSpinInternalPID(0, 1);
                    turretlift.tiltReset();
                    turretlift.closeClawHard();
                    drivebase.intakeSpin(-0.5);
                    if (turretlift.turretTargetReachedInteralPID()){
                        //telemetry.addData("turret return target reached?", true);
                        telemetry.addLine("turret return target reached");
                        turretlift.liftToInternalPID(230,1); // slower so nothing breaks
                        if (turretlift.liftPos() > 170){ //turretlift.liftTargetReachedInternalPID()
                            turretlift.linkageIn();
                            outakestate = OutakeState.RETURN;
                            inputs.IntakeStackToggleMode = false;
                            outakesequencetimer = GlobalTimer.milliseconds();
                        }
                    } else {
                        turretlift.liftToInternalPID(350,1); // going down while turning
                    }
                }
                else {
                    turretlift.liftToInternalPID(350,1); // could be faster
                }
                break;

            case MANUAL_ENCODER_RESET: // manual reset in case anything happens
                turretlift.liftMotorRawControl(gamepad2.left_stick_y);
                turretlift.turretMotorRawControl(gamepad2.left_stick_x);
                turretlift.tiltReset();
                turretlift.closeClawHard();
                if (gamepad2.right_bumper){
                    turretlift.encodersReset();
                    outakestate = OutakeState.READY;
                }
                if (gamepad2.left_bumper){
                    turretlift.linkageOut();
                } else {
                    turretlift.linkageIn();
                }
                break;

        }
        if ((gamepad2.b && outakestate != OutakeState.READY) || (gamepad1.b && outakestate != OutakeState.READY)){
            if (inputs.IntakeStackToggleMode){
                outakestate = OutakeState.RETURN_PICKUP_STACK; // return to ready state no matter what state the system is in
            }
            else{
                outakestate = OutakeState.RETURN; // return to ready state no matter what state the system is in
            }

           // inputs.IntakeStackToggleMode = false;
        }
        justliftPositionChange();
        if (inputs.ManualResetToggleMode){
            outakestate = OutakeState.MANUAL_ENCODER_RESET;
            if (gamepad2.right_stick_button){
                gamepad2.runRumbleEffect(switchRumbleEffect);
                gamepad1.runRumbleEffect(switchRumbleEffect);
            }
        }
    }

    public void intakesequencetransition(){ // simplifies transition from INTAKE state to GRAB state
        if ((gamepad2.dpad_up || gamepad1.dpad_up || turretlift.intakeTouchPressed()) && outakestate != OutakeState.GRAB && outakestate != OutakeState.HEIGHT_LINKAGE && outakestate != OutakeState.RETURN && outakestate != OutakeState.DROP && outakestate != OutakeState.MANUAL_ENCODER_RESET && outakestate != OutakeState.READY && gamepad2.right_trigger < 0.6 && gamepad2.left_trigger < 0.6){ // this was here dont know why && outakestate != OutakeState.DROP
            turretpositiontype = turretlift.degreestoTicks(180); // by default it will go 180 if limit switch touched
            outakestate = OutakeState.GRAB;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.dpad_left || gamepad1.dpad_left){
            turretpositiontype = turretlift.degreestoTicks(90);
            outakestate = OutakeState.GRAB;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.dpad_right || gamepad1.dpad_right){
            turretpositiontype = turretlift.degreestoTicks(-90);
            outakestate = OutakeState.GRAB;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
    }

    public void turretPositionChange(){
        if (gamepad2.dpad_up || gamepad1.dpad_up){
            turretpositiontype = turretlift.degreestoTicks(180);
        }
        else if (gamepad2.dpad_left || gamepad1.dpad_left){
            turretpositiontype = turretlift.degreestoTicks(90);
        }
        else if (gamepad2.dpad_right || gamepad1.dpad_right){
            turretpositiontype = turretlift.degreestoTicks(-90);
        }
    }

    public void liftPositionChange(){
        if (gamepad2.y || gamepad1.y){
            liftpositiontype = liftHighPosition;
            turretlift.tiltUp();
            turretlift.linkageOutHalf();
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
            outakestate = OutakeState.DROP;
        }
        else if (gamepad2.a || gamepad1.a){
            liftpositiontype = 450;
            turretlift.tiltReset();
            turretlift.linkageOutQuarter();
            outakestate = OutakeState.DROP;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.x || gamepad1.x){
            liftpositiontype = liftMidPosition;
            outakestate = OutakeState.DROP;
            turretlift.tiltUpHalf();
            turretlift.linkageOutHalf();
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }


    }
    public void liftservoPositionChange(){
        if (liftpositiontype == liftHighPosition){
            turretlift.tiltUp();
            turretlift.linkageOutHalf();

        }
        else if (liftpositiontype == 450){
          //  liftpositiontype = 450;
            turretlift.tiltReset();
            turretlift.linkageOutQuarter();

        }
        else if (liftpositiontype == liftMidPosition){
           // liftpositiontype = liftMidPosition;
            turretlift.tiltUpHalf();
            turretlift.linkageOutHalf();
        }
    }

    public void justliftPositionChange(){
        if (gamepad2.y || gamepad1.y){
            liftpositiontype = liftHighPosition;
        }
        else if (gamepad2.a || gamepad1.a){
            liftpositiontype = 450;
        }
        else if (gamepad2.x || gamepad1.x){
            liftpositiontype = liftMidPosition;
        }else if (gamepad2.dpad_down || gamepad1.dpad_down){
            liftpositiontype = 50;
        }
    }

    public void rumble(){
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
}

