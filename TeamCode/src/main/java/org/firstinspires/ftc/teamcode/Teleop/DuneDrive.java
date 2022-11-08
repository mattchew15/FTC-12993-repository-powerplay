package org.firstinspires.ftc.teamcode.Teleop;

// Old imports, some not needed
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "DuneDrive")
public class DuneDrive extends LinearOpMode {

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

    // states for asynchronus sequences
    enum OutakeState {
        READY, // ready for pickup
        INTAKE, // either normal, or bar up and down
        GRAB, // grip claw
        INITIAL_LIFT, // has to go over intake motors and rev hubs
        TURN_LIFT_TILT,
        HEIGHT_LINKAGE, // linkage extends once nearly finished turning - stability. Lift goes to correct level simaltaneously
        FINE_ADJUST, // gamepad2 controls fine adjust
        DROP, // gamepad1.rightbumper
        RETURN // calculate the fastest way for everything to be moving simaltaneously
    }

    // create instance of OutakeState Enum and set it to ready
    OutakeState outakestate = OutakeState.READY;

    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        inputs.resetMatchTimer();
        drivebase.motorsSetup();
        turretlift.motorsSetup();
        outakestate = OutakeState.READY;

        outakesequencetimer = 0;
        liftpositiontype = 350;
    }

    @Override
    public void runOpMode() {
        // this is basically init, all setup, hardware classes etc get initialized here
        drivebase.Drivebase_init(hardwareMap);
        turretlift.TurretLift_init(hardwareMap);


        // waits for user to press start on driverhub
        waitForStart();
        if (opModeIsActive()) {
            // runs setup function before main loop
            Setup();

            while (opModeIsActive()) {
                // Main loop. Run class methods here to do stuff
                drivebase.Drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                drivebase.PowerToggle(gamepad1.a);
                liftSequence();
                //inputs.gamepadRumbleTimer();
                //drivebase.motorDirectionTest(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y);
                telemetry.addData("LiftMotorPosition", turretlift.liftPos());
                telemetry.addData("turretPosition", turretlift.tickstoDegrees((int)Math.round(turretlift.turretPos())));
                telemetry.addData("turret raw position", turretlift.turretPos());
                telemetry.addData("lift motor current draw", turretlift.getLiftVoltage());
                telemetry.addData("sequence state", outakestate);
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
                if (gamepad2.right_bumper) {
                    outakestate = OutakeState.INTAKE;
                    intaketype = 0; // normal orientated cone
                }
                else if (gamepad2.left_bumper) {
                    outakestate = OutakeState.INTAKE;
                    intaketype = 1; // sideways bar cone
                }
                /*
                else if (gamepad2.x){
                    outakestate = OutakeState.INTAKE;
                    intaketype = 2; // will be lip facing towards robot
                }
                */

                break;

            case INTAKE:
                turretlift.liftToInternalPID(0,1);
                turretlift.turretSpinInternalPID(0, 1);
                turretlift.readyServos();
                if (intaketype == 0){ // normal intake case
                    drivebase.intakeSpin(0.65);
                    drivebase.intakeBarUp();
                    intakesequencetransition();
                }
                else if (intaketype == 1 || intaketype == 2) { // second intake case
                    drivebase.intakeSpin(0.65);
                    drivebase.intakeBarDown();
                    intakesequencetransition();
                }

                break;

            case GRAB:
                if (GlobalTimer.milliseconds() - outakesequencetimer > 0){ // this if statement is not needed at all
                    drivebase.intakeSpin(0);
                    turretlift.closeClaw();
                    turretlift.linkageIn();
                    drivebase.intakeBarDown();
                    turretPositionChange(); // allows drivers to change the turret position in this state
                    if (GlobalTimer.milliseconds() - outakesequencetimer > 300){
                        turretlift.closeClaw(); // not needed, servo is already going to position?
                        turretlift.liftToInternalPID(350,1);
                        turretlift.tiltUpHalf();
                        if (turretlift.liftTargetReachedInternalPID()){
                            telemetry.addLine("lift is up");
                            turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype), 1); //
                            outakestate = OutakeState.HEIGHT_LINKAGE; // it goes to height linkage thats why the lift goes up by default
                        }
                    }
                }
                break;

            case HEIGHT_LINKAGE:
                turretlift.closeClaw();
                turretlift.liftToInternalPID(liftpositiontype,1);
                turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype), 1);
                if (turretlift.liftTargetReached() && liftpositiontype == liftHighPosition){
                    turretlift.tiltUp();
                }
                else{
                    turretlift.tiltUpHalf();
                }
                liftPositionChange(); // change this to transition state for fine adjust!!! if you dont want fine adjust then dont do it
                break;

            case DROP:
                turretlift.liftToInternalPID(liftpositiontype,1); // makes the lift drop before it returns
                turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype), 1);
                if (GlobalTimer.milliseconds() - outakesequencetimer > 300){
                    if (gamepad1.right_bumper){ //
                        turretlift.openClaw();
                        turretlift.linkageIn();
                        outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
                        turretlift.liftToInternalPID(liftpositiontype - 200,1); // makes the lift drop before it returns
                        outakestate = OutakeState.RETURN;
                    }
                }
                break;

            case RETURN:
                if (GlobalTimer.milliseconds() - outakesequencetimer > 300){ // make faster
                    turretlift.turretSpinInternalPID(0, 1);
                    turretlift.tiltReset();
                    turretlift.linkageIn();
                    turretlift.closeClaw();
                    drivebase.intakeSpin(-0.5);
                    if (turretlift.turretTargetReachedInteralPID()){
                        //telemetry.addData("turret return target reached?", true);
                        telemetry.addLine("turret return target reached");
                        turretlift.liftToInternalPID(0,0.8); // slower so nothing breaks
                        if (turretlift.liftTargetReachedInternalPID()){
                            turretlift.openClaw();
                            outakestate = OutakeState.READY;
                        }
                    } else {
                        turretlift.liftToInternalPID(350,1); // going down while turning
                    }
                }
                else {
                    turretlift.liftToInternalPID(350,1); // could be faster
                }
                break;
        }
        if (gamepad2.b && outakestate != OutakeState.READY || gamepad1.b && outakestate != OutakeState.READY){
            outakestate = OutakeState.RETURN; // return to ready state no matter what state the system is in
        }
        intakesequencetransition(); // not sure this works, but at any stage if you press dpad it will go back to the grab phase and you can rotate the turret
        liftPositionChangenostatechange();
    }

    public void intakesequencetransition(){ // simplifies transition from INTAKE state to GRAB state
        if (gamepad2.dpad_up && outakestate != OutakeState.DROP){ // favour the mid level over the other one
            turretpositiontype = turretlift.degreestoTicks(180);
            outakestate = OutakeState.GRAB;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.dpad_left){
            turretpositiontype = turretlift.degreestoTicks(90);
            outakestate = OutakeState.GRAB;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.dpad_right){
            turretpositiontype = turretlift.degreestoTicks(-90);
            outakestate = OutakeState.GRAB;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.dpad_down){
            outakestate = OutakeState.GRAB;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
    }

    public void turretPositionChange(){
        if (gamepad2.dpad_up){
            turretpositiontype = turretlift.degreestoTicks(180);
        }
        else if (gamepad2.dpad_left){
            turretpositiontype = turretlift.degreestoTicks(90);
        }
        else if (gamepad2.dpad_right){
            turretpositiontype = turretlift.degreestoTicks(-90);
        }
        else if (gamepad2.dpad_down){
            turretpositiontype = 0;
        }
    }

    public void liftPositionChange(){
        if (gamepad2.y){
            liftpositiontype = liftHighPosition;
            outakestate = OutakeState.DROP;
            turretlift.tiltUp();
            turretlift.linkageOutHalf();
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.a){
            liftpositiontype = 450;
            turretlift.tiltReset();
            turretlift.linkageOutQuarter();
            outakestate = OutakeState.DROP;
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }
        else if (gamepad2.x){
            liftpositiontype = 700;
            outakestate = OutakeState.DROP;
            turretlift.tiltUpHalf();
            turretlift.linkageOutHalf();
            outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
        }


    }
    public void liftPositionChangenostatechange(){
        if (gamepad2.y){
            liftpositiontype = liftHighPosition;
        }
        else if (gamepad2.a){
            liftpositiontype = 450;
            turretlift.tiltReset();

        }
        else if (gamepad2.x){
            liftpositiontype = 630;
        }


    }
}

