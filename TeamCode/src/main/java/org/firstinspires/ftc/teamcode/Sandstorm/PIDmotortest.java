package org.firstinspires.ftc.teamcode.Sandstorm;

// Old imports, some not needed
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "PIDmotortest")
public class PIDmotortest extends LinearOpMode {

    //create new instances of class
    DriveBase drivebase = new DriveBase();
    OuttakeSequence outtakeSequence = new OuttakeSequence();
    Inputs inputs = new Inputs();
    Outtake outtake = new Outtake();
    //Inputs inputs = new Inputs();

    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    ElapsedTime GlobalTimer;

    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        drivebase.motorsSetup();
        outtake.hardwareSetup();
        inputs.inputsSetup();
        outtakeSequence.OuttakeSetup();
    }


    @Override
    public void runOpMode() {

        // this is basically init, all setup, hardware classes etc get initialized here
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap); // this might be clashing with initializing 2 of the same thing at once
        outtakeSequence.OuttakeHardware();

        // waits for user to press start on driverhub
        waitForStart();
        if (opModeIsActive()) {
            // runs setup function before main loop
            Setup();

            while (opModeIsActive()) {
                drivebase.Drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                // Main loop. Run class methods here to do stuff
                if(gamepad1.a){
                    outtake.liftTo(500, outtake.liftPos(), 1);
                }
                else if (gamepad1.b){
                    outtake.liftTo(100, outtake.liftPos(), 1);
                }
                if (gamepad1.x){
                    outtake.IntakeSlideTo(0, outtake.IntakeSlidePos(), 1);
                } else if (gamepad1.y){
                    outtake.IntakeSlideTo(500, outtake.IntakeSlidePos(), 1);
                }
                if (gamepad1.dpad_up){
                    outtake.turretSpin(0, outtake.turretPos(), 1);
                } else if (gamepad1.dpad_right){
                    outtake.turretSpin(20, outtake.turretPos(), 1);
                }else if (gamepad1.dpad_left){
                    outtake.turretSpin(-20, outtake.turretPos(), 1);
                }
                telemetry.addData("lift position", outtake.liftPos());
                telemetry.addData("lift pid output", outtake.returnPIDLiftOutput());
                telemetry.addData("intake slide position", outtake.IntakeSlidePos());
                telemetry.addData("intake slide pid output", outtake.returnPIDIntakeSlideOutput());
                telemetry.addData("turret position", outtake.turretPos());
                telemetry.addData("turret pid output", outtake.returnPIDTurretOutput());
                telemetry.update();
            }
        }
    }
}

