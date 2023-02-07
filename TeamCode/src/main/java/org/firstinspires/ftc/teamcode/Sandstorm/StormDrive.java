package org.firstinspires.ftc.teamcode.Sandstorm;

// Old imports, some not needed
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "StormDrive")
public class StormDrive extends LinearOpMode {

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
                //drivebase.motorDirectionTest(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x,gamepad1.right_stick_y);
                drivebase.PowerToggle(gamepad1.x);
                //inputs.gamepadRumbleTimer();
                telemetry.addData("LiftMotorPosition", outtake.liftPos());
               // telemetry.addData("turretPosition", outtake.tickstoDegrees((int)Math.round(outtake.turretPos()))); // might be in the wrong degrees/other
                telemetry.addData("turret raw position", outtake.turretPos());
                telemetry.addData("lift motor current draw", outtake.getLiftVoltage());
                telemetry.addData("intakeSlideMotor", outtake.IntakeSlidePos());

                outtakeSequence.outtakeSequence(); // if gamepad things don't work here then need to pass them in as parameters of this function

                telemetry.addData("Main Outtake State", outtakeSequence.outtakeState);
                telemetry.addData("Intake Out State", outtakeSequence.outtakeState);
                telemetry.addData("Cone Deposit State", outtakeSequence.coneDepositState);
                telemetry.addData("Flip Cone State", outtakeSequence.flipConesState);
                telemetry.addData("Outtake Pickup state", outtakeSequence.outtakePickupState);

                telemetry.update();

            }
        }
    }
}

