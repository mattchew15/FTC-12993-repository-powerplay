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

    // states for asynchronus sequences
    enum OutakeState {
        READY, // ready for pickup
        INTAKE, // either normal, or bar up and down
        GRAB, // grip claw
        INITIAL_LIFT, // has to go over intake motors and rev hubs
        TURN_LIFT_TILT,
        LINKAGE, // linkage extends once nearly finished turning - stability
        FINE_ADJUST, // gamepad2 controls fine adjust
        DROP, // gamepad1.rightbumper
        RETURN // calculate the fastest way for everything to be moving simaltaneously
    }

    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        inputs.resetMatchTimer();
        drivebase.motorsSetup();
        turretlift.motorsSetup();
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
                //inputs.gamepadRumbleTimer();
                //drivebase.motorDirectionTest(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y);

                telemetry.update();

            }
        }
    }

    // sequence functions go here;

}