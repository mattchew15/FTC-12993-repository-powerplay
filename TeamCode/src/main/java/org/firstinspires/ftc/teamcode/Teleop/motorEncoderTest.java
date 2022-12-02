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


@TeleOp(name = "motorEncoderTest")
public class motorEncoderTest extends LinearOpMode {

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


    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        inputs.resetMatchTimer();
        drivebase.motorsSetup();
        turretlift.motorsSetup();

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
                if(gamepad1.a){
                    turretlift.liftTo(500, turretlift.liftPos(), 1);
                }
                else if (gamepad1.b){
                    turretlift.liftTo(100, turretlift.liftPos(), 1);
                }
                telemetry.addData("lift position", turretlift.liftPos());
                telemetry.addData("pid error", turretlift.returnPIDLiftError());
                telemetry.addData("pid output", turretlift.returnPIDLiftOutput());
                telemetry.addData("pid integral sum", turretlift.returnIntegralSum());
                telemetry.update();
            }
        }
    }
}

