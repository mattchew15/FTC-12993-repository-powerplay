package org.firstinspires.ftc.teamcode.Sandstorm;

// Old imports, some not needed
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


@TeleOp(name = "PIDmotortest")
public class PIDmotortest extends LinearOpMode {

    final double xTarget = 0;
    final double yTarget = 0;
    final double headingTarget = Math.toRadians(-90);

    //create new instances of class
    DriveBase drivebase = new DriveBase();
    //OuttakeSequence outtakeSequence = new OuttakeSequence();
    Inputs inputs = new Inputs();
    Outtake outtake = new Outtake();
    final double distanceFromPointThreshold = 2; // this would have to be changed


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
        //outtakeSequence.OuttakeSetup();
    }


    @Override
    public void runOpMode() {

        // this is basically init, all setup, hardware classes etc get initialized here
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap); // this might be clashing with initializing 2 of the same thing at once
        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap);
       // outtakeSequence.OuttakeHardware();
        outtake.encodersReset();

        // waits for user to press start on driverhub
        waitForStart();
        if (opModeIsActive()) {
            // runs setup function before main loop
            Setup();

            while (opModeIsActive()) {
                drivebase.Drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

                // Main loop. Run class methods here to do stuff
                if(gamepad1.a){
                    outtake.liftTo(-500, outtake.liftPos(), 1);
                }
                else if (gamepad1.b){
                    outtake.liftTo(0, outtake.liftPos(), 1);
                }
                if (gamepad1.x){
                    outtake.IntakeSlideTo(0, outtake.IntakeSlidePos(), 1);
                } else if (gamepad1.y){
                    outtake.IntakeSlideTo(-700, outtake.IntakeSlidePos(), 1);
                }
                if (gamepad1.dpad_up){
                    outtake.turretSpin(0, outtake.turretPos(), 1);
                } else if (gamepad1.dpad_right){
                    outtake.turretSpin(8, outtake.turretPos(), 1);
                }else if (gamepad1.dpad_left)
                    outtake.turretSpin(-8, outtake.turretPos(), 1);
                if (gamepad1.right_bumper){
                    outtake.liftToInternalPID(-500, 1);
                } else if (gamepad1.left_bumper){
                    outtake.liftToInternalPID(0, 1);
                }

                if (gamepad1.right_stick_button){
                    outtake.IntakeSlideInternalPID(-700,1);
                } else if (gamepad1.left_stick_button){
                    outtake.IntakeSlideInternalPID(0, 1);
                }
                /*
                telemetry.addData("lift position", outtake.liftPos());
                telemetry.addData("lift pid output", outtake.returnPIDLiftOutput());
                telemetry.addData("lift-reached-target?", outtake.liftTargetReached());
                telemetry.addData("intake slide position", outtake.IntakeSlidePos());
                telemetry.addData("intake slide pid output", outtake.returnPIDIntakeSlideOutput());
                telemetry.addData("intake-slide-reached-target?", outtake.intakeSlideTargetReached());
                telemetry.addData("turret position", outtake.tickstoDegrees((int)Math.round(outtake.turretPos())));
                telemetry.addData("turret position", outtake.turretPos());
                telemetry.addData("turret position", outtake.degreestoTicks(-20));
                telemetry.addData("turret pid output", outtake.returnPIDTurretOutput());
                telemetry.addData("turret-target-reached?", outtake.turretTargetReached());

                telemetry.addData("IntakeArmPosition", outtake.getIntakeArmPos());
                telemetry.addData("OuttakeArmPosition", outtake.getOuttakeArmPos());
                telemetry.addData("IntakeLiftPosition", outtake.getIntakeLiftPos());

                 */
                location.update();
                Pose2d poseEstimate = location.getPoseEstimate();

                double xPosition = poseEstimate.getX();
                double yPosition = poseEstimate.getY();
                double headingPosition = poseEstimate.getHeading();

                telemetry.addData("x", xPosition);
                telemetry.addData("y", yPosition);
                telemetry.addData("heading", headingPosition);

                telemetry.addData("XError", drivebase.getXError());
                telemetry.addData("YError", drivebase.getYError());
                telemetry.addData("HeadingError", drivebase.getHeadingError());

                telemetry.addData("XOutput", drivebase.getXOutput());
                telemetry.addData("YOutput", drivebase.getYOutput());
                telemetry.addData("HeadingOutput", drivebase.getHeadingOutput());

                driveToPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,headingPosition, 1,1);

                telemetry.addData("Distance from target",drivebase.getDistanceFromPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,headingPosition));


                telemetry.update();
                }
            }
        }
    public void driveToPosition(double xTarget, double yTarget, double headingTarget, double xPosition, double yPosition, double headingPosition, double maxTranslationalSpeed, double maxRotationalSpeed){
        inputs.driveToPositionToggle(gamepad1.dpad_down);
        if (inputs.DriveToPositionToggleMode){
            drivebase.DriveToPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,headingPosition, maxTranslationalSpeed,maxTranslationalSpeed); // last values are translationalspeed, and rotational speed
            if ((drivebase.getDistanceFromPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,headingPosition) > distanceFromPointThreshold)){ // have to deal with the heading here, read telemetry for heading angle
               telemetry.addLine("POSITION REACHED");
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
    }


