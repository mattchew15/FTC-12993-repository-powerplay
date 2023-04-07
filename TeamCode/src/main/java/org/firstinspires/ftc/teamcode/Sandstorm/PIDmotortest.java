package org.firstinspires.ftc.teamcode.Sandstorm;

// Old imports, some not needed
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Current;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.sql.Time;


@TeleOp(name = "PIDmotortest")
public class PIDmotortest extends LinearOpMode {

    final double xTarget = 0;
    final double yTarget = 0;
    final double headingTarget = Math.toRadians(180);

    //create new instances of class
    DriveBase drivebase = new DriveBase();
    //OuttakeSequence outtakeSequence = new OuttakeSequence();
    Inputs inputs = new Inputs();
    Outtake outtake = new Outtake();
    final double distanceFromPointThreshold = 2; // this would have to be changed

    double CurrentTime;
    double LoopTime;
    double PreviousTime;

    //Inputs inputs = new Inputs();

    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    ElapsedTime GlobalTimer;
    ElapsedTime LoopTimeTimer;

    // random setup function that runs once start is pressed but before main loop
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        LoopTimeTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        LoopTimeTimer.reset();
        drivebase.motorsSetup();
        outtake.hardwareSetup();
        inputs.inputsSetup();
        //outtakeSequence.OuttakeSetup();
    }


    @Override
    public void runOpMode() {

        // this is basically init, all setup, hardware classes etc get initialized here
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads - might not work??
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //
        drivebase.Drivebase_init(hardwareMap);
        outtake.Outtake_init(hardwareMap); // this might be clashing with initializing 2 of the same thing at once
        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap);
        outtake.encodersReset();

        // waits for user to press start on driverhub
        waitForStart();
        if (opModeIsActive()) {
            // runs setup function before main loop
            Setup();

            while (opModeIsActive()) {
                CurrentTime = LoopTimeTimer.nanoseconds();
                LoopTime = CurrentTime - PreviousTime; // previous loop time
                telemetry.addData("Loop Time", LoopTime/LoopTimeTimer.nanoseconds()); // should return loop time for previous loop
                // Main loop. Run class methods here to do stuff
                if(gamepad1.a){
                    outtake.liftTo(GlobalsCloseHighAuto.LiftHighPosition, outtake.liftPosition, 1);
                    outtake.OuttakeArmScoreAuto();
                }
                else if (gamepad1.b){
                    outtake.liftTo(0, outtake.liftPosition, 1);
                    outtake.OuttakeArmReady();
                }
                if (gamepad1.x){
                    outtake.IntakeSlideTo(0, outtake.intakeSlidePosition, 1);
                } else if (gamepad1.y){
                    outtake.IntakeSlideTo(GlobalsCloseHighAuto.IntakeSlideOutTicks, outtake.intakeSlidePosition, 1);
                }
                if (gamepad1.dpad_up){
                    outtake.turretSpin(0, outtake.turretPosition, 1);
                } else if (gamepad1.dpad_right){
                    outtake.turretSpin(GlobalsCloseHighAuto.TurretLeftposition, outtake.turretPosition, 1);
                }else if (gamepad1.dpad_left)
                    outtake.turretSpin(GlobalsCloseHighAuto.TurretRightposition, outtake.turretPosition, 1);
                if (gamepad1.right_bumper){
                    outtake.turretSpinInternalPID(-23, 1);
                } else if (gamepad1.left_bumper){
                    outtake.turretSpinInternalPID(0, 1);
                }

                if (gamepad1.right_stick_button){
                    outtake.IntakeSlideInternalPID(-500,1);
                } else if (gamepad1.left_stick_button){
                    outtake.IntakeSlideInternalPID(0, 1);
                }
                /*
                telemetry.addData("lift position", outtake.liftPosition);
                telemetry.addData("lift pid output", outtake.returnPIDLiftOutput());
                telemetry.addData("lift-reached-target?", outtake.liftTargetReached());
                telemetry.addData("intake slide position", outtake.intakeSlidePosition);
                telemetry.addData("intake slide pid output", outtake.returnPIDIntakeSlideOutput());
                telemetry.addData("intake-slide-reached-target?", outtake.intakeSlideTargetReached());
                telemetry.addData("turret position", outtake.tickstoDegrees((int)Math.round(outtake.turretPosition)));
                telemetry.addData("turret position", outtake.turretPosition);
                telemetry.addData("turret position", outtake.degreestoTicks(-20));
                telemetry.addData("turret pid output", outtake.returnPIDTurretOutput());
                telemetry.addData("turret-target-reached?", outtake.turretTargetReached());

                 */
                telemetry.addData("IntakeArmPosition", outtake.getIntakeArmPos());
                telemetry.addData("IntakeLiftPosition", outtake.getIntakeLiftPos());


                location.update();
                Pose2d poseEstimate = location.getPoseEstimate();

                double xPosition = poseEstimate.getX();
                double yPosition = poseEstimate.getY();
                double headingPosition = poseEstimate.getHeading();
                double correctedHeading = Angle.normDelta(headingPosition);

                outtake.outtakeReads();
                telemetry.addData("x", xPosition);
                telemetry.addData("y", yPosition);

                telemetry.addData("corrected heading", correctedHeading);

                telemetry.addData("XError", drivebase.getXError());
                telemetry.addData("YError", drivebase.getYError());
                telemetry.addData("HeadingError", drivebase.getHeadingError());

                telemetry.addData("XOutput", drivebase.getXOutput());
                telemetry.addData("YOutput", drivebase.getYOutput());
                telemetry.addData("HeadingOutput", drivebase.getHeadingOutput());

                driveToPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,correctedHeading, 1,1, headingPosition);

                telemetry.addData("Distance from target",drivebase.getDistanceFromPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,headingPosition));
                telemetry.addData("Automatic driving toggle", inputs.DriveToPositionToggleMode);

                telemetry.addData("liftPosition", outtake.liftPosition);
                telemetry.addData("turretposition", outtake.turretPosition);
                telemetry.addData("IntakeslidePosition", outtake.intakeSlidePosition);
                telemetry.addData("liftError", outtake.liftError());

                PreviousTime = CurrentTime;
                LoopTimeTimer.reset();
                telemetry.update();
                }
            }
        }
    public void driveToPosition(double xTarget, double yTarget, double headingTarget, double xPosition, double yPosition, double correctedHeading, double maxTranslationalSpeed, double maxRotationalSpeed, double rawHeading){
        inputs.driveToPositionToggle(gamepad1.dpad_down);

        // try a different angle wrap for holding drivebase position to the other side (offvset by 180 degrees if over a certain extent?) - if this works, integrate this into the one method to make autonomous easier

        if (inputs.DriveToPositionToggleMode){
            //drivebase.Drive(0,0,0); // this ensures there is no conflicting commands from the joysticks
            drivebase.DriveToPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,correctedHeading, maxTranslationalSpeed,maxRotationalSpeed); // last values are translationalspeed, and rotational speed
            if ((drivebase.getDistanceFromPosition(xTarget,yTarget,headingTarget,xPosition,yPosition,rawHeading) > distanceFromPointThreshold)){ // have to deal with the heading here, read telemetry for heading angle
               telemetry.addLine("POSITION REACHED");
            }
        } else {
            drivebase.Drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
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


