package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleop.DriveBase;
import org.firstinspires.ftc.teamcode.Teleop.DuneDrive;
import org.firstinspires.ftc.teamcode.Teleop.Inputs;
import org.firstinspires.ftc.teamcode.Teleop.TurretLift;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Queue;
import java.util.Vector;




@Autonomous(name = "RED_AUTO_RIGHT")
public class RED_AUTO_RIGHT extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    int stacknumber; // 5 for 5 cones, 4 for 4 cones etc.
    double autoTimer;
    int liftHighPosition = 850;
    boolean outakeReady;

    // create class instances
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // before was in init but can be here
    DriveBase drivebase = new DriveBase();
    TurretLift turretlift = new TurretLift();
    Inputs inputs = new Inputs();

    enum AutoState {
        PRELOAD_DRIVE,
        PRELOAD_DROP,
        DRIVE_INTO_STACK,
        DRIVE_OUT_STACK,
        PARK,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    AutoState currentState = AutoState.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(35, -63, Math.toRadians(90));
    // Initialize SampleMecanumDrive

    // functions runs on init
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        inputs.resetMatchTimer();
        drivebase.motorsSetup();
        turretlift.motorsSetup();

        currentState = AutoState.IDLE; // this go here?
        autoTimer = 0;
        outakeReady = true;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here
        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90))) // spline to spline heading, first angle is target, second angle is target angle during path
                .splineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)), Math.toRadians(90)) // end effects shape of spline, first angle is the target heading
                .build();

        Trajectory IntoConeStack = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(47,-12))
                .build();

        Trajectory OutConeStack = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(35,-12))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        // open cv changes the state at the start depending on cone rotation
        // open cv vision if statements to change variable and display telemetry here

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot


            // main switch statement logic
            switch (currentState) {
                case PRELOAD_DRIVE:
                    drive.followTrajectoryAsync(PreloadDrive);
                    // get outake ready - do timer to make it later, putt hsi in a function
                    outakeOutReady(160,1,350);
                    if (!drive.isBusy()) {
                        currentState = AutoState.PRELOAD_DROP;
                        autoTimer = GlobalTimer.milliseconds();
                        drive.followTrajectoryAsync(IntoConeStack);
                        turretlift.openClaw();
                    }
                    break;

                case PRELOAD_DROP:
                    if (GlobalTimer.milliseconds() - autoTimer > 300){
                        readyOutake();
                    }

                    }
                    else if (intaketype == 1 || intaketype == 2) { // second intake case
                        drivebase.intakeSpin(0.8);
                        drivebase.intakeBarDown();
                        intakesequencetransition();
                    }

                    break;

                case GRAB:
                    if (GlobalTimer.milliseconds() - outakesequencetimer > 0){ // this if statement is not needed at all
                        turretlift.closeClaw();
                        turretPositionChange(); // allows drivers to change the turret position in this state
                        if (GlobalTimer.milliseconds() - outakesequencetimer > 200){
                            turretlift.closeClaw(); // not needed, servo is already going to position?
                            turretlift.liftToInternalPID(72,turretlift.liftPos(),1);
                            if (turretlift.liftTargetReached()){
                                turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype),turretlift.turretPos(), 1);
                                turretlift.tiltUp();
                                if (turretlift.turretTargetReached()){
                                    outakestate = DuneDrive.OutakeState.HEIGHT_LINKAGE;
                                }
                            }
                        }
                    }
                    break;

                case HEIGHT_LINKAGE:
                    turretlift.closeClaw();
                    turretlift.liftToInternalPID( liftpositiontype , turretlift.liftPos(),1);
                    turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype),turretlift.turretPos(), 1);
                    turretlift.tiltUp();
                    turretlift.linkageOut();
                    liftPositionChange(); // change this to transition state for fine adjust!!! if you dont want fine adjust then dont do it
                    break;

                case DROP:
                    turretlift.liftToInternalPID( liftpositiontype , turretlift.liftPos(),1);
                    turretlift.turretSpinInternalPID((int)Math.round(turretpositiontype),turretlift.turretPos(), 1);
                    turretlift.tiltUp();
                    turretlift.linkageOut();
                    if (GlobalTimer.milliseconds() - outakesequencetimer > 300){
                        if (gamepad1.right_bumper){
                            turretlift.openClaw();
                            // outakesequencetimer = GlobalTimer.milliseconds(); //  reset timer
                            outakestate = DuneDrive.OutakeState.RETURN;
                        }
                    }

                    break;

                case RETURN:
                    turretlift.turretSpinInternalPID(0,turretlift.turretPos(), 1);
                    turretlift.readyServos();
                    if (turretlift.turretTargetReachedInteralPID()){
                        //telemetry.addData("turret return target reached?", true);
                        turretlift.liftToInternalPID(0, turretlift.liftPos(),1);
                        if (turretlift.liftTargetReachedInteralPID()){
                            turretlift.openClaw();
                            outakestate = DuneDrive.OutakeState.READY;
                        }
                    }

                    break;
            }
            // Updates driving for trajectories
            drive.update();
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
        }

    }

    public void readyOutake(){
        turretlift.turretSpinInternalPID(0, 1);
        turretlift.readyServos();
        drivebase.intakeSpin(-0.1);
        if (turretlift.turretTargetReachedInteralPID()){
            //telemetry.addData("turret return target reached?", true);
            telemetry.addLine("turret return target reached");
            turretlift.liftToInternalPID(0,0.5); // could be faster
            if (turretlift.liftTargetReachedInternalPID()){
                turretlift.openClaw();
                outakeReady = true; // need a false here
            }
        }
    }

    public void outakeOutReady(int turretPosition, int liftSpeed, int liftposition){ // way to use timers here
        if (turretlift.liftTargetReachedInternalPID()){
            telemetry.addLine("lift is up");
            turretlift.turretSpinInternalPID((int)Math.round(turretlift.degreestoTicks(turretPosition)), 1); //

        }
        else{
            drivebase.intakeSpin(0);
            turretlift.closeClaw();
            turretlift.linkageIn();
            drivebase.intakeBarDown();
            turretlift.liftToInternalPID(liftposition,liftSpeed);
            turretlift.tiltUpHalf();
            outakeReady = false;
        }
    }
}


