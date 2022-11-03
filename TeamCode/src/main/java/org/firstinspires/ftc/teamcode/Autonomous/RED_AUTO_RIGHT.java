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
    double autoTimer;
    final int liftHighPosition = 850;
    final int liftMidPosition = 650;
    int heightChangeInterval;
    boolean outakeResetReady;
    boolean outakeOutReady;
    int numCycles;
    int SignalRotation;
    int slowerVelocityConstraint;

    // create class instances

    //DriveBase drivebase = new DriveBase(); // hardware classes
    TurretLift turretlift = new TurretLift();
    //Inputs inputs = new Inputs();
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1"; // what our webcam is called in hardware class

    enum AutoState {
        PRELOAD_DRIVE,
        PRELOAD_DROP,
        DRIVE_INTO_STACK,
        WAIT_AFTER_GRAB_STACK,
        DRIVE_OUT_STACK,
        WAIT_AFTER_DUMP_MID,
        PARK,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    AutoState currentState = AutoState.IDLE;


    // Initialize SampleMecanumDrive

    // function runs on init
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        //inputs.resetMatchTimer();
        //drivebase.motorsSetup();
        turretlift.motorsSetup();

        currentState = AutoState.PRELOAD_DRIVE; // this go here?
        autoTimer = 0;
        outakeResetReady = true;
        outakeOutReady = false;
        numCycles = 0;
        heightChangeInterval = 0;
        slowerVelocityConstraint = 20;
    }
    // Define our start pose
    Pose2d startPose = new Pose2d(35, -63, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        //drivebase.Drivebase_init(hardwareMap);
        turretlift.TurretLift_init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // road drive class

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });


        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here
        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading(new Pose2d(37, -15, Math.toRadians(4)))
                //.splineTo(new Vector2d(35, -40), Math.toRadians(-90)) // spline to spline heading, first angle is target, second angle is target angle during path
                //.splineToSplineHeading(new Pose2d(35, -12, Math.toRadians(0)), Math.toRadians(-90)) // end effects shape of spline, first angle is the target heading
                .build();

        Trajectory IntoConeStackPreload = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(50,-4.5), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory OutConeStackAfterPreload = drive.trajectoryBuilder(IntoConeStackPreload.end())
                .lineTo(new Vector2d(36,-6), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        Trajectory OutConeStack = drive.trajectoryBuilder(new Pose2d(50, -7, Math.toRadians(0)))
                .lineTo(new Vector2d(36,-6), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory IntoConeStack = drive.trajectoryBuilder(OutConeStack.end())
                .lineToLinearHeading(new Pose2d(50.5, -8, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(OutConeStack.end())
                .lineTo(new Vector2d(60,-6))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(OutConeStack.end())
                .lineTo(new Vector2d(5,-6))
                .build();

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.addData("Yellow Percent:", sleeveDetection.yelPercent);
            telemetry.addData("Blue Percent:", sleeveDetection.bluPercent);
            telemetry.addData("Red Percent:", sleeveDetection.redPercent);
            telemetry.update();
            turretlift.closeClaw();
        }


        waitForStart();
        if (isStopRequested()) return;

        // open cv changes the state at the start depending on cone rotation
        // open cv vision if statements to change variable and display telemetry here
        if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.LEFT) {
            telemetry.addLine("Rotation Left");
            SignalRotation = 1;

        } else if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addLine("Rotation Right");
            SignalRotation = 3;
        } else {
            telemetry.addLine("Rotation Centre");
            SignalRotation = 2;

        }
        // runs instantly once
        drive.followTrajectoryAsync(PreloadDrive);
        autoTimer = GlobalTimer.milliseconds(); // reset timer
        turretlift.closeClaw();

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heightChangeInterval", heightChangeInterval);
            telemetry.addData("autostate", currentState);
            telemetry.addData("number of cycles:", numCycles);
            telemetry.addData ("outtakeoutReady", outakeOutReady);
            telemetry.addData ("outtakeresetReady", outakeResetReady);

            // main switch statement logic
            switch (currentState) {
                case PRELOAD_DRIVE:
                    turretlift.closeClaw();
                    outakeOutReady(-130,1,350, liftHighPosition); // get outake ready - do timer to make it later, putt hsi in a function
                    if (!drive.isBusy() && outakeOutReady) { //
                        if (GlobalTimer.milliseconds() - autoTimer > 0){
                            turretlift.openClaw(); // preload drop
                            telemetry.addLine("PRELOAD DROP!!");
                            autoTimer = GlobalTimer.milliseconds();
                            drive.followTrajectoryAsync(IntoConeStackPreload);
                            currentState = AutoState.PRELOAD_DROP;
                            }
                        } // this timer goes off at the start of the code
                    break;

                case PRELOAD_DROP:
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        readyOutake();  // linkage goes in
                        if (!drive.isBusy()){
                            telemetry.addLine("drive is finished first stack intake ready");
                            if (GlobalTimer.milliseconds() - autoTimer > 2000) {
                                telemetry.addLine("grab");
                                turretlift.closeClaw();
                                autoTimer = GlobalTimer.milliseconds(); // ready to grab stack
                                currentState = AutoState.WAIT_AFTER_GRAB_STACK;
                            }
                        }
                    }
                    else {
                        turretlift.liftToInternalPID(600,1); // move the lift down when it drops
                    }
                    break;

                case WAIT_AFTER_GRAB_STACK:
                    if (GlobalTimer.milliseconds() - autoTimer > 700){
                        turretlift.linkageNearlyOut();
                        turretlift.liftToInternalPID(liftMidPosition,1);
                        if (GlobalTimer.milliseconds() - autoTimer > 800){
                            turretlift.linkageIn();
                            if (GlobalTimer.milliseconds() - autoTimer > 1300){ // timer to let the linkage extend
                                if (numCycles == 0){
                                    drive.followTrajectoryAsync(OutConeStackAfterPreload);
                                    currentState = AutoState.DRIVE_OUT_STACK;
                                    autoTimer = GlobalTimer.milliseconds(); // reset timer
                                }
                                else{
                                    drive.followTrajectoryAsync(OutConeStack);
                                    currentState = AutoState.DRIVE_OUT_STACK;
                                    autoTimer = GlobalTimer.milliseconds(); // reset timer
                                }
                            }
                            }
                        }
                    break;

                case DRIVE_OUT_STACK:
                    outakeOutReady(130,1,liftMidPosition, liftMidPosition);

                    if (!drive.isBusy() && outakeOutReady){
                        turretlift.openClaw();
                        if (numCycles == 3){
                            currentState = AutoState.PARK;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                            numCycles += 1;
                        }
                        else{
                            heightChangeInterval += 26; // tune this changes how high it changes each time
                            numCycles += 1;
                            currentState = AutoState.WAIT_AFTER_DUMP_MID;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                        }
                    }
                    break;

                case WAIT_AFTER_DUMP_MID:
                    turretlift.liftToInternalPID(liftMidPosition - 300, 1);
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        turretlift.linkageIn();
                        if (GlobalTimer.milliseconds() - autoTimer > 900){
                            currentState = AutoState.DRIVE_INTO_STACK;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                            drive.followTrajectoryAsync(IntoConeStack);
                        }
                    }
                    break;

                case DRIVE_INTO_STACK:
                        readyOutake();
                        if (!drive.isBusy() && outakeResetReady){
                            if (GlobalTimer.milliseconds() - autoTimer > 2000) {
                                telemetry.addLine("grab");
                                turretlift.closeClaw();
                                currentState = AutoState.WAIT_AFTER_GRAB_STACK;
                                autoTimer = GlobalTimer.milliseconds(); // ready to grab stack
                            }
                        }
                    break;

                case PARK:
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        readyOutake();
                        if (SignalRotation == 1){
                            drive.followTrajectoryAsync(ParkLeft);
                            currentState = AutoState.IDLE;
                        }
                        else if (SignalRotation == 3){
                            drive.followTrajectoryAsync(ParkRight);
                            currentState = AutoState.IDLE;
                        }
                        else{
                            currentState = AutoState.IDLE; // doesn't have to drive anywhere, already in position hopefully
                        }
                    }
                    break;

                case IDLE:
                    telemetry.addLine("WWWWWWWWWWW");
                    outakeIdle();
                    break;

            }
            // Updates driving for trajectories
            drive.update();
            telemetry.update();
        }

    }

    public void readyOutake(){
        turretlift.turretSpinInternalPID(0, 1);
        turretlift.readyServos();
        if (turretlift.liftPos() < 400){
            turretlift.liftToInternalPID(145 - heightChangeInterval,0.5); // could be faster
            turretlift.openClawHard();
            if (turretlift.liftTargetReachedInternalPID()){
                turretlift.linkageOut();
                outakeResetReady = true; // need a false here if (turretlift.liftTargetReachedInternalPID())
            }
        }else{
            outakeResetReady = false;
            turretlift.liftToInternalPID(350,0.5);
        }
    }

    public void outakeOutReady(int turretPosition, int liftSpeed, int liftposition, int liftposition2){ // way to use timers here
        if (turretlift.liftPos() > 300){
            turretlift.turretSpinInternalPID((int)Math.round(turretlift.degreestoTicks(turretPosition)), 1); //
            if (turretlift.turretTargetReachedInteralPID()){
                turretlift.liftToInternalPID(liftposition2, 1);
                turretlift.tiltUp();
                turretlift.linkageOut();
                if (turretlift.liftPos() > liftposition2 - 100){
                    outakeOutReady = true;
                    telemetry.addLine("lift is up");
                }
            }
        }
        else{
            turretlift.liftToInternalPID(liftposition, 1);
            turretlift.closeClaw();
            //turretlift.linkageIn(); // might cause trouble
            turretlift.tiltUpHalf();
            outakeOutReady = false;
        }
    }
    public void outakeIdle(){
        turretlift.turretSpinInternalPID(0, 1);
        turretlift.readyServos();
        if (turretlift.liftPos() < 400){
            turretlift.liftToInternalPID(0,1); // could be faster
            turretlift.closeClaw();
            turretlift.linkageIn();
            if (turretlift.liftTargetReachedInternalPID()){
                outakeResetReady = true; // need a false here if (tusrretlift.liftTargetReachedInternalPID())
            }
        }else{
            outakeResetReady = false;
            turretlift.liftToInternalPID(350,0.5);
        }
    }
}


