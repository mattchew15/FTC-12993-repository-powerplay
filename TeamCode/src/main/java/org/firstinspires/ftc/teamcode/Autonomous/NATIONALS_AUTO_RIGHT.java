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




@Autonomous(name = "NATIONALS_AUTO_RIGHT")
public class NATIONALS_AUTO_RIGHT extends LinearOpMode {
    //:)

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;
    final int liftHighPosition = 850;
    final int liftMidPosition = 630;
    int heightChangeInterval;
    boolean outakeResetReady;
    boolean outakeOutReady;
    int numCycles;
    int SignalRotation;
    int slowerVelocityConstraint;
    final double outconestackX = 27;
    final double outconestackY = -9.5;
    final double outconestackRotation = 0;

    // create class instances

    //DriveBase drivebase = new DriveBase(); // hardware classes
    TurretLift turretlift = new TurretLift();
    //Inputs inputs = new Inputs();
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1"; // what our webcam is called in hardware class

    enum AutoState {
        PRELOAD_DRIVE,
        WAIT_AFTER_DUMP_PREDLOAD,
        PRELOAD_INTO_STACK,
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
    Pose2d startPose = new Pose2d(38, -69, Math.toRadians(0));

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


        // functions runs on startz
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30, -48, Math.toRadians(outconestackRotation)))
                //.lineTo(new Vector2d(33, -15))
                //.splineTo(new Vector2d(35, -40), Math.toRadians(-90)) // spline to spline heading, first angle is target, second angle is target angle during path
                //.splineToSplineHeading(new Pose2d(35, -12, Math.toRadians(0)), Math.toRadians(-90)) // end effects shape of spline, first angle is the target heading
                .build();

        Trajectory IntoConeStackPreload = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(outconestackX,outconestackY))
                .build();


        Trajectory IntoConeStack = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)), true)
                .lineToLinearHeading(new Pose2d(48.5, -9.2, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)))
                .lineTo(new Vector2d(62,-9))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)))
                .lineTo(new Vector2d(-1,-9))
                .build();

        Trajectory ParkCentre = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)))
                .lineTo(new Vector2d(28,-9))
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
        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
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
                    outakeOutReady(180,1,350, liftMidPosition); // get outake ready - do timer to make it later, putt hsi in a function
                    if (!drive.isBusy() && outakeOutReady) { //
                        turretlift.linkageOutHalf();
                        autoTimer = GlobalTimer.milliseconds();
                        currentState = AutoState.WAIT_AFTER_DUMP_PREDLOAD;
                        }
                     // this timer goes off at the start of the code
                    break;

                case WAIT_AFTER_DUMP_PREDLOAD:
                    if (GlobalTimer.milliseconds() - autoTimer > 200){ // wait until linkage is out
                        turretlift.openClaw(); // preload drop
                        turretlift.liftToInternalPID(liftMidPosition-100,1); // move the lift down when it drops
                        if (GlobalTimer.milliseconds() - autoTimer > 400){
                            readyOutake();  // linkage goes in
                            drive.followTrajectoryAsync(IntoConeStackPreload);
                            currentState = AutoState.PRELOAD_INTO_STACK;
                        }
                    }
                    break;

                case PRELOAD_INTO_STACK:
                    readyOutake();  // linkage goes in
                    if (!drive.isBusy()){ // unfortunately its not one smooth drive
                        drive.followTrajectoryAsync(IntoConeStack);
                        currentState = AutoState.DRIVE_INTO_STACK;
                    }
                    break;

                case DRIVE_INTO_STACK:
                    readyOutake();
                    if ((!drive.isBusy() && outakeResetReady) || (turretlift.intakeTouchPressed() && outakeResetReady)){ // if either the drive finished or touch sensor - see comment below
                        //drive.breakfollowing(); // if you want to pause before picking up this is where it would be implimented
                        Trajectory OutConeStack = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(new Vector2d(outconestackX,outconestackY), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        drive.followTrajectoryAsync(OutConeStack); // move backwards instantly - might no work
                        telemetry.addLine("grab");
                        turretlift.closeClaw(); // should put road runner into idle mode so it doesn't keep moving here
                        currentState = AutoState.WAIT_AFTER_GRAB_STACK;
                        autoTimer = GlobalTimer.milliseconds(); // ready to grab stack
                    }
                    break;

                case WAIT_AFTER_GRAB_STACK: // if it is the linkage extending first before it drives in, then need to do position feedback on the servo to hold at its current position
                    if (GlobalTimer.milliseconds() - autoTimer > 300){
                        //turretlift.linkageNearlyOut(); because its moving backwards this shoudn't be needed
                        turretlift.liftToInternalPID(liftMidPosition,1);
                        if (GlobalTimer.milliseconds() - autoTimer > 450){
                            turretlift.linkageIn();
                            if (GlobalTimer.milliseconds() - autoTimer > 500){ // this could be reduced
                                //Trajectory OutConeStack = drive.trajectoryBuilder(poseEstimate)
                                //        .lineTo(new Vector2d(outconestackX,outconestackY), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                //               SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                //        .build();

                                //drive.followTrajectoryAsync(OutConeStack);
                                currentState = AutoState.DRIVE_OUT_STACK;
                                autoTimer = GlobalTimer.milliseconds(); // reset timer
                            }
                        }
                    }
                    break;

                case DRIVE_OUT_STACK:
                    outakeOutReady(130,1,liftMidPosition, liftMidPosition); // what's faster, driving or outake - balance of both is best
                    if (!drive.isBusy() && outakeOutReady){
                        turretlift.openClaw();
                        if (numCycles == 4){ // controls how many cycles we do
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
                        if (GlobalTimer.milliseconds() - autoTimer > 500){ // could be faster
                            currentState = AutoState.DRIVE_INTO_STACK;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                            drive.followTrajectoryAsync(IntoConeStack);
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
                            drive.followTrajectoryAsync(ParkCentre);
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
        if (turretlift.turretTargetReachedInteralPID()){
            turretlift.liftToInternalPID(145 - heightChangeInterval,0.5); // could be faster
            turretlift.openClawHard();
            if (turretlift.liftTargetReachedInternalPID()){
                turretlift.linkageOut(); // should be driving to touch tthe cone, not extending to touch the cone
                outakeResetReady = true; // need a false here if (turretlift.liftTargetReachedInternalPID())
            }
            else {
                turretlift.linkageOutHalf();
            }
        }else{
            outakeResetReady = false;
            turretlift.liftToInternalPID(400,0.5);
            turretlift.readyServos();
        }
    }

    public void outakeOutReady(int turretPosition, int liftSpeed, int liftposition, int liftposition2){ // way to use timers here
        if (turretlift.liftPos() > 300){
            turretlift.turretSpinInternalPID((int)Math.round(turretlift.degreestoTicks(turretPosition)), 1); //
            if (turretlift.turretTargetReachedInteralPID()) {
                turretlift.liftToInternalPID(liftposition2, 1);
                turretlift.tiltUp();
                if (turretlift.liftPos() > liftposition2 - 100) { // change this to get rid of stupid timer
                    outakeOutReady = true;
                    telemetry.addLine("lift is up");
                }

            } else {
                turretlift.closeClaw(); //whats happening here is that it is
                turretlift.linkageIn(); // making sure stuff stays in when turret is turning
            }
        }
        else{
            turretlift.liftToInternalPID(liftposition, 1);
            turretlift.closeClaw();
            turretlift.linkageIn(); // might cause trouble
            turretlift.tiltUpHalf();
            outakeOutReady = false;
        }
    }
    public void outakeIdle(){
        turretlift.turretSpinInternalPID(0, 1);
        turretlift.readyServos();
        if (turretlift.liftPos() < 400){
            turretlift.liftToInternalPID(0,0.6); // could be faster
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


