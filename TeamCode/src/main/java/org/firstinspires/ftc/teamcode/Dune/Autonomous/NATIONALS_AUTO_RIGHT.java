package org.firstinspires.ftc.teamcode.Dune.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Dune.TurretLift;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;


@Autonomous(name = "NATIONALS_AUTO_RIGHT", group = "Autonomous")
public class NATIONALS_AUTO_RIGHT extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;
    final int liftHighPosition = 850;
    final int liftMidPosition = 815;
    int heightChange;
    boolean outakeResetReady;
    boolean outakeOutReady;
    boolean linkageOutReady;
    int numCycles;
    int SignalRotation;
    int slowerVelocityConstraint;

    final double outconestackX = 40;
    final double outconestackY = -6.1;
    final double outconestackRotation = 0;


    // create class instances

    TurretLift turretlift = new TurretLift();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;


    enum AutoState {
        PRELOAD_DRIVE,
        WAIT_AFTER_DUMP_PREDLOAD,
        PRELOAD_INTO_STACK,
        DRIVE_INTO_STACK,
        WAIT_AFTER_GRAB_STACK,
        DRIVE_OUT_STACK,
        WAIT_AFTER_DUMP_MID,
        PARK,
        IDLE,
        WAIT_BEFORE_PARK
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
        linkageOutReady = false;
        numCycles = 0;
        heightChange = 190; // starting cone stack height
        slowerVelocityConstraint = 12;
    }
    // Define our start pose
    Pose2d startPose = new Pose2d(38, -69, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamLeft"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        // initialize hardware
        turretlift.TurretLift_init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // road drive class

        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(38, -18.8, Math.toRadians(outconestackRotation)))
                //.lineTo(new Vector2d(33, -15))
                //.splineTo(new Vector2d(35, -40), Math.toRadians(-90)) // spline to spline heading, first angle is target, second angle is target angle during path
                //.splineToSplineHeading(new Pose2d(35, -12, Math.toRadians(0)), Math.toRadians(-90)) // end effects shape of spline, first angle is the target heading
                .build();

        Trajectory IntoConeStackPreload = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(outconestackX,outconestackY))
                .build();


        Trajectory IntoConeStack = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)), true)
                .lineToLinearHeading(new Pose2d(55.5, -6.5, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)))
                .lineTo(new Vector2d(64,outconestackY))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)))
                .lineTo(new Vector2d(8,outconestackY))
                .build();

        Trajectory ParkCentre = drive.trajectoryBuilder(new Pose2d(outconestackX, outconestackY, Math.toRadians(outconestackRotation)))
                .lineTo(new Vector2d(35,outconestackY))
                .build();

        while (!isStarted()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
            turretlift.closeClaw();
            turretlift.linkageIn();
        }


        waitForStart();
        if (isStopRequested()) return;

        // open cv changes the state at the start depending on cone rotation
        // open cv vision if statements to change variable and display telemetry here
        if(tagOfInterest == null || tagOfInterest.id == 1) {
            telemetry.addLine("Rotation Left");
            SignalRotation = 1;
        }
        else if(tagOfInterest.id == 2) {
            telemetry.addLine("Rotation Centre");
            SignalRotation = 2;
        }
        else {
            telemetry.addLine("Rotation Right");
            SignalRotation = 3;
        }

        // runs instantly once
        drive.followTrajectoryAsync(PreloadDrive);
        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
        turretlift.closeClaw();

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot

            // Print pose to telemetry
           // telemetry.addData("x", poseEstimate.getX());
           // telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("STACK PICK UP HEIGHT", heightChange);
            telemetry.addData("lift position", turretlift.liftPos());
            telemetry.addData("autostate", currentState);
            telemetry.addData("number of cycles:", numCycles);
            telemetry.addData ("outtakeoutReady", outakeOutReady);
            telemetry.addData ("outtakeresetReady", outakeResetReady);

            // main switch statement logic
            switch (currentState) {
                case PRELOAD_DRIVE:
                    turretlift.closeClaw();
                    outakeOutReady(180,1,350, liftMidPosition); // get outake ready - do timer to make it later, putt hsi in a function
                    if (linkageOutReady){
                        turretlift.linkageOutQuarter();
                    }
                    if (!drive.isBusy() && outakeOutReady) { //
                        autoTimer = GlobalTimer.milliseconds();
                        currentState = AutoState.WAIT_AFTER_DUMP_PREDLOAD;
                        }
                     // this timer goes off at the start of the code
                    break;

                case WAIT_AFTER_DUMP_PREDLOAD:
                    if (GlobalTimer.milliseconds() - autoTimer > 450){ // wait after claw
                        turretlift.openClaw();
                        //turretlift.liftToInternalPID(liftMidPosition-100,1); // move the lift down when it drops
                        if (GlobalTimer.milliseconds() - autoTimer > 650){
                            turretlift.linkageIn();
                            if (GlobalTimer.milliseconds() - autoTimer > 700){ // could be faster
                                readyOutake();  // linkage goes in
                                drive.followTrajectoryAsync(IntoConeStackPreload);
                                currentState = AutoState.PRELOAD_INTO_STACK;
                            }
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
                        turretlift.closeClaw(); // should put road runner into idle mode so it doesn't keep moving here
                        Trajectory OutConeStack = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(new Vector2d(outconestackX,outconestackY), SampleMecanumDrive.getVelocityConstraint(slowerVelocityConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        drive.followTrajectoryAsync(OutConeStack); // move backwards instantly - might no work
                        telemetry.addLine("grab");
                        currentState = AutoState.WAIT_AFTER_GRAB_STACK;
                        autoTimer = GlobalTimer.milliseconds(); // ready to grab stack
                    }
                    break;

                case WAIT_AFTER_GRAB_STACK: // if it is the linkage extending first before it drives in, then need to do position feedback on the servo to hold at its current position
                    if (GlobalTimer.milliseconds() - autoTimer > 120){
                        //turretlift.linkageNearlyOut(); because its moving backwards this shoudn't be needed
                        turretlift.liftToInternalPID(liftMidPosition,1);
                        if (GlobalTimer.milliseconds() - autoTimer > 400){
                            turretlift.linkageIn();
                            if (GlobalTimer.milliseconds() - autoTimer > 460){ // this could be reduced
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
                    outakeOutReady(144,1,liftMidPosition, liftMidPosition); // what's faster, driving or outake - balance of both is best
                    if (linkageOutReady){
                        turretlift.linkageNearlyOut();
                    }

                    if (!drive.isBusy() && outakeOutReady){
                        if (numCycles == 4){ // controls how many cycles we do
                            currentState = AutoState.WAIT_BEFORE_PARK;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                            numCycles += 1;
                        }
                        else{
                            heightChange -= 32; // tune this changes how high it changes each time
                            numCycles += 1;
                            currentState = AutoState.WAIT_AFTER_DUMP_MID;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                        }
                    }
                    break;

                case WAIT_AFTER_DUMP_MID:
                    if (GlobalTimer.milliseconds() - autoTimer > 300){ // wait after claw
                        turretlift.openClaw();
                        if (GlobalTimer.milliseconds() - autoTimer > 500){
                            turretlift.linkageIn();
                            if (GlobalTimer.milliseconds() - autoTimer > 600){ // could be faster
                                currentState = AutoState.DRIVE_INTO_STACK;
                                autoTimer = GlobalTimer.milliseconds(); // reset timer
                                drive.followTrajectoryAsync(IntoConeStack);
                            }
                        }
                    }
                    break;

                case WAIT_BEFORE_PARK:
                    if (GlobalTimer.milliseconds() - autoTimer > 600){
                        turretlift.openClaw(); // last drop may be different
                        if (GlobalTimer.milliseconds() - autoTimer > 850){
                            currentState = AutoState.PARK;
                        }
                    }
                    break;

                case PARK:
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
            //turretlift.liftToInternalPID(heightChange,0.4); // could be faster
            turretlift.liftTo(heightChange,turretlift.liftPos(),1);
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
                //turretlift.tiltUp();
                linkageOutReady = true;
                if (turretlift.liftPos() > liftposition2 - 100) { // change this to get rid of stupid timer
                    outakeOutReady = true;
                    telemetry.addLine("lift is up");
                }

            } else {
                turretlift.closeClaw(); //whats happening here is that it is
                turretlift.linkageIn(); // making sure stuff stays in when turret is turning
                linkageOutReady = false;
            }
        }
        else{
            turretlift.liftToInternalPID(liftposition, 1);
            turretlift.closeClaw();
            turretlift.linkageIn(); // might cause trouble
            //turretlift.tiltUpHalf();
            outakeOutReady = false;
            linkageOutReady = false;
        }
    }
    public void outakeIdle(){
        turretlift.turretSpinInternalPID(0, 1);
        turretlift.linkageIn();
        turretlift.tiltReset();
        turretlift.closeClawHard();
        if (turretlift.liftPos() < 400){
            turretlift.liftToInternalPID(0,0.6); // could be faster
            turretlift.closeClawHard();
            turretlift.linkageIn();
            if (turretlift.liftTargetReachedInternalPID()){
                outakeResetReady = true; // need a false here if (tusrretlift.liftTargetReachedInternalPID())
            }
        }else{
            outakeResetReady = false;
            turretlift.liftToInternalPID(350,0.5);
        }
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}


