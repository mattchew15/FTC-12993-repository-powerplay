package org.firstinspires.ftc.teamcode.Sandstorm.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Sandstorm.DriveBase;
import org.firstinspires.ftc.teamcode.Sandstorm.GlobalsFarHighAuto;
import org.firstinspires.ftc.teamcode.Sandstorm.Inputs;
import org.firstinspires.ftc.teamcode.Sandstorm.Outtake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;


@Autonomous(name = "Left 1+5 Far High Auto", group = "Autonomous")
public class FIVE_FAR_HIGH_LEFT extends LinearOpMode {
    GlobalsFarHighAuto globalsFarHighAuto = new GlobalsFarHighAuto();
    int SideMultiplier = -1; // this multiplies everything that changes with the right side
    double AngleOffset = Math.toRadians(180); // this adds to every angle
    String webcamname = globalsFarHighAuto.WebCamRightName; // this is the webcam name for the right or left

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    boolean OtherSide;

    int numCycles;
    int SignalRotation;

    double correctedHeading;
    double xPosition;
    double yPosition;
    double headingPosition;

    // create class instances
    Outtake outtake = new Outtake();
    DriveBase drivebase = new DriveBase();
    Inputs inputs = new Inputs();
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
        OUT_AFTER_PRELOAD_DRIVE,
        OUTTAKE_CONE_AFTER_PRELOAD,
        OUTTAKE_CONE,
        DROP,
        GRAB_OFF_STACK,
        AFTER_GRAB_OFF_STACK,
        TRANSFER_CONE,
        OUTTAKE_CONE_NO_INTAKE_SLIDES,
        RETRACT_SLIDES,
        DRIVE_OTHER_SIDE_AND_TRANSFER,
        TURN_OTHER_STACK,
        PARK,
        IDLE,
    }

    // We define the current state we're on
    // Default to IDLE
    AutoState currentState;

    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        outtake.hardwareSetup();
        drivebase.motorsSetup();
        inputs.inputsSetup(); // hopefully won't conflict
        currentState = AutoState.PRELOAD_DRIVE; // this go here?
        autoTimer = 0;
        numCycles = 0;
        outtake.encodersReset();
        OtherSide = false;
    }
    // Define our start pose

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamname), cameraMonitorViewId);
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
        outtake.Outtake_init(hardwareMap);
        drivebase.Drivebase_init(hardwareMap); // this might conflict with road runner
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // road drive class

        // out cone stack position
        Pose2d startPose = new Pose2d(GlobalsFarHighAuto.startPoseX*SideMultiplier, GlobalsFarHighAuto.startPoseY, GlobalsFarHighAuto.startPoseAngle*SideMultiplier+AngleOffset);
        Pose2d OutConePose = new Pose2d(GlobalsFarHighAuto.outconestackX*SideMultiplier, GlobalsFarHighAuto.outconestackY, GlobalsFarHighAuto.outconeStackRotation*SideMultiplier+AngleOffset);

        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(GlobalsFarHighAuto.PreloadDriveX*SideMultiplier, GlobalsFarHighAuto.PreloadDriveY, GlobalsFarHighAuto.PreloadDriveRotation*SideMultiplier+AngleOffset))
                .build();

        Trajectory DriveOutStackAfterPreload = drive.trajectoryBuilder(PreloadDrive.end()) // actual drive out will occur during loop
                .lineToLinearHeading(new Pose2d(GlobalsFarHighAuto.outconestackX*SideMultiplier, GlobalsFarHighAuto.outconestackY, GlobalsFarHighAuto.outconeStackRotation*SideMultiplier+AngleOffset), SampleMecanumDrive.getVelocityConstraint(GlobalsFarHighAuto.slowerVelocityConstraintOut, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory DriveIntoStack = drive.trajectoryBuilder(DriveOutStackAfterPreload.end()) //
                .lineToLinearHeading(new Pose2d(GlobalsFarHighAuto.inconestackX*SideMultiplier,  GlobalsFarHighAuto.inconestackY, GlobalsFarHighAuto.inStackRotation*SideMultiplier+AngleOffset), SampleMecanumDrive.getVelocityConstraint(GlobalsFarHighAuto.slowerVelocityConstraintIn, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(OutConePose)
                .lineTo(new Vector2d(SideMultiplier == 1 ? -GlobalsFarHighAuto.parkLeft: GlobalsFarHighAuto.parkRight,GlobalsFarHighAuto.PreloadDriveY))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(OutConePose)
                .lineTo(new Vector2d(SideMultiplier == 1 ? -GlobalsFarHighAuto.parkRight: GlobalsFarHighAuto.parkLeft,GlobalsFarHighAuto.PreloadDriveY))
                .build();

        Trajectory ParkCentre = drive.trajectoryBuilder(OutConePose)
                .lineTo(new Vector2d(SideMultiplier == 1?-GlobalsFarHighAuto.parkCentre * SideMultiplier:GlobalsFarHighAuto.parkCentre,GlobalsFarHighAuto.PreloadDriveY))
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
            sleep(20); // idk why
            outtake.OuttakeClawClose();
            outtake.IntakeClipOpen();
            outtake.OuttakeArmReady();
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
        camera.stopStreaming(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry

            xPosition = poseEstimate.getX();
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();
            correctedHeading = inputs.angleWrap(headingPosition);
            outtake.outtakeReads();

            telemetry.addData("OtherSide", OtherSide);
            telemetry.addData("x", xPosition);
            telemetry.addData("y", yPosition);
            telemetry.addData("heading", headingPosition);
            telemetry.addData("corrected heading", correctedHeading);

            telemetry.addData("autostate", currentState);
            telemetry.addData("Intake Slide Position", outtake.intakeSlidePosition);
            telemetry.addData("Intake Slide Target Reached", outtake.intakeSlideTargetReachedSmallerThreshold());

            telemetry.addData("liftPosition", outtake.liftPosition);
            telemetry.addData("lift target reached", outtake.liftTargetReached());

            telemetry.addData("XError", drivebase.getXError());
            telemetry.addData("YError", drivebase.getYError());
            telemetry.addData("HeadingError", drivebase.getHeadingError());

            telemetry.addData("XOutput", drivebase.getXOutput());
            telemetry.addData("YOutput", drivebase.getYOutput());
            telemetry.addData("HeadingOutput", drivebase.getHeadingOutput());

            telemetry.addData("number of cycles:", numCycles);
            outtake.IntakeClipOpen();
            outtake.ConeArmReady();

            // main switch statement logic
            switch (currentState) {
                case PRELOAD_DRIVE:
                    outtake.IntakeSlideInternalPID(0,1); // might break something
                    outtake.liftToInternalPID(0, 1);
                    outtake.turretSpinInternalPID(0,1);
                    outtake.OuttakeSlideReady();
                    outtake.OuttakeClawClose();
                    outtake.OuttakeArmUpright();
                    outtake.IntakeClawOpen();
                    outtake.IntakeLift5();
                    if (!drive.isBusy()){
                        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                        currentState = AutoState.OUT_AFTER_PRELOAD_DRIVE;
                        drive.followTrajectoryAsync(DriveOutStackAfterPreload);
                        outtake.OuttakeArmUpright();
                    }
                    break;

                case OUT_AFTER_PRELOAD_DRIVE:
                    holdTurretPosition();
                    if (!drive.isBusy()){
                        currentState = AutoState.OUTTAKE_CONE;
                    }
                    break;

                case OUTTAKE_CONE:
                    OuttakeCone(true); // next state is drop
                    break;
                case DROP:
                    outtake.IntakeSlideTo(GlobalsFarHighAuto.IntakeSlideOutTicks, outtake.intakeSlidePosition, 1); // move to just before the stack
                    dropCone(300);
                    if (GlobalTimer.milliseconds() - autoTimer > 300){
                        currentState = AutoState.GRAB_OFF_STACK;
                        drive.followTrajectoryAsync(DriveIntoStack);
                        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    }
                    break;
                case GRAB_OFF_STACK:
                    if (GlobalTimer.milliseconds() - autoTimer > 350){ // doesn't hit the pole
                        outtake.OuttakeArmReady();
                        outtake.turretSpin(0, outtake.turretPosition,0.8);
                    }
                    outtake.BraceReady();
                    outtake.OuttakeSlideReady(); // drops down on pole a bit
                    outtake.liftToInternalPID(0,1);
                    outtake.IntakeSlideTo(GlobalsFarHighAuto.IntakeSlideOutTicks, outtake.intakeSlidePosition, 1); // slower
                    if (outtake.intakeClawTouchPressed() || !drive.isBusy()){
                        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                        currentState = AutoState.AFTER_GRAB_OFF_STACK;
                        Trajectory OutConeStack = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(GlobalsFarHighAuto.outconestackX*SideMultiplier,GlobalsFarHighAuto.outconestackY, GlobalsFarHighAuto.outconeStackRotation*SideMultiplier+AngleOffset), SampleMecanumDrive.getVelocityConstraint(GlobalsFarHighAuto.slowerVelocityConstraintOut, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        drive.followTrajectoryAsync(OutConeStack); // move backwards instantly - might not work
                    }
                    break;
                case AFTER_GRAB_OFF_STACK: // grabs off the stack while driving backwards
                    outtake.BraceReady();
                    outtake.OuttakeClawOpen();
                    outtake.OuttakeArmReady();
                    outtake.turretSpinInternalPID(0,1);
                    outtake.liftToInternalPID(2, 1);
                    outtake.turretSpin(0, outtake.turretPosition,0.8);
                    if (GlobalTimer.milliseconds() - autoTimer > 0){
                        outtake.IntakeClawClose();
                        if (GlobalTimer.milliseconds() - autoTimer > 120){
                            outtake.IntakeArmConeHoldForTransfer();
                            if (outtake.intakeLiftPosition > 275){
                                outtake.IntakeArmTransfer();
                                if ((outtake.getIntakeArmPos() > 137)){ // this reads the position of the intake arm
                                    outtake.IntakeSlideInternalPID(3,1);
                                    if (outtake.intakeSlidePosition > -400){
                                        outtake.IntakeLiftTransfer();
                                        if (outtake.intakeSlidePosition > -5){ // this controls when the claw closes
                                            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                                            currentState = AutoState.TRANSFER_CONE;
                                            outtake.OuttakeClawClose();
                                            outtake.BraceActive();
                                        }
                                    }
                                }
                            } else {
                                outtake.IntakeLift5();
                            }
                        }
                    }
                    break;
                case TRANSFER_CONE:
                    outtake.IntakeSlideInternalPID(4,1); // so it holds in when transferring
                    outtake.turretSpinInternalPID(0,1); // spin turret after
                    outtake.liftToInternalPID(2, 1);
                    outtake.OuttakeClawClose();
                    outtake.BraceActive();
                    if (GlobalTimer.milliseconds()-autoTimer > 90){
                        outtake.IntakeClawOpenHard();
                        if (GlobalTimer.milliseconds()-autoTimer > 130){
                            if (xPosition < GlobalsFarHighAuto.xValueBeforeSlidesExtend){ // kinda like a temporal marker
                                if (numCycles == 5){
                                    outtake.IntakeArmReady();
                                    outtake.IntakeLift3();
                                    autoTimer = GlobalTimer.milliseconds(); // reset timer
                                    currentState = AutoState.OUTTAKE_CONE_NO_INTAKE_SLIDES;
                                } else{
                                    outtake.IntakeArmReady();
                                    autoTimer = GlobalTimer.milliseconds(); // reset timer
                                    currentState = AutoState.OUTTAKE_CONE;
                                }
                            }
                        }
                    }
                    break;
                case OUTTAKE_CONE_NO_INTAKE_SLIDES:
                    OuttakeCone(false);
                    break;
                case RETRACT_SLIDES:
                    dropCone(350);
                    if (GlobalTimer.milliseconds() - autoTimer > 350){
                        outtake.liftToInternalPID(GlobalsFarHighAuto.LiftHighPosition, 1);
                        holdTurretPosition();
                    }
                    if (GlobalTimer.milliseconds() - autoTimer > 650){
                        currentState = AutoState.PARK;
                    }

                    break;


                case PARK:
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
                    outtake.turretSpin(0,outtake.turretPosition,1); // spin turret after
                    outtake.liftToInternalPID(0, 1);
                    outtake.IntakeSlideInternalPID(1,1);
                    outtake.turretSpinInternalPID(0,1);
                    outtake.OuttakeArmReady();
                    break;

            }
            // Updates driving for trajectories
            drive.update();
            telemetry.update();
        }

    }
    public void OuttakeCone(boolean Intake){
        if (Intake){ // if parameter is set to false it won't do the intake slides just outtake
            outtake.IntakeSlideInternalPID(GlobalsFarHighAuto.IntakeSlideOutTicks, 1); // move to just before the stack
            intakeLiftHeight();
            outtake.IntakeClawOpenHard();
            outtake.IntakeArmReady();
        } else {
            outtake.IntakeSlideInternalPID(0,1); // move to just before the stack
        }

        outtake.liftToInternalPID(GlobalsFarHighAuto.LiftHighPosition10, 1); // this shouldn't affect anything only make it go faster
        outtake.OuttakeClawClose();
        outtake.OuttakeArmScoreAuto();
        outtake.BraceActiveAuto();

        holdTurretPosition();

        if (outtake.liftPosition < GlobalsFarHighAuto.LiftHighPosition+10){
            if (!Intake){ // on the last one
                autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                currentState = AutoState.RETRACT_SLIDES;
                outtake.OuttakeArmScoreAuto();
                outtake.BraceActiveAuto();
                numCycles += 1;
            } else {
                autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                currentState = AutoState.DROP;
                outtake.OuttakeArmScoreAuto();
                outtake.BraceActiveAuto();
                numCycles += 1;
            }
        }
    }
    public void dropCone(int waitBeforeRetract){
        if (GlobalTimer.milliseconds() - autoTimer > 200){ // small wait
            if (GlobalTimer.milliseconds() - autoTimer > 280){
                outtake.OuttakeClawOpenHard();
                if (GlobalTimer.milliseconds() - autoTimer > waitBeforeRetract){
                    outtake.BraceReadyAuto(); // might need a new position for this
                    outtake.liftToInternalPID(2, 1);
                    if (GlobalTimer.milliseconds() - autoTimer > 400){
                        outtake.OuttakeSlideReady(); // drops down on pole a bit
                    } else {
                        holdTurretPosition();
                        outtake.liftToInternalPID(GlobalsFarHighAuto.LiftHighPosition,1);
                    }
                } else {
                    outtake.liftToInternalPID(GlobalsFarHighAuto.LiftHighPosition,1);
                }
            } else {
                outtake.OuttakeSlideScoreDrop(); // drops down on pole a bit
                outtake.OuttakeArmDeposit();
                outtake.liftToInternalPID(GlobalsFarHighAuto.LiftHighPosition,1);
            }
        } else {
            outtake.liftToInternalPID(GlobalsFarHighAuto.LiftHighPosition,1);
        }
    }

    public void intakeLiftHeight(){
        if (numCycles == 0){
            outtake.IntakeLift5();
        } else if (numCycles == 1){
            outtake.IntakeLift4();
        } else if (numCycles == 2){
            outtake.IntakeLift3();
        } else if (numCycles == 3){
            outtake.IntakeLift2();
        } else if (numCycles == 4){
            outtake.IntakeLift1();
        } else if (numCycles == 5){
            outtake.IntakeLiftReady();
        }
    }
    public void holdTurretPosition(){
        outtake.turretSpin(GlobalsFarHighAuto.TurretRightposition * SideMultiplier,outtake.turretPosition,1);
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


