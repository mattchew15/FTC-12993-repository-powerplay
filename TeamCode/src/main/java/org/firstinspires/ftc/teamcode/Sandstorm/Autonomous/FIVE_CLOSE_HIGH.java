package org.firstinspires.ftc.teamcode.Sandstorm.Autonomous;
import static org.firstinspires.ftc.teamcode.Sandstorm.GlobalsCloseHighAuto.outconestackY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Sandstorm.DriveBase;
import org.firstinspires.ftc.teamcode.Sandstorm.GlobalsCloseHighAuto;
import org.firstinspires.ftc.teamcode.Sandstorm.Inputs;
import org.firstinspires.ftc.teamcode.Sandstorm.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;


@Autonomous(name = "1+5 Close-High Auto", group = "Autonomous")
public class FIVE_CLOSE_HIGH extends LinearOpMode {

    GlobalsCloseHighAuto globalsCloseHighAuto = new GlobalsCloseHighAuto();
    int SideMultiplier = 1; // this multiplies everything that changes with the right side
    double AngleOffset = Math.toRadians(0); // this adds to every angle
    String webcamname = globalsCloseHighAuto.WebCamLeftName; // this is the webcam name for the right or left

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    boolean outakeResetReady;
    boolean outakeOutReady;

    int numCycles;
    int SignalRotation;
    int slowerVelocityConstraint;

    double correctedHeading;
    double xPosition;
    double yPosition;
    double headingPosition;
    double dt;
    double prev_time;

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
        OUTTAKE_CONE_AFTER_PRELOAD,
        OUTTAKE_CONE,
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
        slowerVelocityConstraint = 12;
        outtake.encodersReset();
    }
    // Define our start pose

    Pose2d startPose = new Pose2d(globalsCloseHighAuto.startPoseX * SideMultiplier, globalsCloseHighAuto.startPoseY, globalsCloseHighAuto.startPoseAngle + AngleOffset);

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot read or write to the same motor mor ethan once or it will issues multiple bulk reads
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //

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

        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(globalsCloseHighAuto.outconestackX*SideMultiplier, globalsCloseHighAuto.outconestackY, globalsCloseHighAuto.outconeStackRotation * SideMultiplier + AngleOffset))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(SideMultiplier == 1 ? -GlobalsCloseHighAuto.parkLeft: GlobalsCloseHighAuto.parkRight * SideMultiplier, outconestackY))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(SideMultiplier == 1 ? -GlobalsCloseHighAuto.parkRight: GlobalsCloseHighAuto.parkLeft * SideMultiplier, outconestackY))
                .build();

        Trajectory ParkCentre = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(-globalsCloseHighAuto.parkCentre * SideMultiplier, globalsCloseHighAuto.outconestackY))
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

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();
            telemetry.addData("Loop Time", dt);

            xPosition = poseEstimate.getX();
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();
            correctedHeading = inputs.angleWrap(headingPosition);
            outtake.outtakeReads();

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

            outtake.ConeArmReady();
            // main switch statement logic
            switch (currentState) {
                case PRELOAD_DRIVE:
                    outtake.IntakeSlideInternalPID(0,1); // might break something
                    outtake.liftToInternalPID(0,1);
                    outtake.turretSpinInternalPID(0,1);
                    outtake.OuttakeSlideReady();
                    outtake.OuttakeClawClose();
                    outtake.OuttakeArmUpright();
                    outtake.IntakeClawOpen();
                    outtake.IntakeLift5();
                    if (!drive.isBusy()){
                        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                        currentState = AutoState.OUTTAKE_CONE;
                        outtake.OuttakeArmScoreAuto();
                    }
                    break;

                case OUTTAKE_CONE:
                    holdDrivebasePosition();
                    if (GlobalTimer.milliseconds() - autoTimer > 0){
                        OuttakeCone(true); // next state is grab off
                    }
                    break;

                case GRAB_OFF_STACK:
                    holdDrivebasePosition(); // dropping cone
                    dropCone(300);
                    if (GlobalTimer.milliseconds() - autoTimer < 300){
                        outtake.liftToInternalPID(GlobalsCloseHighAuto.LiftHighPosition, 1);
                        holdTurretPosition();
                    }

                    if (GlobalTimer.milliseconds() - autoTimer > 300) { // time taken to drop cone
                        if (outtake.intakeClawTouchPressed() || GlobalTimer.milliseconds() - autoTimer > 600){ // outtake.intakeClawTouchPressed() || GlobalTimer.milliseconds() - autoTimer > 680
                            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                        } else {
                            outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideOutTicks, 0.7); // slower
                        }
                    } else {
                        outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideNotQuiteOutTicks, 1); // slower
                    }

                    break;
                case AFTER_GRAB_OFF_STACK: // grabs off the stack
                    outtake.OuttakeClawOpen();
                    outtake.OuttakeArmReady();
                    outtake.BraceReady();
                    holdDrivebasePosition();
                    outtake.turretSpinInternalPID(0,1);
                    outtake.liftToInternalPID(2, 1);
                    if (GlobalTimer.milliseconds() - autoTimer > 0){
                        outtake.IntakeClawClose();
                        if (GlobalTimer.milliseconds() - autoTimer > 200){
                            outtake.IntakeLiftTransfer();
                            if (GlobalTimer.milliseconds()-autoTimer > 230){
                                outtake.IntakeArmTransfer();
                                if ((outtake.getIntakeArmPos() > 150) && GlobalTimer.milliseconds()-autoTimer > 270){ // this reads the position of the intake arm
                                    outtake.IntakeSlideInternalPID(2,1);
                                    if (outtake.intakeSlidePosition > -2 && outtake.getIntakeArmPos() > 195){ // this controls when the claw closes
                                        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                                        currentState = AutoState.TRANSFER_CONE;
                                        outtake.OuttakeClawClose();
                                        outtake.BraceActive();
                                    }
                                } else {
                                    outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideBackFromStack, 0.4); // this pulls slides in while doing stuff
                                }
                            }
                        } else {
                            outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideOutTicks, 1); // slower
                        }
                    } else {
                        outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideOutTicks, 1); // slower
                    }
                    break;
                case TRANSFER_CONE:
                    holdDrivebasePosition();
                    outtake.IntakeSlideInternalPID(6,1); // so it holds in when transferring
                    outtake.turretSpinInternalPID(0,1); // spin turret after
                    outtake.liftToInternalPID(2, 1);
                    outtake.OuttakeClawClose();
                    if (GlobalTimer.milliseconds()-autoTimer > 100){ // time between claw transfers
                        outtake.IntakeClawOpenHard();
                        if (GlobalTimer.milliseconds()-autoTimer > 200){
                            if (numCycles == 5){
                                outtake.IntakeArmReady();
                                outtake.IntakeLift3();
                                autoTimer = GlobalTimer.milliseconds(); // reset timer
                                currentState = AutoState.OUTTAKE_CONE_NO_INTAKE_SLIDES;
                            } else{
                                outtake.IntakeArmReady();
                                outtake.IntakeLift3();
                                outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideNotQuiteOutTicks, outtake.intakeSlidePosition, 1);

                                autoTimer = GlobalTimer.milliseconds(); // reset timer
                                currentState = AutoState.OUTTAKE_CONE;
                            }
                        }
                    }
                    break;
                case OUTTAKE_CONE_NO_INTAKE_SLIDES:
                    OuttakeCone(false); // in a function so that i don't have to make 2 changes
                    holdDrivebasePosition();

                    break;
                case RETRACT_SLIDES:
                    holdDrivebasePosition();
                    dropCone(300);
                    if (GlobalTimer.milliseconds() - autoTimer < 300){
                        outtake.liftToInternalPID(GlobalsCloseHighAuto.LiftHighPosition, 1);
                        holdTurretPosition();
                    }
                    if (GlobalTimer.milliseconds()-autoTimer > 600){
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
                    outtake.OuttakeClawOpen();
                    outtake.OuttakeArmReady();
                    outtake.turretSpinInternalPID(0,1); // spin turret after
                    outtake.liftToInternalPID(-2, 1);
                    outtake.IntakeSlideInternalPID(3,1);
                    break;

            }
            // Updates driving for trajectories
            drive.update();
            telemetry.update();
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

    public void OuttakeCone(boolean Intake){
        if (Intake){ // if parameter is set to false it won't do the intake slides just outtake
            outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideNotQuiteOutTicks, 1); // move to just before the stack
            intakeLiftHeight();
            outtake.IntakeClawOpenHard();
            outtake.IntakeArmReady();
        } else {
            outtake.IntakeSlideInternalPID(0,1); // move to just before the stack
        }

        outtake.liftToInternalPID(globalsCloseHighAuto.LiftHighPosition, 1);
        outtake.OuttakeClawClose();
        outtake.OuttakeArmScoreAuto();
        outtake.BraceActiveAuto();

        holdTurretPosition();

        if (outtake.liftPosition < globalsCloseHighAuto.LiftHighPosition+10){
            if (!Intake){ // on the last one
                autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                currentState = AutoState.RETRACT_SLIDES;
                outtake.OuttakeArmScoreAuto();
                outtake.BraceActiveAuto();
                numCycles += 1;
            } else {
                autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                currentState = AutoState.GRAB_OFF_STACK;
                outtake.OuttakeArmScoreAuto();
                outtake.BraceActiveAuto();
                numCycles += 1;
            }
        }
    }
    public void dropCone(int waitBeforeRetract){
        holdDrivebasePosition(); // dropping cone
        if (GlobalTimer.milliseconds() - autoTimer > 120){ // small wait
            if (GlobalTimer.milliseconds() - autoTimer > 190){
                outtake.OuttakeClawOpenHard();
                if (GlobalTimer.milliseconds() - autoTimer > waitBeforeRetract){
                    outtake.BraceReadyAuto(); // might need a new position for this
                    outtake.liftToInternalPID(2, 1);
                    if (GlobalTimer.milliseconds() - autoTimer > 350){
                        outtake.turretSpinInternalPID(0, 0.8);
                        outtake.OuttakeSlideReady(); // drops down on pole a bit
                    } else {
                        holdTurretPosition();
                    }
                }
            } else {
                outtake.OuttakeSlideScoreDrop(); // drops down on pole a bit
                outtake.OuttakeArmDeposit();

            }
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
    public void holdDrivebasePosition(){ // THE OUTCONESTACKROTATION SHOULD BE NEGATIVE
        drivebase.DriveToPositionAutonomous(globalsCloseHighAuto.outconestackX * SideMultiplier, globalsCloseHighAuto.outconestackY,globalsCloseHighAuto.outconeStackRotation* SideMultiplier + AngleOffset,xPosition,yPosition,correctedHeading, 1,1); // last values are translationalspeed, and rotational speed
    }
    public void holdTurretPosition(){
        outtake.turretSpin(globalsCloseHighAuto.TurretLeftposition * SideMultiplier, outtake.turretPosition,1);
    }
}


