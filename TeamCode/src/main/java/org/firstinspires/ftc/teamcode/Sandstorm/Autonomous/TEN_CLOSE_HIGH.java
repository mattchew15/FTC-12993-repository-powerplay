package org.firstinspires.ftc.teamcode.Sandstorm.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
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


@Autonomous(name = "1+10 Close-High Auto", group = "Autonomous")
public class TEN_CLOSE_HIGH extends LinearOpMode {

    GlobalsCloseHighAuto globalsCloseHighAuto = new GlobalsCloseHighAuto();
    int SideMultiplier = -1; // this multiplies everything that changes with the right side
    double AngleOffset = Math.toRadians(180); // this adds to every angle
    String webcamname = globalsCloseHighAuto.WebCamLeftName; // this is the webcam name for the right or left

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    boolean outakeResetReady;
    boolean outakeOutReady;
    boolean OtherSide;

    int numCycles;
    int SignalRotation;
    int slowerVelocityConstraint;

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
        OtherSide = false;
    }
    // Define our start pose

    Pose2d startPose = new Pose2d(globalsCloseHighAuto.startPoseX * SideMultiplier, globalsCloseHighAuto.startPoseY, globalsCloseHighAuto.startPoseAngle + AngleOffset);

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

        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(globalsCloseHighAuto.outconestackX, globalsCloseHighAuto.outconestackY, globalsCloseHighAuto.outconeStackRotation * SideMultiplier + AngleOffset))
                .build();

        Trajectory DriveOtherSide = drive.trajectoryBuilder(PreloadDrive.end())
                .lineToLinearHeading(new Pose2d(globalsCloseHighAuto.outconestackX, globalsCloseHighAuto.outconestackY, globalsCloseHighAuto.outconeStackRotation * SideMultiplier + AngleOffset))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(DriveOtherSide.end())
                .lineTo(new Vector2d(SideMultiplier == 1 ? globalsCloseHighAuto.parkRight: globalsCloseHighAuto.parkLeft * SideMultiplier, globalsCloseHighAuto.outconestackY))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(DriveOtherSide.end())
                .lineTo(new Vector2d(SideMultiplier == 1 ? globalsCloseHighAuto.parkLeft: globalsCloseHighAuto.parkRight * SideMultiplier, globalsCloseHighAuto.outconestackY))
                .build();

        Trajectory ParkCentre = drive.trajectoryBuilder(DriveOtherSide.end())
                .lineTo(new Vector2d(globalsCloseHighAuto.parkCentre, globalsCloseHighAuto.outconestackY))
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

            xPosition = poseEstimate.getX();
            yPosition = poseEstimate.getY();
            headingPosition = Angle.normDelta(poseEstimate.getHeading());
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


            // main switch statement logic
            switch (currentState) {
                case PRELOAD_DRIVE:
                    outtake.IntakeSlideInternalPID(0,1); // might break something
                    outtake.liftTo(0, outtake.liftPosition,1);
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
                    if (GlobalTimer.milliseconds() - autoTimer > 70){ // small wait
                        if (GlobalTimer.milliseconds() - autoTimer > 120){
                            outtake.OuttakeClawOpenHard();
                            if (GlobalTimer.milliseconds() - autoTimer > 135){
                                outtake.BraceReady(); // might need a new position for this
                                outtake.liftTo(2, outtake.liftPosition, 1);
                                if (GlobalTimer.milliseconds() - autoTimer > 250){
                                    outtake.turretSpin(0, outtake.turretPosition,0.8);
                                    outtake.OuttakeSlideReady(); // drops down on pole a bit
                                } else {
                                    holdTurretPosition();
                                }
                            } else {
                                outtake.liftTo(globalsCloseHighAuto.LiftHighPosition, outtake.liftPosition, 1);
                                holdTurretPosition();
                            }
                        } else {
                            outtake.OuttakeSlideScoreDrop(); // drops down on pole a bit
                            outtake.OuttakeArmDeposit();
                            outtake.liftTo(globalsCloseHighAuto.LiftHighPosition, outtake.liftPosition, 1);
                            holdTurretPosition();
                        }
                    } else {
                        outtake.liftTo(globalsCloseHighAuto.LiftHighPosition, outtake.liftPosition, 1);
                        holdTurretPosition();
                    }

                    if (GlobalTimer.milliseconds() - autoTimer > 300) { // time taken to drop cone
                        if (outtake.intakeClawTouchPressed() || GlobalTimer.milliseconds() - autoTimer > 680){
                            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                        } else {
                            outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideOutTicks, outtake.intakeSlidePosition, 0.5); // slower
                        }
                    } else {
                        outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideNotQuiteOutTicks, outtake.intakeSlidePosition, 0.8); // slower
                    }


                    break;
                case AFTER_GRAB_OFF_STACK: // grabs off the stack
                    outtake.OuttakeClawOpen();
                    outtake.OuttakeArmReady();
                    holdDrivebasePosition();
                    outtake.turretSpinInternalPID(0,1);
                    outtake.liftTo(2, outtake.liftPosition, 1);
                    if (GlobalTimer.milliseconds() - autoTimer > 0){
                        outtake.IntakeClawClose();
                        if (GlobalTimer.milliseconds() - autoTimer > 200){
                            outtake.IntakeLiftTransfer();
                            if (GlobalTimer.milliseconds()-autoTimer > 230){
                                outtake.IntakeArmTransfer();
                                if ((outtake.getIntakeArmPos() > 150)){ // this reads the position of the intake arm
                                    outtake.IntakeSlideTo(3,outtake.intakeSlidePosition,1);
                                    if (outtake.intakeSlidePosition > -9){ // this controls when the claw closes
                                        if (numCycles == 5){
                                            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                                            currentState = AutoState.DRIVE_OTHER_SIDE_AND_TRANSFER;
                                            drive.followTrajectoryAsync(DriveOtherSide);
                                            outtake.OuttakeClawClose();
                                        } else {
                                            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                                            currentState = AutoState.TRANSFER_CONE;
                                            outtake.OuttakeClawClose();
                                        }
                                    }
                                } else {
                                    outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideBackFromStack, outtake.intakeSlidePosition, 0.6); // this pulls slides in while doing stuff
                                }
                            }
                        } else {
                            outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideOutTicks, outtake.intakeSlidePosition, 1); // slower
                        }
                    } else {
                        outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideOutTicks, outtake.intakeSlidePosition, 1); // slower
                    }
                    break;
                case TRANSFER_CONE:
                    holdDrivebasePosition();
                    outtake.IntakeSlideTo(4,outtake.intakeSlidePosition,1); // so it holds in when transferring
                    outtake.turretSpin(0,outtake.turretPosition,1); // spin turret after
                    outtake.liftTo(2, outtake.liftPosition, 1);
                    outtake.OuttakeClawClose();
                    if (GlobalTimer.milliseconds()-autoTimer > 100){ // time between claw transfers
                        outtake.IntakeClawOpenHard();
                        if (GlobalTimer.milliseconds()-autoTimer > 200){
                            if (numCycles == 10){
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
                case DRIVE_OTHER_SIDE_AND_TRANSFER:
                    outtake.OuttakeClawClose();
                    outtake.IntakeSlideTo(1,outtake.intakeSlidePosition,1);
                    outtake.turretSpin(0,outtake.turretPosition,1); // spin turret after
                    outtake.liftTo(3, outtake.liftPosition, 1);
                    if (GlobalTimer.milliseconds()-autoTimer > 200){ // time between claw transfers
                        outtake.IntakeClawOpenHard();
                        if (!drive.isBusy()){ // if its finished driving
                            outtake.IntakeArmReady();
                            outtake.IntakeLift3();
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                            drive.setPoseEstimate(new Pose2d(xPosition,yPosition, headingPosition + Math.toRadians(180)));
                            currentState = AutoState.TURN_OTHER_STACK;
                            // offsets the heading to avoid the annoying angle wrap glitch
                        }
                    }
                    break;
                case TURN_OTHER_STACK:
                    // turns using my PID
                    holdDrivebaseOtherSide();
                    outtake.IntakeSlideTo(1,outtake.intakeSlidePosition,1);
                    outtake.turretSpin(0,outtake.turretPosition,1); // spin turret after
                    outtake.liftTo(0, outtake.liftPosition, 1);
                    //threshold is 1 inch, 2 degrees
                    if ((drivebase.getDistanceFromPosition(globalsCloseHighAuto.outconestackXOtherSide * SideMultiplier, globalsCloseHighAuto.outconestackY, globalsCloseHighAuto.outconeStackRotationOtherSide * SideMultiplier + AngleOffset,xPosition,yPosition,headingPosition) < 1) && drivebase.getHeadingError() < Math.toRadians(Math.abs(2))){ // have to deal with the heading here, read telemetry for heading angle
                        autoTimer = GlobalTimer.milliseconds(); // reset timer
                        currentState = AutoState.OUTTAKE_CONE;
                        holdDrivebaseOtherSide(); // just so it runs it this loop and doesn't drift??
                        OtherSide = true; // indicates that we are going to do the other stack
                    }
                    break;
                case OUTTAKE_CONE_NO_INTAKE_SLIDES:
                    OuttakeCone(false); // in a function so that i don't have to make 2 changes
                    holdDrivebasePosition();
                    break;
                case RETRACT_SLIDES:
                    holdDrivebasePosition();
                    outtake.OuttakeClawOpen();
                    if (GlobalTimer.milliseconds() - autoTimer > 150){
                        outtake.liftTo(0, outtake.liftPosition, 1);
                        outtake.BraceReady();
                        if (GlobalTimer.milliseconds() - autoTimer > 200){
                            outtake.OuttakeArmReady();
                            outtake.turretSpin(0,outtake.turretPosition,1); // spin turret after
                            if (outtake.liftTargetReached()){
                                autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                                currentState = AutoState.PARK;
                            }
                        }
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
                    outtake.liftTo(0, outtake.liftPosition, 1);
                    outtake.IntakeSlideTo(1,outtake.intakeSlidePosition,1);
                    break;

            }
            // Updates driving for trajectories
            drive.update();
            telemetry.update();
            if ((currentState == AutoState.IDLE || currentState == AutoState.DRIVE_OTHER_SIDE_AND_TRANSFER) && outtake.intakeSlidePosition > -2){
                outtake.IntakeClipHold();
            } else {
                outtake.IntakeClipOpen();
            }
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
            outtake.IntakeLift5();
            OtherSide = true; // indicates that we are going to do the other stack
        } else if (numCycles == 6){
            outtake.IntakeLift4();
        } else if (numCycles == 7){
            outtake.IntakeLift3();
        } else if (numCycles == 8){
            outtake.IntakeLift2();
        } else if (numCycles == 9){
            outtake.IntakeLift1();
        }
    }

    public void OuttakeCone(boolean Intake){
        if (Intake){ // if parameter is set to false it won't do the intake slides just outtake
            outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideNotQuiteOutTicks, outtake.intakeSlidePosition, 1); // move to just before the stack
            intakeLiftHeight();
            outtake.IntakeClawOpenHard();
            outtake.IntakeArmReady();
        } else {
            outtake.IntakeSlideTo(0, outtake.intakeSlidePosition,1); // move to just before the stack
        }

        outtake.liftTo(globalsCloseHighAuto.LiftHighPosition, outtake.liftPosition, 1);
        outtake.OuttakeClawClose();
        outtake.OuttakeArmScoreAuto();
        outtake.BraceActiveAuto();

        holdTurretPosition();

        if (outtake.liftPosition < globalsCloseHighAuto.LiftHighPosition+10){
            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
            currentState = AutoState.GRAB_OFF_STACK;
            outtake.OuttakeArmScoreAuto();
            outtake.BraceActiveAuto();
            numCycles += 1;
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
        if (!OtherSide){
            drivebase.DriveToPositionAutonomous(globalsCloseHighAuto.outconestackX * SideMultiplier, globalsCloseHighAuto.outconestackY,-globalsCloseHighAuto.outconeStackRotation* SideMultiplier + AngleOffset,xPosition,yPosition,headingPosition, 1,1); // last values are translationalspeed, and rotational speed
        } else if (OtherSide){
            holdDrivebaseOtherSide();
        }
    }

    public void holdDrivebaseOtherSide(){ // inputs the raw heading instead of corrected heading
        drivebase.DriveToPositionAutonomous(globalsCloseHighAuto.outconestackX * SideMultiplier, globalsCloseHighAuto.outconestackY,-globalsCloseHighAuto.outconeStackRotation* SideMultiplier + AngleOffset,xPosition,yPosition,headingPosition, 1,1); // last values are translationalspeed, and rotational speed
    }

    public void holdTurretPosition(){
        if (!OtherSide){
            outtake.turretSpin(globalsCloseHighAuto.TurretLeftposition, outtake.turretPosition,1);
        } else {
            outtake.turretSpin(globalsCloseHighAuto.TurretRightposition, outtake.turretPosition,1);
        }
    }
}


