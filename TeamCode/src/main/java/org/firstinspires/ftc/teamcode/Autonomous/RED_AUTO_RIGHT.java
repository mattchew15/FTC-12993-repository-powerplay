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

    // create class instances
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // road drive class
    DriveBase drivebase = new DriveBase(); // hardware classes
    TurretLift turretlift = new TurretLift();
    Inputs inputs = new Inputs();
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1"; // what our webcam is called in hardware class

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

    // function runs on init
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        inputs.resetMatchTimer();
        drivebase.motorsSetup();
        turretlift.motorsSetup();

        currentState = AutoState.PRELOAD_DRIVE; // this go here?
        autoTimer = 0;
        outakeResetReady = true;
        outakeOutReady = false;
        numCycles = 0;
        heightChangeInterval = 0;
    }


    @Override
    public void runOpMode() throws InterruptedException {

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
        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90))) // spline to spline heading, first angle is target, second angle is target angle during path
                .splineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)), Math.toRadians(90)) // end effects shape of spline, first angle is the target heading
                .build();

        Trajectory IntoConeStack = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(47,-12))
                .build();

        Trajectory OutConeStack = drive.trajectoryBuilder(IntoConeStack.end())
                .lineTo(new Vector2d(35,-12))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(OutConeStack.end())
                .lineTo(new Vector2d(58.5,-12))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(OutConeStack.end())
                .lineTo(new Vector2d(12,-12))
                .build();

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.addData("Yellow Percent:", sleeveDetection.yelPercent);
            telemetry.addData("Blue Percent:", sleeveDetection.bluPercent);
            telemetry.addData("Red Percent:", sleeveDetection.redPercent);
            telemetry.update();
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

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose

            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot
            // main switch statement logic
            switch (currentState) {
                case PRELOAD_DRIVE:
                    drive.followTrajectoryAsync(PreloadDrive);
                    // get outake ready - do timer to make it later, putt hsi in a function
                    outakeOutReady(160,1,350, liftHighPosition);
                    if (!drive.isBusy()) {
                        currentState = AutoState.PRELOAD_DROP;
                        autoTimer = GlobalTimer.milliseconds();
                        turretlift.openClaw();
                    }
                    break;

                case PRELOAD_DROP:
                    if (GlobalTimer.milliseconds() - autoTimer > 300){
                        readyOutake();
                        drive.followTrajectoryAsync(IntoConeStack);
                        if (drive.isBusy()){
                            turretlift.closeClaw();
                            currentState = AutoState.DRIVE_OUT_STACK;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                        }
                    }
                    break;

                case DRIVE_OUT_STACK:
                    if (GlobalTimer.milliseconds() - autoTimer > 200){ // this if statement is not needed at all
                        turretlift.liftToInternalPID(liftMidPosition,1);
                        if (GlobalTimer.milliseconds() - autoTimer > 300){
                            turretlift.linkageIn();
                            drive.followTrajectoryAsync(OutConeStack);
                            if (GlobalTimer.milliseconds() - autoTimer > 350){
                                outakeOutReady(170,1,liftMidPosition, liftMidPosition);
                                if (!drive.isBusy() && outakeOutReady){
                                    turretlift.openClaw();
                                    if (numCycles == 3){
                                        currentState = AutoState.PARK;
                                        autoTimer = GlobalTimer.milliseconds(); // reset timer
                                        numCycles += 1;
                                    }
                                    else{
                                        heightChangeInterval += 20; // tune this changes how high it changes each time
                                        numCycles += 1;
                                        currentState = AutoState.DRIVE_INTO_STACK;
                                        autoTimer = GlobalTimer.milliseconds(); // reset timer
                                    }
                                }
                            }
                        }
                    }
                    break;

                case DRIVE_INTO_STACK:
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        drive.followTrajectoryAsync(IntoConeStack);
                        readyOutake();
                        if (!drive.isBusy() && outakeResetReady){
                            turretlift.closeClaw();
                            currentState = AutoState.DRIVE_OUT_STACK;
                            autoTimer = GlobalTimer.milliseconds(); // reset timer
                        }
                    }
                    break;

                case PARK:
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        readyOutake();
                        if (SignalRotation == 1){
                            drive.followTrajectoryAsync(ParkLeft);
                            if (!drive.isBusy()){
                                currentState = AutoState.IDLE;
                            }
                        }
                        else if (SignalRotation == 3){
                            drive.followTrajectoryAsync(ParkRight);
                            if (!drive.isBusy()){
                                currentState = AutoState.IDLE;
                            }
                        }
                        else{
                            currentState = AutoState.IDLE;
                        }
                    }
                    break;

                case IDLE:
                    telemetry.addLine("W Auto");
                    break;

            }
            // Updates driving for trajectories
            drive.update();
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heightChangeInterval", heightChangeInterval);
            telemetry.addData("autostate", currentState);
            telemetry.addData("number of cycles:", numCycles);
        }

    }

    public void readyOutake(){
        turretlift.turretSpinInternalPID(0, 1);
        turretlift.liftToInternalPID(360,0.5);
        turretlift.readyServos();
        if (turretlift.turretTargetReachedInteralPID()){
            turretlift.liftToInternalPID(200 - heightChangeInterval,0.5); // could be faster
            turretlift.openClaw();
            turretlift.linkageOut();
            if (turretlift.liftTargetReachedInternalPID()){
                outakeResetReady = true; // need a false here if (turretlift.liftTargetReachedInternalPID())
            }
            else{
                outakeResetReady = false;
            }
        }else{
            outakeResetReady = false;
        }

    }

    public void outakeOutReady(int turretPosition, int liftSpeed, int liftposition, int liftposition2){ // way to use timers here
        if (turretlift.liftTargetReachedInternalPID()){
            telemetry.addLine("lift is up");
            turretlift.turretSpinInternalPID((int)Math.round(turretlift.degreestoTicks(turretPosition)), 1); //
            if (turretlift.turretTargetReachedInteralPID()){
                turretlift.liftToInternalPID(liftposition2, 1);
                turretlift.tiltUp();
                turretlift.linkageOutHalf();
                if (turretlift.liftTargetReachedInternalPID()){
                    outakeOutReady = true;
                }
            }
        }
        else{
            drivebase.intakeSpin(0);
            turretlift.closeClaw();
            turretlift.linkageIn();
            drivebase.intakeBarDown();
            turretlift.liftToInternalPID(liftposition,liftSpeed);
            turretlift.tiltUpHalf();
            outakeOutReady = false;
        }
    }
}


