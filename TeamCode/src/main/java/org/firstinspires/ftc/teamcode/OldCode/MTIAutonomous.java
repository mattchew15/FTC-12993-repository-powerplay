package org.firstinspires.ftc.teamcode;

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




@Autonomous(name = "MTIAutonomous")
public class MTIAutonomous extends LinearOpMode {
    private DcMotor SlidesLeft;
    private DcMotor SlidesRight;
    private DcMotor Intake;
    private Servo CapArm;
    private Servo CapGrab;
    private DcMotor Turret;
    private Servo BucketServoLeft;
    private Servo BucketServoRight;
    ColorSensor FreightDetect;
    private Servo IntakeServoL;
    private Servo IntakeServoR;
    private Servo DumpServo;
    private Servo Sweeper;
    private CRServo CarouselL;
    private CRServo CarouselR;
    private DistanceSensor LeftDistance;
    private DistanceSensor FrontDistance;
    private DistanceSensor RightDistance;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;

    ElapsedTime GlobalTimer;
    double IntakeTimer;
    boolean DriveIntoWarehouseStraight;
    double NumCycles;
    double IntoWarehouseXValue;
    double CurrentWarehouseXValue;
    double CurrentDumpXValue;
    double IncreaseXDistance;
    double CapstonePosition;
    double DrivetoXPosition;
    double AutoState;

    enum State {
        DETECTION_IDLE_STATE,
        PRELOAD_DRIVE,
        PRELOAD_DUMP,
        POSITION_FIRST_DRIVE,
        FIRST_BALL_DRIVE,
        FIRST_BALL_DUMP,
        SECOND_BALL_DRIVE,
        SECOND_BALL_DUMP,
        THIRD_BALL_DRIVE,
        THIRD_BALL_LINEUP,
        REVERSE_INTAKE,
        DRIVE_DUMP_UPPER,
        DRIVE_STRAIGHT_WAREHOUSE,
        DRIVE_ANGLED_WAREHOUSE,
        DRIVE_OUT_DUMP,
        DUMP,
        DUMP_LOW,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    OpenCvWebcam webcam1 = null;

    // Define our start pose
    Pose2d startPose = new Pose2d(50, -32, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        CapArm = hardwareMap.get(Servo.class, "CapArm");
        SlidesLeft = hardwareMap.get(DcMotor.class, "SlidesMotorLeft");
        SlidesRight = hardwareMap.get(DcMotor.class, "SlidesMotorRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        IntakeServoL = hardwareMap.get(Servo.class, "IntakeServoL");
        IntakeServoR = hardwareMap.get(Servo.class, "IntakeServoR");
        BucketServoLeft = hardwareMap.get(Servo.class, "BucketServoL");
        BucketServoRight = hardwareMap.get(Servo.class, "BucketServoR");
        Sweeper = hardwareMap.get(Servo.class, "Sweeper");
        Turret = hardwareMap.get(DcMotor.class, "TurretMotor");
        CarouselL = hardwareMap.crservo.get("CarouselServoL");
        CarouselR = hardwareMap.crservo.get("CarouselServoR");
        DumpServo = hardwareMap.get(Servo.class, "DumpServo");
        FreightDetect = hardwareMap.get(ColorSensor.class, "FreightDetect");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        //reset encoders
        SlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        IntakeTimer = 0;
        DriveIntoWarehouseStraight = true;
        NumCycles = 0;
        IntoWarehouseXValue = 25; // causes empty path exeption segment if too low, idk why generates a trajectory
        CurrentWarehouseXValue = IntoWarehouseXValue;
        CurrentDumpXValue = 15;
        IncreaseXDistance = 5;
        DrivetoXPosition = IntoWarehouseXValue;
        AutoState = 0;

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        Trajectory PreloadDumpDrive = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12, -3))
                .build();

        Trajectory PositionFirstDrive = drive.trajectoryBuilder(PreloadDumpDrive.end())
                .lineToLinearHeading(new Pose2d(13, -5, Math.toRadians(-90)))
                .build();

        Trajectory FirstBallDrive = drive.trajectoryBuilder(PositionFirstDrive.end())
                .lineTo(new Vector2d(13, -17))
                .build();

        Trajectory FirstBallDump = drive.trajectoryBuilder(FirstBallDrive.end(), true)
                .lineTo(new Vector2d(10, -15))
                .build();

        Trajectory SecondBallDrive = drive.trajectoryBuilder(FirstBallDump.end())
                .lineTo(new Vector2d(13, -32))
                .build();

        Trajectory SecondBallDump = drive.trajectoryBuilder(SecondBallDrive.end(), true)
                .lineTo(new Vector2d(10, -15))
                .build();

        Trajectory ThirdBallDrive = drive.trajectoryBuilder(SecondBallDump.end())
                .lineTo(new Vector2d(14, -47))
                .build();

        Trajectory ThirdBallLineup = drive.trajectoryBuilder(ThirdBallDrive.end(), true)
                .lineToLinearHeading(new Pose2d(12, -72, Math.toRadians(0)))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        // open cv changes the state at the start depending on the location of TSE
        // check if state changes during init, if so code here
        // make sure for dump lower to set "drive.followasynchronous"

        currentState = State.PRELOAD_DRIVE;
        drive.followTrajectoryAsync(PreloadDumpDrive);

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot


            // Trajectories
            Trajectory DriveIntoStraight = drive.trajectoryBuilder(new Pose2d(12, -72, Math.toRadians(0))) // needs to drive in further each  (use variables for x)
                    .lineTo(new Vector2d(18, -72))
                    .lineTo(new Vector2d(IntoWarehouseXValue, -72), SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory DriveIntoAngled = drive.trajectoryBuilder(new Pose2d(12, -72, Math.toRadians(0))) // needs to drive in further each  (use variables for x)
                    .lineToLinearHeading(new Pose2d(IntoWarehouseXValue, -72, Math.toRadians(-20)), SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();


            switch (currentState) {
                case PRELOAD_DRIVE:
                    BucketServoRight.setPosition(0.53); // Extend bucket
                    BucketServoLeft.setPosition(0.3);
                    DumpServo.setPosition(0.7);
                    SetTurretPos(230);
                    if (!drive.isBusy()) {
                        AutoState = 1;
                        currentState = State.DRIVE_DUMP_UPPER;
                        //IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                    }
                    break;


                case POSITION_FIRST_DRIVE:
                    IntakeServoL.setPosition(0.9);
                    IntakeServoR.setPosition(0.01); // Set everything to initial position
                    Intake.setPower(1);
                    SlidesRight.setTargetPosition(2);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to min
                    SlidesLeft.setTargetPosition(2);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to min
                    BucketServoRight.setPosition(0.05); // Retract Bucket
                    DumpServo.setPosition(0.85);
                    BucketServoLeft.setPosition(0.78);
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(FirstBallDrive);
                        currentState = State.FIRST_BALL_DRIVE;
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                    }
                    break;

                case FIRST_BALL_DRIVE:
                    IntakeServoL.setPosition(0.9);
                    IntakeServoR.setPosition(0.01); // Set everything to initial position
                    Intake.setPower(1);
                    SlidesRight.setTargetPosition(2);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to min
                    SlidesLeft.setTargetPosition(2);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to min
                    BucketServoRight.setPosition(0.05); // Retract Bucket
                    DumpServo.setPosition(0.85);
                    BucketServoLeft.setPosition(0.78);
                    SetTurretPos(0);
                    if (!drive.isBusy()) {

                        currentState = State.FIRST_BALL_DUMP;
                        Intake.setPower(0);
                        drive.followTrajectoryAsync(FirstBallDump);
                        IntakeServoL.setPosition(0.36); // Flip up intakes
                        IntakeServoR.setPosition(0.55);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer

                    }
                    break;


                case SECOND_BALL_DRIVE:
                    IntakeServoL.setPosition(0.9);
                    IntakeServoR.setPosition(0.01); // Set everything to initial position
                    Intake.setPower(1);
                    SlidesRight.setTargetPosition(2);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to min
                    SlidesLeft.setTargetPosition(2);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to min
                    BucketServoRight.setPosition(0.05); // Retract Bucket
                    DumpServo.setPosition(0.85);
                    BucketServoLeft.setPosition(0.78);
                    if (!drive.isBusy()) {

                        currentState = State.SECOND_BALL_DUMP;
                        Intake.setPower(0);
                        drive.followTrajectoryAsync(SecondBallDump);
                        IntakeServoL.setPosition(0.36); // Flip up intakes
                        IntakeServoR.setPosition(0.55);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer

                    }
                    break;

                case THIRD_BALL_DRIVE:
                    IntakeServoL.setPosition(0.9);
                    IntakeServoR.setPosition(0.01); // Set everything to initial position
                    Intake.setPower(1);
                    SlidesRight.setTargetPosition(2);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to min
                    SlidesLeft.setTargetPosition(2);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to min
                    BucketServoRight.setPosition(0.05); // Retract Bucket
                    DumpServo.setPosition(0.85);
                    BucketServoLeft.setPosition(0.78);
                    if (!drive.isBusy()) {

                        currentState = State.THIRD_BALL_LINEUP;
                        Intake.setPower(0);
                        drive.followTrajectoryAsync(ThirdBallLineup);
                        IntakeServoL.setPosition(0.36); // Flip up intakes
                        IntakeServoR.setPosition(0.55);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer

                    }
                    break;

                case THIRD_BALL_LINEUP:
                    SetTurretPos(175);
                    if (!drive.isBusy()) {
                        currentState = State.REVERSE_INTAKE;
                        AutoState = 4;
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                    }
                    break;

                case REVERSE_INTAKE:
                    if (GlobalTimer.milliseconds() - IntakeTimer > 50) { // unecessary thing
                        Intake.setPower(-1); // Reverse intake
                        if (GlobalTimer.milliseconds() - IntakeTimer > 1000) { // the original timer + 1000 ms for slides
                            currentState = State.DRIVE_DUMP_UPPER;
                            //IntakeTimer = GlobalTimer.milliseconds();

                        }
                    }

                    break;

                case FIRST_BALL_DUMP:
                    SetTurretPos(0);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 350) {
                        Intake.setPower(-1); // Reverse intake
                        if (GlobalTimer.milliseconds() - IntakeTimer > 1000) { // the original timer + 1000 ms for slides
                            IntakeServoL.setPosition(0.9);
                            IntakeServoR.setPosition(0.01);
                            Intake.setPower(0);
                            SlidesRight.setTargetPosition(-1260);
                            SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SlidesRight.setPower(1); // Send slides to max
                            SlidesLeft.setTargetPosition(-1260);
                            SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SlidesLeft.setPower(1); // Send slides to max
                            BucketServoRight.setPosition(0.53); // Extend bucket
                            BucketServoLeft.setPosition(0.3);
                            DumpServo.setPosition(0.7);
                            if (!drive.isBusy() && SlidesLeft.getCurrentPosition() < -1200) {
                                currentState = State.DUMP;
                                AutoState = 2;
                                IntakeTimer = GlobalTimer.milliseconds();

                            }
                        }
                    }

                    break;

                case SECOND_BALL_DUMP:
                    SetTurretPos(0);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 350) {
                        Intake.setPower(-1); // Reverse intake
                        if (GlobalTimer.milliseconds() - IntakeTimer > 1000) { // the original timer + 1000 ms for slides
                            IntakeServoL.setPosition(0.9);
                            IntakeServoR.setPosition(0.01);
                            Intake.setPower(0);
                            SlidesRight.setTargetPosition(-1260);
                            SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SlidesRight.setPower(1); // Send slides to max
                            SlidesLeft.setTargetPosition(-1260);
                            SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SlidesLeft.setPower(1); // Send slides to max
                            BucketServoRight.setPosition(0.53); // Extend bucket
                            BucketServoLeft.setPosition(0.3);
                            DumpServo.setPosition(0.7);
                            if (!drive.isBusy() && SlidesLeft.getCurrentPosition() < -1200) {
                                currentState = State.DUMP;
                                AutoState = 3;
                                IntakeTimer = GlobalTimer.milliseconds();

                            }
                        }
                    }

                    break;

                case DRIVE_DUMP_UPPER:
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    SlidesRight.setTargetPosition(-1350);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to max
                    SlidesLeft.setTargetPosition(-1350);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to max
                    BucketServoRight.setPosition(0.53); // Extend bucket
                    BucketServoLeft.setPosition(0.3);
                    DumpServo.setPosition(0.7);
                    if (SlidesRight.getCurrentPosition() < -1300) {
                        currentState = State.DUMP;
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                        IntoWarehouseXValue += IncreaseXDistance;
                    }
                    break;


                case DUMP:
                    DumpServo.setPosition(0.85);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 100) { // wait between dump
                        if (AutoState == 1){
                            drive.followTrajectoryAsync(PositionFirstDrive);
                            currentState = State.POSITION_FIRST_DRIVE;
                        }
                        else if (AutoState == 2){
                            drive.followTrajectoryAsync(SecondBallDrive);
                            currentState = State.SECOND_BALL_DRIVE;
                        }
                        else if (AutoState == 3){
                            drive.followTrajectoryAsync(ThirdBallDrive);
                            currentState = State.THIRD_BALL_DRIVE;
                        }
                        else{
                            if (DriveIntoWarehouseStraight) {
                                drive.followTrajectoryAsync(DriveIntoStraight);
                                currentState = State.DRIVE_STRAIGHT_WAREHOUSE;
                                IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                            } else {
                                drive.followTrajectoryAsync(DriveIntoAngled);
                                currentState = State.DRIVE_ANGLED_WAREHOUSE;
                                IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                            }

                        }
                    }
                    break;

                case DRIVE_STRAIGHT_WAREHOUSE:
                    SetTurretPos(175);
                    IntakeServoL.setPosition(0.9);
                    IntakeServoR.setPosition(0.01); // Set everything to initial position
                    Intake.setPower(1);
                    SlidesRight.setTargetPosition(2);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to min
                    SlidesLeft.setTargetPosition(2);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to min
                    BucketServoRight.setPosition(0.05); // Retract Bucket
                    DumpServo.setPosition(0.85);
                    BucketServoLeft.setPosition(0.78);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 750 && GlobalTimer.milliseconds() - IntakeTimer < 2000) {
                        Sweeper.setPosition(0.15 - 0.15 * ((GlobalTimer.milliseconds() - IntakeTimer - 750) / 1250));
                    }
                    else {
                        Sweeper.setPosition(0.65);
                    }
                    if (!drive.isBusy() || FreightDetect.red() > 1000) {
                        // if using distance senosr use here to
                        //drive.setPoseEstimate(); distance sensor calculations

                        Trajectory DumpDriveOut = drive.trajectoryBuilder(poseEstimate, true)
                                .lineTo(new Vector2d(12, -72))
                                .build();

                        currentState = State.DRIVE_OUT_DUMP;
                        CurrentWarehouseXValue = poseEstimate.getX();
                        IntoWarehouseXValue = CurrentWarehouseXValue;
                        Intake.setPower(0);
                        drive.followTrajectoryAsync(DumpDriveOut);
                        IntakeServoL.setPosition(0.36); // Flip up intakes
                        IntakeServoR.setPosition(0.55);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                        DriveIntoWarehouseStraight = false;


                    }

                    break;

                case DRIVE_ANGLED_WAREHOUSE:
                    SetTurretPos(175);
                    IntakeServoL.setPosition(0.9);
                    IntakeServoR.setPosition(0.01); // Set everything to initial position
                    Intake.setPower(1);
                    SlidesRight.setTargetPosition(2);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to min
                    SlidesLeft.setTargetPosition(2);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to min
                    BucketServoRight.setPosition(0.05); // Retract Bucket
                    DumpServo.setPosition(0.85);
                    BucketServoLeft.setPosition(0.78);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 750 && GlobalTimer.milliseconds() - IntakeTimer < 2000) {
                        Sweeper.setPosition(0.15 - 0.15 * ((GlobalTimer.milliseconds() - IntakeTimer - 750) / 1250));
                    }
                    else {
                        Sweeper.setPosition(0.65);
                    }
                    if (!drive.isBusy() || FreightDetect.red() > 1000) {
                        // if using distance senosr use here to
                        //drive.setPoseEstimate(); distance sensor calculations

                        Trajectory DumpDriveOut = drive.trajectoryBuilder(poseEstimate, true)
                                .lineToLinearHeading(new Pose2d(12, -72, Math.toRadians(0)))
                                .build();

                        currentState = State.DRIVE_OUT_DUMP;
                        CurrentWarehouseXValue = poseEstimate.getX();
                        IntoWarehouseXValue = CurrentWarehouseXValue;
                        Intake.setPower(0);
                        drive.followTrajectoryAsync(DumpDriveOut);
                        IntakeServoL.setPosition(0.36); // Flip up intakes
                        IntakeServoR.setPosition(0.55);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                        DriveIntoWarehouseStraight = true;


                    }

                    break;


                case DRIVE_OUT_DUMP:
                    SetTurretPos(175);
                    Sweeper.setPosition(0.65);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 350) {
                        Intake.setPower(-1); // Reverse intake
                        if (GlobalTimer.milliseconds() - IntakeTimer > 1000) { // the original timer + 1000 ms for slides
                            IntakeServoL.setPosition(0.9);
                            IntakeServoR.setPosition(0.01);
                            Intake.setPower(0);
                            Sweeper.setPosition(0.65);
                            SlidesRight.setTargetPosition(-1260);
                            SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SlidesRight.setPower(1); // Send slides to max
                            SlidesLeft.setTargetPosition(-1260);
                            SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SlidesLeft.setPower(1); // Send slides to max
                            BucketServoRight.setPosition(0.53); // Extend bucket
                            BucketServoLeft.setPosition(0.3);
                            DumpServo.setPosition(0.7);
                            if (!drive.isBusy()) {
                                currentState = State.DUMP;
                                NumCycles += 1;
                                IntoWarehouseXValue = DrivetoXPosition + (IncreaseXDistance * NumCycles);
                                CurrentDumpXValue = poseEstimate.getX();
                                DriveIntoWarehouseStraight = false;
                                IntakeTimer = GlobalTimer.milliseconds();

                            }
                        }
                    }

                    break;

                case IDLE:

                    break;
            }

            // Cap Arm Holds position throughout match
            CapArm.setPosition(0.7);
            // Updates driving for trajectories
            drive.update();


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            //telemetry.addData("SlidesLeftPos", SlidesLeft.getCurrentPosition());
            telemetry.addData("State", currentState);
            telemetry.addData("AutoState", AutoState);
            telemetry.addData("RED", FreightDetect.red());
            telemetry.addData("TurretPosition", Turret.getCurrentPosition());
            telemetry.update();



        }

    }

    public void SetTurretPos (int TurretPos) {
        Turret.setTargetPosition(TurretPos);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPower(1);
    }

}


