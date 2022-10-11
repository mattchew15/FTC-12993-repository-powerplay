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




@Autonomous(name = "BLUE_CYCLE_MTI")
public class BLUE_CYCLE_MTI extends LinearOpMode {
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
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

    enum State {
        DETECTION_IDLE_STATE,
        DRIVE_DUMP_LOWER_FIRST,
        DRIVE_DUMP_LOWER_WAREHOUSE,
        DRIVE_DUMP_UPPER,
        DRIVE_DUMP_MID,
        DRIVE_STRAIGHT_WAREHOUSE,
        DRIVE_ANGLED_WAREHOUSE,
        DRIVE_MOSTANGLED_WAREHOUSE,
        DRIVE_OUT_DUMP,
        DRIVE_OUT_DUMP_AFTER_ANGLED,
        DUMP,
        DUMP_LOW,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    OpenCvWebcam webcam1 = null;

    // Define our start pose
    Pose2d startPose = new Pose2d(15, -63, Math.toRadians(0));

    class detectstuff extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 255.0, 0.0);


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


            Rect rightRect = new Rect(1, 1, 426, 719);
            Rect leftRect = new Rect(426, 1, 426, 719);
            Rect midRect = new Rect(853, 1, 426, 719);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);
            Imgproc.rectangle(outPut, midRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            if (leftavgfin < rightavgfin && leftavgfin < midavgfin) {
                //telemetry.addLine("The Capstone Is At Mid");
                CapstonePosition = 2;

            } else if (rightavgfin < midavgfin && rightavgfin < leftavgfin) {
                //telemetry.addLine("The Capstone Is At Left");
                CapstonePosition = 1;
            } else if (midavgfin < rightavgfin && midavgfin < leftavgfin) {
                //telemetry.addLine("The Capstone is At Right");
                CapstonePosition = 3;

            }

            return (outPut);

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new BLUE_CYCLE_MTI.detectstuff());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });

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
        IntoWarehouseXValue = 43;
        CurrentWarehouseXValue = IntoWarehouseXValue;
        CurrentDumpXValue = 15;
        IncreaseXDistance = 3.5;
        DrivetoXPosition = IntoWarehouseXValue;

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        Trajectory DumpLower = drive.trajectoryBuilder(startPose, true)
                .lineToConstantHeading(new Vector2d(2, -81))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        // open cv changes the state at the start depending on the location of TSE
        // check if state changes during init, if so code here
        // make sure for dump lower to set "drive.followasynchronous"
        if (CapstonePosition == 1) {
            currentState = State.DRIVE_DUMP_LOWER_FIRST;
            telemetry.addLine("The Capstone Is At Left");
            drive.followTrajectoryAsync(DumpLower);
        } else if (CapstonePosition == 2) {
            currentState = State.DRIVE_DUMP_MID;
            telemetry.addLine("The Capstone Is At Mid");
        } else {
            currentState = State.DRIVE_DUMP_UPPER;
            telemetry.addLine("The Capstone is At Right");
        }

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot


            // Trajectories



            Trajectory DumpLower2 = drive.trajectoryBuilder(DumpLower.end())
                    .splineToConstantHeading(new Vector2d(13, -63), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(35, -63), Math.toRadians(0)) // second one slowed down
                    .splineToConstantHeading(new Vector2d(50, -63), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory DriveIntoStraight = drive.trajectoryBuilder(new Pose2d(15, -63, Math.toRadians(0))) // needs to drive in further each  (use variables for x)
                    .lineTo(new Vector2d((IntoWarehouseXValue - 10), -63))
                    .lineTo(new Vector2d(IntoWarehouseXValue, -63), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory DriveIntoAngled = drive.trajectoryBuilder(new Pose2d(15, -63, Math.toRadians(0)))
                    .splineTo(new Vector2d(43, -63), Math.toRadians(0)) // second one slowed down
                    .splineTo(new Vector2d(IntoWarehouseXValue, -58), Math.toRadians(20), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();



            Trajectory DumpDriveOutAfterAngled = drive.trajectoryBuilder(DriveIntoAngled.end(), true)
                    .splineTo(new Vector2d(30, -63), Math.toRadians(-20))
                    .splineTo(new Vector2d(15, -63), Math.toRadians(-20))
                    .build();

            switch (currentState) {
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
                        NumCycles += 1;
                        IntoWarehouseXValue += IncreaseXDistance;
                    }
                    break;

                case DRIVE_DUMP_MID:
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    SlidesRight.setTargetPosition(-1200);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(0.5); // Send slides to max
                    SlidesLeft.setTargetPosition(-1200);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(0.5); // Send slides to max
                    BucketServoRight.setPosition(0.73); // Extend bucket
                    BucketServoLeft.setPosition(0.1);
                    DumpServo.setPosition(0.7);
                    if (SlidesRight.getCurrentPosition() < -1150) {
                        currentState = State.DUMP;
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                    }
                    break;

                case DRIVE_DUMP_LOWER_FIRST:
                    SlidesRight.setTargetPosition(-200);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to max
                    SlidesLeft.setTargetPosition(-200);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to max
                    BucketServoRight.setPosition(0.73); // Extend bucket
                    BucketServoLeft.setPosition(0.1);
                    DumpServo.setPosition(0.7);
                    if (!drive.isBusy()) {
                        currentState = State.DUMP_LOW;
                        NumCycles += 1;
                        IntoWarehouseXValue += IncreaseXDistance;
                        drive.followTrajectoryAsync(DumpLower2);
                    }
                    break;
                case DUMP_LOW:
                    DumpServo.setPosition(0.85);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 500) { //might need increasing
                        currentState = State.DRIVE_DUMP_LOWER_WAREHOUSE;
                        drive.followTrajectoryAsync(DumpLower2);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer

                    }
                    break;
                case DUMP:
                    DumpServo.setPosition(0.85);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 100) { // wait between dump
                        currentState = State.DRIVE_STRAIGHT_WAREHOUSE;
                        drive.followTrajectoryAsync(DriveIntoStraight);
                    }
                    break;
                case DRIVE_DUMP_LOWER_WAREHOUSE: // drives into warehouse after dumping in low position

                    IntakeServoL.setPosition(0.9);
                    IntakeServoR.setPosition(0.01); // Set everything to initial position
                    Intake.setPower(1);
                    SlidesRight.setTargetPosition(2);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to max
                    SlidesLeft.setTargetPosition(2);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to max
                    BucketServoRight.setPosition(0.05); // Retract Bucket
                    DumpServo.setPosition(0.85);
                    BucketServoLeft.setPosition(0.78);
                    if (!drive.isBusy() || FreightDetect.red() > 200) {
                        currentState = State.DRIVE_OUT_DUMP;
                        Trajectory DumpDriveOut = drive.trajectoryBuilder(poseEstimate, true)
                                .lineTo(new Vector2d(15, -63))
                                .build();

                        drive.followTrajectoryAsync(DumpDriveOut);
                        IntakeServoL.setPosition(0.36); // Flip up intakes
                        IntakeServoR.setPosition(0.55);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                    }
                    break;

                case DRIVE_STRAIGHT_WAREHOUSE:
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
                    if (!drive.isBusy() || FreightDetect.red() > 200) {
                        // if using distance senosr use here to
                        //drive.setPoseEstimate(); distance sensor calculations

                        Trajectory DumpDriveOut = drive.trajectoryBuilder(poseEstimate, true)
                                .lineTo(new Vector2d(15, -63))
                                .build();

                        currentState = State.DRIVE_OUT_DUMP;
                        CurrentWarehouseXValue = poseEstimate.getX();
                        IntoWarehouseXValue = CurrentWarehouseXValue;
                        Intake.setPower(0);
                        drive.followTrajectoryAsync(DumpDriveOut);
                        IntakeServoL.setPosition(0.36); // Flip up intakes
                        IntakeServoR.setPosition(0.55);
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer


                    }

                    break;



                case DRIVE_OUT_DUMP:
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
                            if (!drive.isBusy() || SlidesLeft.getCurrentPosition() < -1200) {
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
                    Turret.setTargetPosition(200);
                    Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Turret.setPower(1);
                    BL.setPower(0.2);
                    BR.setPower(0.2);
                    FL.setPower(0.2);
                    FR.setPower(0.2);
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
            //telemetry.addData("AutoState", currentState);
            telemetry.addData("Number of cycles", NumCycles);
            telemetry.addData("X value Trajectory Warehouse", IntoWarehouseXValue);
            telemetry.addData("Current X value Warehouse", CurrentWarehouseXValue);
            telemetry.update();

            // sets the turret position based on the state
            if (currentState == State.DUMP_LOW) {
                Turret.setTargetPosition(0);
                Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Turret.setPower(1);
                telemetry.addData("Turret Position", "LowDump");

            }
            else if (NumCycles == 5) {
                currentState = State.IDLE;
            }
            else {
                Turret.setTargetPosition(0);
                Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Turret.setPower(1);
                telemetry.addData("Turret Position", "Normal(Alliance)");
            }

        }

    }

}


