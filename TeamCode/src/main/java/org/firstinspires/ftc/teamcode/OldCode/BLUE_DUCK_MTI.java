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




@Autonomous(name = "BLUE_DUCK_MTI")
public class BLUE_DUCK_MTI extends LinearOpMode {
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
        FIRST_DRIVES,
        SECOND_DRIVES,
        TURN,
        DUMP_UPPER,
        DUMP_MID,
        DUMP_LOW,
        DUMP,
        DUMP_DRIVE,
        PREPARE_FOR_PARK,
        PARK,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    OpenCvWebcam webcam1 = null;


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
            //telemetry.addData("leftavgfin", leftavgfin);
            //telemetry.addData("rightavgfin", rightavgfin);
            //telemetry.addData("midavgfin", midavgfin);
            return (outPut);

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new BLUE_DUCK_MTI.detectstuff());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });

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

        // Define our start pose
        Pose2d startPose = new Pose2d(15, -63, Math.toRadians(0));

        // Set inital pose
        drive.setPoseEstimate(startPose);


        //trajectories
        Trajectory FirstDrives = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(50, -32))
                .build();

        Trajectory SecondDrives = drive.trajectoryBuilder(FirstDrives.end())
                .lineToConstantHeading(new Vector2d(15, -3))
                .build();


        double turnAngle = Math.toRadians(180);

        Pose2d turnend = SecondDrives.end().plus(new Pose2d(0, 0, turnAngle));

        Trajectory DumpDrive = drive.trajectoryBuilder(turnend)
                .lineToConstantHeading(new Vector2d(-38, 20))
                .build();

        Trajectory PrepareForPark = drive.trajectoryBuilder(DumpDrive.end())
                .lineToLinearHeading(new Pose2d(-58, 15, Math.toRadians(90)))
                .build();

        Trajectory Park = drive.trajectoryBuilder(PrepareForPark.end())
                .lineTo(new Vector2d(-63, 35))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        currentState = State.FIRST_DRIVES;
        drive.followTrajectoryAsync(FirstDrives);
        // open cv changes the state at the start depending on the location of TSE
        // check if state changes during init, if so code here
        // make sure for dump lower to set "drive.followasynchronous"
        if (CapstonePosition == 1) {
            telemetry.addLine("The Capstone Is At Left");
        } else if (CapstonePosition == 2) {
            telemetry.addLine("The Capstone Is At Mid");
        } else {
            telemetry.addLine("The Capstone is At Right");
        }

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot

            IntakeServoL.setPosition(0.36); // Flip up intakes
            IntakeServoR.setPosition(0.55);

            switch (currentState) {
                case FIRST_DRIVES:

                    if (!drive.isBusy()) {
                        currentState = State.SECOND_DRIVES;
                        drive.followTrajectoryAsync(SecondDrives);
                    }
                    break;

                case SECOND_DRIVES:

                    if (!drive.isBusy()) {
                        currentState = State.TURN;
                        drive.turnAsync(turnAngle);
                    }
                    break;

                case TURN:

                    if (!drive.isBusy()) {
                        if (CapstonePosition == 1) {
                            IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                            currentState = State.DUMP_LOW;

                        } else if (CapstonePosition == 2) {
                            IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                            currentState = State.DUMP_MID;
                        } else if (CapstonePosition == 3) {

                            currentState = State.DUMP_UPPER;
                        }
                    }
                    break;

                case DUMP_UPPER:
                    SlidesRight.setTargetPosition(-1000);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to max
                    SlidesLeft.setTargetPosition(-1000);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to max
                    BucketServoRight.setPosition(0.53); // Extend bucket
                    BucketServoLeft.setPosition(0.3);
                    DumpServo.setPosition(0.7);
                    if (SlidesLeft.getCurrentPosition() < -950) {
                        currentState = State.DUMP;
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                    }
                    break;

                case DUMP_DRIVE:

                    if (!drive.isBusy()) {
                        if (CapstonePosition == 3) {
                            currentState = State.DUMP;
                            IntakeTimer = GlobalTimer.milliseconds();

                        } else if (CapstonePosition == 2) {
                            currentState = State.DUMP;
                            IntakeTimer = GlobalTimer.milliseconds();
                        }
                    }
                    break;

                case DUMP_MID:
                    BucketServoRight.setPosition(0.53); // Extend bucket
                    BucketServoLeft.setPosition(0.3);
                    DumpServo.setPosition(0.7);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 2000) {
                        currentState = State.DUMP_DRIVE;
                        drive.followTrajectoryAsync(DumpDrive);
                    }
                    break;

                case DUMP_LOW:
                    BucketServoRight.setPosition(0.73); // Extend bucket
                    BucketServoLeft.setPosition(0.1);
                    DumpServo.setPosition(0.7);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 2000) {
                        currentState = State.DUMP_DRIVE;
                        drive.followTrajectoryAsync(DumpDrive);
                    }
                    break;


                case DUMP:
                    DumpServo.setPosition(0.85);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 2000) { // wait between dump
                        currentState = State.PREPARE_FOR_PARK;
                        drive.followTrajectoryAsync(PrepareForPark);
                    }
                    break;

                case PREPARE_FOR_PARK:
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
                        currentState = State.PARK;
                        drive.followTrajectoryAsync(Park);
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:

                    break;

            }


            // Cap Arm Holds position throughout match
            CapArm.setPosition(0.7);
            // Updates driving for trajectories
            drive.update();

            Turret.setTargetPosition(0);
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(1);
            telemetry.addData("Turret Position", "LowDump");
            telemetry.addData("State", currentState);
            telemetry.addData("SlidesLeftPos", SlidesLeft.getCurrentPosition());
            telemetry.addData("Capstone Position", CapstonePosition);

            telemetry.update();

        }

    }

}




/* old code lol for blue duck side
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




@Autonomous(name = "BLUE_DUCK_MTI")
public class BLUE_DUCK_MTI extends LinearOpMode {
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
        FIRST_DRIVES,
        SECOND_DRIVES,
        TURN,
        DUMP_UPPER,
        DUMP_MID,
        DUMP_LOW,
        DUMP,
        DUMP_DRIVE,
        PREPARE_FOR_PARK,
        PARK,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    OpenCvWebcam webcam1 = null;


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
            //telemetry.addData("leftavgfin", leftavgfin);
            //telemetry.addData("rightavgfin", rightavgfin);
            //telemetry.addData("midavgfin", midavgfin);
            return (outPut);

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new BLUE_DUCK_MTI.detectstuff());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });

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

        // Define our start pose
        Pose2d startPose = new Pose2d(-39, 63, Math.toRadians(0));

        // Set inital pose
        drive.setPoseEstimate(startPose);


        //trajectories
        Trajectory FirstDrives = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-61, 50))
                .build();

        Trajectory SecondDrives = drive.trajectoryBuilder(FirstDrives.end())
                .lineToConstantHeading(new Vector2d(-61, 20))
                .build();


        double turnAngle = Math.toRadians(180);

        Pose2d turnend = SecondDrives.end().plus(new Pose2d(0, 0, turnAngle));

        Trajectory DumpDrive = drive.trajectoryBuilder(turnend)
                .lineToConstantHeading(new Vector2d(-38, 20))
                .build();

        Trajectory PrepareForPark = drive.trajectoryBuilder(DumpDrive.end())
                .lineToLinearHeading(new Pose2d(-58, 15, Math.toRadians(90)))
                .build();

        Trajectory Park = drive.trajectoryBuilder(PrepareForPark.end())
                .lineTo(new Vector2d(-63, 35))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        currentState = State.FIRST_DRIVES;
        drive.followTrajectoryAsync(FirstDrives);
        // open cv changes the state at the start depending on the location of TSE
        // check if state changes during init, if so code here
        // make sure for dump lower to set "drive.followasynchronous"
        if (CapstonePosition == 1) {
            telemetry.addLine("The Capstone Is At Left");
        } else if (CapstonePosition == 2) {
            telemetry.addLine("The Capstone Is At Mid");
        } else {
            telemetry.addLine("The Capstone is At Right");
        }

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot

            IntakeServoL.setPosition(0.36); // Flip up intakes
            IntakeServoR.setPosition(0.55);

            switch (currentState) {
                case FIRST_DRIVES:

                    if (!drive.isBusy()) {
                        currentState = State.SECOND_DRIVES;
                        drive.followTrajectoryAsync(SecondDrives);
                    }
                    break;

                case SECOND_DRIVES:

                    if (!drive.isBusy()) {
                        currentState = State.TURN;
                        drive.turnAsync(turnAngle);
                    }
                    break;

                case TURN:

                    if (!drive.isBusy()) {
                        if (CapstonePosition == 1) {
                            IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                            currentState = State.DUMP_LOW;

                        } else if (CapstonePosition == 2) {
                            IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                            currentState = State.DUMP_MID;
                        } else if (CapstonePosition == 3) {

                            currentState = State.DUMP_UPPER;
                        }
                    }
                    break;

                case DUMP_UPPER:
                    SlidesRight.setTargetPosition(-1000);
                    SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesRight.setPower(1); // Send slides to max
                    SlidesLeft.setTargetPosition(-1000);
                    SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlidesLeft.setPower(1); // Send slides to max
                    BucketServoRight.setPosition(0.53); // Extend bucket
                    BucketServoLeft.setPosition(0.3);
                    DumpServo.setPosition(0.7);
                    if (SlidesLeft.getCurrentPosition() < -950) {
                        currentState = State.DUMP;
                        IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                    }
                    break;

                case DUMP_DRIVE:

                    if (!drive.isBusy()) {
                        if (CapstonePosition == 3) {
                            currentState = State.DUMP;
                            IntakeTimer = GlobalTimer.milliseconds();

                        } else if (CapstonePosition == 2) {
                            currentState = State.DUMP;
                            IntakeTimer = GlobalTimer.milliseconds();
                        }
                    }
                    break;

                case DUMP_MID:
                    BucketServoRight.setPosition(0.53); // Extend bucket
                    BucketServoLeft.setPosition(0.3);
                    DumpServo.setPosition(0.7);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 2000) {
                        currentState = State.DUMP_DRIVE;
                        drive.followTrajectoryAsync(DumpDrive);
                    }
                    break;

                case DUMP_LOW:
                    BucketServoRight.setPosition(0.73); // Extend bucket
                    BucketServoLeft.setPosition(0.1);
                    DumpServo.setPosition(0.7);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 2000) {
                        currentState = State.DUMP_DRIVE;
                        drive.followTrajectoryAsync(DumpDrive);
                    }
                    break;


                case DUMP:
                    DumpServo.setPosition(0.85);
                    if (GlobalTimer.milliseconds() - IntakeTimer > 2000) { // wait between dump
                        currentState = State.PREPARE_FOR_PARK;
                        drive.followTrajectoryAsync(PrepareForPark);
                    }
                    break;

                case PREPARE_FOR_PARK:
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
                        currentState = State.PARK;
                        drive.followTrajectoryAsync(Park);
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:

                    break;

            }


            // Cap Arm Holds position throughout match
            CapArm.setPosition(0.7);
            // Updates driving for trajectories
            drive.update();

            Turret.setTargetPosition(0);
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(1);
            telemetry.addData("Turret Position", "LowDump");
            telemetry.addData("State", currentState);
            telemetry.addData("SlidesLeftPos", SlidesLeft.getCurrentPosition());
            telemetry.addData("Capstone Position", CapstonePosition);

            telemetry.update();

        }

    }

}





 */
