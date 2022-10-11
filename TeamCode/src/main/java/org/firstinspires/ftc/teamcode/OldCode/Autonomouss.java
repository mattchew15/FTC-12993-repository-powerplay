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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Queue;
import java.util.Vector;




@Autonomous(name = "Autonomouss")
public class Autonomouss extends LinearOpMode {

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

    @Override
    public void runOpMode() {
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


        waitForStart();
        if (opModeIsActive()) {
            // creating trajectories
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(15, 63, Math.toRadians(0));
            drive.setPoseEstimate(startPose);

            Trajectory Drive = drive.trajectoryBuilder(startPose, true)
                    .build();

            Trajectory DriveIntoAngled = drive.trajectoryBuilder(new Pose2d(15, -63, Math.toRadians(0)))
                    .splineTo(new Vector2d((IntoWarehouseXValue - 10), -63), Math.toRadians(0)) // second one slowed down
                    .splineTo(new Vector2d(IntoWarehouseXValue, -58), Math.toRadians(20), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            Trajectory DumpDriveOutAfterAngled = drive.trajectoryBuilder(DriveIntoAngled.end(), true)
                    .splineTo(new Vector2d(30, -63), Math.toRadians(-20))
                    .splineTo(new Vector2d(15, -63), Math.toRadians(-20))
                    .build();

            drive.followTrajectory(DriveIntoAngled);
            drive.followTrajectory(DumpDriveOutAfterAngled);

        }
    }
}

//@disabled

