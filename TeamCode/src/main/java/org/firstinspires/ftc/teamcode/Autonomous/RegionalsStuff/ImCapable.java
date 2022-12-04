package org.firstinspires.ftc.teamcode.Autonomous.RegionalsStuff;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Queue;
import java.util.Vector;

@Autonomous(name = "Base pole cycle", group= "Autonomous")

public class ImCapable extends LinearOpMode {

    int slowerVelocityConstraint;
    final double outconestackRotation = 180;

    private void Setup(){
        slowerVelocityConstraint = 20;
    }

    Pose2d startPose = new Pose2d(38, -69, Math.toRadians(180));

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // road drive class

    @Override
    public void runOpMode() throws InterruptedException {

        Setup();

        drive.setPoseEstimate(startPose);

        Trajectory Test = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(38, -20, Math.toRadians(0)))
                .lineTo(new Vector2d(35, -23))
                .lineTo(new Vector2d(58.33, -11.71))
                .lineTo(new Vector2d(11.67, -11.71))
                .lineTo(new Vector2d(58.33, -11.71))

                .build();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            drive.followTrajectoryAsync(Test);
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate(); // gets the position of the robot

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.update();

            drive.update();
        }
    }
}
