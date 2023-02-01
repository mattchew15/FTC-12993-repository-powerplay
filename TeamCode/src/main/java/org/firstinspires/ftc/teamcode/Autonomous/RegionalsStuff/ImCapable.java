package org.firstinspires.ftc.teamcode.Autonomous.RegionalsStuff;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
