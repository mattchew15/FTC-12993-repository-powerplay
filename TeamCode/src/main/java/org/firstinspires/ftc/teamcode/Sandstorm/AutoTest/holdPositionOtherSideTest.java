package org.firstinspires.ftc.teamcode.Sandstorm.AutoTest;
import static org.firstinspires.ftc.teamcode.Sandstorm.CloseHighAutos.GlobalsCloseHighAuto.outconestackXOtherSide;
import static org.firstinspires.ftc.teamcode.Sandstorm.CloseHighAutos.GlobalsCloseHighAuto.outconestackY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sandstorm.DriveBase;
import org.firstinspires.ftc.teamcode.Sandstorm.Inputs;
import org.firstinspires.ftc.teamcode.Sandstorm.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name = "Hold Position Other Side Test", group = "Autonomous")
public class holdPositionOtherSideTest extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    boolean OtherSide;

    int numCycles;
    int SignalRotation;
    int slowerVelocityConstraint;

    double correctedHeadingOtherSide;
    double xPosition;
    double yPosition;
    double headingPosition;
    double offsetHeading;

    // create class instances
    Outtake outtake = new Outtake();
    DriveBase drivebase = new DriveBase();
    Inputs inputs = new Inputs();
    OpenCvCamera camera;

    enum AutoState {

        TURN_OTHER_STACK,
        TURN_FORWARDS
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
        currentState = AutoState.TURN_OTHER_STACK;
        autoTimer = 0;
        numCycles = 0;
        slowerVelocityConstraint = 12;
        outtake.encodersReset();
        OtherSide = true;
    }
    // Define our start pose

    Pose2d startPose = new Pose2d(outconestackXOtherSide, outconestackY, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // check this
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.enable();

        // initialize hardware
        outtake.Outtake_init(hardwareMap);
        drivebase.Drivebase_init(hardwareMap); // this might conflict with road runner
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // road drive class

        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        //drive.followTrajectoryAsync(PreloadDrive);
        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry

            xPosition = poseEstimate.getX();
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading(); // returns raw heading position
            offsetHeading = inputs.offsetAngle90(headingPosition);
            telemetry.addData("OtherSide", OtherSide);
            telemetry.addData("heading", Math.toDegrees(headingPosition));
            telemetry.addData("offset heading", Math.toDegrees(offsetHeading));

            //telemetry.addData("autostate", currentState);
            telemetry.addData("HeadingError", drivebase.getHeadingError());
            telemetry.addData("HeadingOutput", drivebase.getHeadingOutput());
            telemetry.addData("XError", drivebase.getXError());
            telemetry.addData("Xoutput", drivebase.getXOutput());
            telemetry.addData("YError", drivebase.getYError());
            telemetry.addData("Youtput", drivebase.getYOutput());



            // main switch statement logic
            switch (currentState) {
                case TURN_OTHER_STACK:
                    // turns using my PID
                    holdDrivebaseOtherSide();
                    outtake.IntakeSlideTo(1,outtake.IntakeSlidePos(),1);
                    outtake.turretSpin(0,outtake.turretPos(),1); // spin turret after
                    outtake.liftTo(0, outtake.liftPos(), 1);
                    //threshold is 1 inch, 2 degrees
                    if (gamepad1.b){ // have to deal with the heading here, read telemetry for heading angle
                        autoTimer = GlobalTimer.milliseconds(); // reset timer
                        currentState = AutoState.TURN_FORWARDS;
                    }
                    break;
                case TURN_FORWARDS:
                    // turns using my PID

                    holdDrivebase();
                    outtake.IntakeSlideTo(1,outtake.IntakeSlidePos(),1);
                    outtake.turretSpin(0,outtake.turretPos(),1); // spin turret after
                    outtake.liftTo(0, outtake.liftPos(), 1);
                    //threshold is 1 inch, 2 degrees
                    if (gamepad1.a){ // have to deal with the heading here, read telemetry for heading angle
                        currentState = AutoState.TURN_OTHER_STACK;
                    }
                    break;
            }
            // Updates driving for trajectories
            drive.update();
            telemetry.update();
        }

    }
    public void holdDrivebaseOtherSide(){ // inputs the raw heading instead of corrected heading
        drivebase.DriveToPositionAutonomous2(outconestackXOtherSide,outconestackY,Math.toRadians(90),xPosition,yPosition,offsetHeading,headingPosition, 1,1); // last values are translationalspeed, and rotational speed
    }
    public void holdDrivebase(){ // inputs the raw heading instead of corrected heading
        drivebase.DriveToPositionAutonomous2(outconestackXOtherSide,outconestackY,Math.toRadians(-90),xPosition,yPosition,offsetHeading,headingPosition, 1,1); // last values are translationalspeed, and rotational speed
    }
}


