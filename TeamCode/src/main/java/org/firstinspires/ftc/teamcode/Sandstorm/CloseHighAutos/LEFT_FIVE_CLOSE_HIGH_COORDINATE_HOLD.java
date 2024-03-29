package org.firstinspires.ftc.teamcode.Sandstorm.CloseHighAutos;
import static org.firstinspires.ftc.teamcode.Sandstorm.CloseHighAutos.GlobalsCloseHighAuto.outconestackY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Sandstorm.AutoTest.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Sandstorm.DriveBase;
import org.firstinspires.ftc.teamcode.Sandstorm.FarHighAutos.RIGHT_FIVE_FAR_HIGH;
import org.firstinspires.ftc.teamcode.Sandstorm.HighSpeedCamera;
import org.firstinspires.ftc.teamcode.Sandstorm.Inputs;
import org.firstinspires.ftc.teamcode.Sandstorm.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;


@Autonomous(name = "Left 1+5 Close-High Auto", group = "Autonomous")
public class LEFT_FIVE_CLOSE_HIGH_COORDINATE_HOLD extends LinearOpMode {

    GlobalsCloseHighAuto globalsCloseHighAuto = new GlobalsCloseHighAuto();
    int SideMultiplier = -1; // this multiplies everything that changes with the right side
    double AngleOffset = Math.toRadians(180); // this adds to every angle
    String webcamname = globalsCloseHighAuto.WebCamRightName; // this is the webcam name for the right or left

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;
    boolean goToPark;
    HighSpeedCamera highSpeedCamera;

    int numCycles;
    int SignalRotation;
    int slowerVelocityConstraint;

    double correctedHeading;
    double xPosition;
    double yPosition;
    double headingPosition;
    double offsetHeading;
    double dt;
    double prev_time;

    // create class instances
    Outtake outtake = new Outtake();
    DriveBase drivebase = new DriveBase();
    Inputs inputs = new Inputs();
    SampleMecanumDrive drive;



    enum AutoState {
        DELAY,
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
        outtake.hardwareSetup();
        drivebase.motorsSetup();
        inputs.inputsSetup(); // hopefully won't conflict
        currentState = AutoState.DELAY; // this go here?
        autoTimer = 0;
        numCycles = 0;
        slowerVelocityConstraint = 12;
        outtake.encodersReset();
        goToPark = true;
    }
    // Define our start pose

    Pose2d startPose = new Pose2d(globalsCloseHighAuto.startPoseX * SideMultiplier, globalsCloseHighAuto.startPoseY, globalsCloseHighAuto.startPoseAngle + AngleOffset);
    Pose2d stackPosition = new Pose2d(68*SideMultiplier, -12, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();




        // initialize hardware
        outtake.Outtake_init(hardwareMap);
        drivebase.Drivebase_init(hardwareMap); // this might conflict with road runner
        drive = new SampleMecanumDrive(hardwareMap); // road drive class

        // functions runs on start
        Setup();
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        Trajectory PreloadDrive = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(globalsCloseHighAuto.outconestackX*SideMultiplier, globalsCloseHighAuto.outconestackY, globalsCloseHighAuto.outconeStackRotation * SideMultiplier + AngleOffset))
                .build();

        Trajectory ParkRight = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(SideMultiplier == 1 ? -GlobalsCloseHighAuto.parkLeft: GlobalsCloseHighAuto.parkRight, outconestackY))
                .build();

        Trajectory ParkLeft = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(SideMultiplier == 1 ? -GlobalsCloseHighAuto.parkRight: GlobalsCloseHighAuto.parkLeft, outconestackY))
                .build();

        Trajectory ParkCentre = drive.trajectoryBuilder(PreloadDrive.end())
                .lineTo(new Vector2d(-globalsCloseHighAuto.parkCentre * SideMultiplier, globalsCloseHighAuto.outconestackY))
                .build();

        while (!isStarted()) {


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


        // runs instantly once
        autoTimer = GlobalTimer.milliseconds();
        GlobalTimer.reset();

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
            offsetHeading = inputs.offsetAngle90(headingPosition);
            outtake.outtakeReads();
            outtake.ConeArmReady();
            telemetry.addData("autoState", currentState);

            // main switch statement logic
            switch (currentState) {
                case DELAY:
                    outtake.OuttakeClawClose();
                    outtake.IntakeClipOpen();
                    outtake.OuttakeArmReady();
                    if (GlobalTimer.milliseconds() - autoTimer > 6000){
                        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                        currentState = AutoState.PRELOAD_DRIVE;
                        drive.followTrajectoryAsync(PreloadDrive);
                    }
                    break;
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
                        //outtake.OuttakeArmScoreAuto();
                    }
                    break;

                case OUTTAKE_CONE:
                    holdDrivebasePosition(poseEstimate);
                    if (true){
                        OuttakeCone(true,poseEstimate); // next state is grab off
                    }
                    break;

                case GRAB_OFF_STACK:
                    holdDrivebasePosition(poseEstimate); // dropping cone
                    dropCone(350,poseEstimate);
                    if (GlobalTimer.milliseconds() - autoTimer < 350){
                        outtake.liftToInternalPID(GlobalsCloseHighAuto.LiftHighPosition, 1);
                        holdTurretPosition(poseEstimate,1);
                    }

                    if (GlobalTimer.milliseconds() - autoTimer > 370) { // time taken to drop cone
                        if ((outtake.intakeClawTouchPressed() || GlobalTimer.milliseconds() - autoTimer > 600) && (drivebase.getHeadingError() < Math.toRadians(2))){ // outtake.intakeClawTouchPressed() || GlobalTimer.milliseconds() - autoTimer > 680
                            outtake.IntakeClawClose();
                            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                        } else {
                            outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideOutTicks, 0.9); // slower
                        }
                    } else {
                        outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideNotQuiteOutTicks, 1); // slower
                    }

                    break;
                case AFTER_GRAB_OFF_STACK: // grabs off the stack
                    outtake.OuttakeClawOpen();
                    outtake.OuttakeArmReady();
                    outtake.BraceReady();
                    holdDrivebasePosition(poseEstimate);
                    outtake.turretSpinInternalPID(0,1);
                    outtake.liftToInternalPID(2, 1);
                    if (GlobalTimer.milliseconds() - autoTimer > 0){
                        outtake.IntakeClawClose();
                        if (GlobalTimer.milliseconds() - autoTimer > 210){
                            if (outtake.intakeLiftPosition > 275){
                                outtake.IntakeArmTransfer();
                                if ((numCycles==1? outtake.intakeArmPosition > 175: outtake.intakeArmPosition > 150) && GlobalTimer.milliseconds()-autoTimer > 600){ // this reads the position of the intake arm
                                    outtake.IntakeSlideInternalPID(7,1);
                                    outtake.IntakeLiftTransfer();
                                    if (outtake.intakeSlidePosition > 0 && outtake.intakeArmPosition > 195){ // this controls when the claw closes
                                        autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                                        currentState = AutoState.TRANSFER_CONE;
                                        outtake.OuttakeClawClose();
                                        outtake.BraceActive();
                                    }
                                } else{
                                    outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideBackFromStack, 0.42); // this pulls slides in while doing stuff
                                }
                            } else {
                                outtake.IntakeLift5();
                                outtake.IntakeArmConeHoldForTransfer();
                            }
                        } else {
                            outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideBackFromStack, 0.42); // slower
                        }
                    } else {
                        outtake.IntakeSlideInternalPID(globalsCloseHighAuto.IntakeSlideOutTicks, 1); // slower
                    }
                    break;
                case TRANSFER_CONE:
                    holdDrivebasePosition(poseEstimate);
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
                    OuttakeCone(false,poseEstimate); // in a function so that i don't have to make 2 changes
                    holdDrivebasePosition(poseEstimate);

                    break;
                case RETRACT_SLIDES:
                    holdDrivebasePosition(poseEstimate);
                    dropCone(350, poseEstimate);
                    if (GlobalTimer.milliseconds() - autoTimer < 350){
                        outtake.liftToInternalPID(GlobalsCloseHighAuto.LiftHighPosition, 1);
                        holdTurretPosition(poseEstimate,1);
                    }
                    if (GlobalTimer.milliseconds() - autoTimer > 750){ // lifttarget reached doesn't work, is instantly true
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

            if ((GlobalTimer.milliseconds() > 27500) && goToPark && currentState != AutoState.IDLE){
                goToPark = false;
                currentState = AutoState.PARK;
            }

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
    public void OuttakeCone(boolean Intake, Pose2d currentPose){
        if (Intake){ // if parameter is set to false it won't do the intake slides just outtake
            outtake.IntakeSlideTo(globalsCloseHighAuto.IntakeSlideNotQuiteOutTicks,outtake.intakeSlidePosition, 1); // move to just before the stack
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

        holdTurretPosition(currentPose,1);

        if ((outtake.liftPosition < globalsCloseHighAuto.LiftHighPosition+10)){
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
    public void dropCone(int waitBeforeRetract, Pose2d currentPose){
        if (GlobalTimer.milliseconds() - autoTimer > 200){ // small wait
            if (GlobalTimer.milliseconds() - autoTimer > 280){
                outtake.OuttakeClawOpenHard();
                if (GlobalTimer.milliseconds() - autoTimer > waitBeforeRetract){
                    outtake.BraceReadyAuto(); // might need a new position for this
                    outtake.liftToInternalPID(2, 1);
                    if (GlobalTimer.milliseconds() - autoTimer > 400){
                        outtake.turretSpinInternalPID(0, 0.8);
                        outtake.OuttakeSlideReady(); // drops down on pole a bit
                    } else {
                        holdTurretPosition(currentPose,1);
                    }
                }
            } else {
                outtake.OuttakeSlideScoreDrop(); // drops down on pole a bit
                outtake.OuttakeArmDeposit();
            }
        }
    }


    public void holdDrivebasePosition(Pose2d currentPose){ // THE OUTCONESTACKROTATION SHOULD BE NEGATIVE
        //drivebase.DriveToPositionAutonomous(globalsCloseHighAuto.outconestackX * SideMultiplier, globalsCloseHighAuto.outconestackY,globalsCloseHighAuto.outconeStackRotationHOLDPID* SideMultiplier,xPosition,yPosition,offsetHeading, 1,1); // last values are translationalspeed, and rotational speed
        drivebase.DriveToPositionAutonomous3(GlobalsCloseHighAuto.stackTargetX*SideMultiplier,GlobalsCloseHighAuto.stackTargetY, globalsCloseHighAuto.outconestackX * SideMultiplier, globalsCloseHighAuto.outconestackY,xPosition,yPosition,headingPosition, 1,1, telemetry, currentPose); // last values are translationalspeed, and rotational speed
    }
    public void holdTurretPosition(Pose2d currentPose, double maxOutput){
        outtake.turretPointToPole(GlobalsCloseHighAuto.poleTargetX * SideMultiplier,GlobalsCloseHighAuto.poleTargetY,currentPose,maxOutput,telemetry);
    }

}


