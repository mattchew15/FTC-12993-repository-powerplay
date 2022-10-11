package org.firstinspires.ftc.teamcode;

//@disabled

import android.graphics.Paint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "EpicSlide")
public class EpicSlide extends LinearOpMode {

    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor Intake;
    private Servo CapArm;
    private Servo CapGrab;
    private DcMotor Turret;
    private Servo BucketServoLeft;
    private Servo BucketServoRight;
    private DcMotor SlidesRight;
    private DcMotor SlidesLeft;
    ColorSensor FreightDetect;
    private Servo IntakeServoL;
    private Servo IntakeServoR;
    private Servo DumpServo;
    private CRServo CarouselL;
    private CRServo CarouselR;

    BNO055IMU imu;

    Orientation angles;
    ElapsedTime GlobalTimer;
    double PowerBase;
    double PowerBaseTurn;
    double PowerStrafe;
    double CapArmUpPos;
    double CapGrabOpenPos;
    double CapGrabClosedPos;
    double CapArmScorePos;
    double CapState;
    double CapArmGrabPos;
    boolean CapTogglePressed;
    int TurretPos;
    int IntakeActiveState;
    double IntakeTimer;
    int IntakeState;
    int CarouselDirection;
    int SlidesTarget;
    double CarouselTimer;
    double IntakeLeftDownPos;
    double IntakeRightDownPos;
    double IntakeLeftUpPos;
    double IntakeRightUpPos;
    double BucketLeftDownPos;
    double BucketLeftMidPos;
    double BucketLeftUpPos;
    double BucketRightDownPos;
    double BucketRightMidPos;
    double BucketRightUpPos;
    double DumpOpenPos;
    double DumpClosedPos;
    double CapTimer;
    boolean PowerToggled;
    int SlideHeight;

    private void Setup() {
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        PowerBase = 1;
        PowerBaseTurn = 1;
        PowerStrafe = 1;
        PowerToggled = false;

        IntakeLeftDownPos = 0.85;
        IntakeRightDownPos = 0.06;
        IntakeRightUpPos = 1;
        IntakeLeftUpPos = 1;
        BucketLeftDownPos = 0.78;
        BucketLeftMidPos = 1;
        BucketLeftUpPos = 1;
        BucketRightDownPos = 0.05;
        BucketRightMidPos = 1;
        BucketRightUpPos = 1;
        DumpClosedPos = 1;
        DumpOpenPos = 0;

        CapArmUpPos = 0.7;
        CapGrabOpenPos = 0.65;
        CapGrabClosedPos = 0.35;
        CapArmScorePos = 0.5;
        CapArmGrabPos = 0.21;
        CapState = 0;
        CapTogglePressed = false;
        CapTimer = 0;
        TurretPos = 0;
        SlideHeight = 2;

        IntakeActiveState = 0;
        IntakeTimer = 0;
        IntakeState = 0;
        CarouselTimer = 0;
        CarouselDirection = 1;
        SlidesTarget = -1450;

        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
    }

    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");

        SlidesLeft = hardwareMap.get(DcMotor.class, "SlidesMotorLeft");
        SlidesRight = hardwareMap.get(DcMotor.class, "SlidesMotorRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        IntakeServoL = hardwareMap.get(Servo.class, "IntakeServoL");
        IntakeServoR = hardwareMap.get(Servo.class, "IntakeServoR");
        DumpServo = hardwareMap.get(Servo.class, "DumpServo");
        CapArm = hardwareMap.get(Servo.class, "CapArm");
        CapGrab = hardwareMap.get(Servo.class, "CapGrab");
        BucketServoLeft = hardwareMap.get(Servo.class, "BucketServoL");
        BucketServoRight = hardwareMap.get(Servo.class, "BucketServoR");
        Turret = hardwareMap.get(DcMotor.class, "TurretMotor");

        CarouselL = hardwareMap.crservo.get("CarouselServoL");
        CarouselR = hardwareMap.crservo.get("CarouselServoR");

        FreightDetect = hardwareMap.get(ColorSensor.class, "FreightDetect");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        SlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (opModeIsActive()) {

            Setup();

            while (opModeIsActive()) {



                //Strafe(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                if (gamepad1.a){
                    BL.setTargetPosition(1000);
                    BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BL.setPower(1);
                }
                else if (gamepad1.b){
                    BL.setTargetPosition(-1000);
                    BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BL.setPower(1);

                }
                telemetry.addData("Slides position", BL.getCurrentPosition());
                telemetry.update();

            }
        }
    }

    public void Strafe(double LY, double LX, double RX) {
        BL.setPower(LY * (PowerBase - (LX * PowerStrafe * 0.5)) - RX * PowerBaseTurn + LX * PowerStrafe);
        FL.setPower(LY * (PowerBase - (LX * PowerStrafe * 0.5)) - RX * PowerBaseTurn - LX * PowerStrafe);
        BR.setPower(LY * (PowerBase - (LX * PowerStrafe * 0.5)) + RX * PowerBaseTurn - LX * PowerStrafe);
        FR.setPower(LY * (PowerBase - (LX * PowerStrafe * 0.5)) + RX * PowerBaseTurn + LX * PowerStrafe);
    }

}