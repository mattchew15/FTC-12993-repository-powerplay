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

@TeleOp(name = "MTIDrive")
public class MTIDrive extends LinearOpMode {

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
    double manual;

    private void Setup() {
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        PowerBase = 1;
        PowerBaseTurn = 1;
        PowerStrafe = 1;
        PowerToggled = false;

        IntakeLeftDownPos = 0.85;
        IntakeRightDownPos = 0.06;
        IntakeRightUpPos = 1;
        IntakeLeftUpPos = 1;
        BucketLeftDownPos =0.78;
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
        CapArmScorePos = 0.54;
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
        SlidesTarget = -1400;
        manual = 0;

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

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (opModeIsActive()) {

            Setup();

            while (opModeIsActive()) {

                Capping(gamepad1.left_bumper);
                Strafe(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                PowerToggle(gamepad1.left_stick_button);
                TurretSwivel(gamepad2.dpad_left, gamepad2.x, gamepad2.a, gamepad2.b , gamepad2.dpad_right, gamepad2.left_trigger, gamepad2.right_trigger, gamepad2.right_stick_button);

                telemetry.addData("distance front", frontDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("distance left", leftDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("distance right", rightDistance.getDistance(DistanceUnit.CM));
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("gyro", angles.firstAngle);

                telemetry.addData("Turret", Turret.getCurrentPosition());

                Intake(gamepad2.left_bumper);
                SlideHeightToggle(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.a, gamepad2.left_stick_y);
                telemetry.addData("SlidesLeft Position", SlidesLeft.getCurrentPosition());
                telemetry.addData("SlidesRight Position", SlidesRight.getCurrentPosition());
                telemetry.addData("Turret Position", Turret.getCurrentPosition());
                Carousel(gamepad1.dpad_left, gamepad1.dpad_right);
                telemetry.addData("Red Colour", FreightDetect.red());
                telemetry.addData("turretmanual", manual);

                telemetry.update();

            }
        }
    }

    public void Strafe(double LY, double LX, double RX) {
        BL.setPower(LY * (PowerBase - (LX * PowerStrafe*0.5)) - RX * PowerBaseTurn + LX * PowerStrafe);
        FL.setPower(LY * (PowerBase - (LX * PowerStrafe*0.5)) - RX * PowerBaseTurn - LX * PowerStrafe);
        BR.setPower(LY * (PowerBase - (LX * PowerStrafe*0.5)) + RX * PowerBaseTurn - LX * PowerStrafe);
        FR.setPower(LY * (PowerBase - (LX * PowerStrafe*0.5)) + RX * PowerBaseTurn + LX * PowerStrafe);
    }

    public void PowerToggle(boolean toggle) {
        if (toggle) {
            if (!PowerToggled) {
                if (PowerBase == 1) {
                    PowerBase = 0.33;
                    PowerBaseTurn = 0.33;
                    PowerStrafe = 0.33;
                } else {
                    PowerBase = 1;
                    PowerBaseTurn = 1;
                    PowerStrafe = 1;
                }
                PowerToggled = true;
            }
        }
        else {
            PowerToggled = false;
        }
    }

    public void Capping(boolean button){

        if (button) {
            if (!CapTogglePressed) {
                CapTogglePressed = true;
                CapState = (CapState + 1) % 7;
            }
        }
        else {
            CapTogglePressed = false;
        }

        if (CapState == 0) {
            CapGrab.setPosition(CapGrabOpenPos);
            CapArm.setPosition(CapArmUpPos);
            telemetry.addData("Capstate", CapState);
        }
        if (CapState == 1) {
            CapArm.setPosition(CapArmGrabPos);
            telemetry.addData("Capstate", CapState);
        }
        else if (CapState == 2) {
            CapGrab.setPosition(CapGrabClosedPos);
            telemetry.addData("Capstate", CapState);
            CapTimer = GlobalTimer.milliseconds();
            CapState = 3;

        }
        else if (CapState == 3) {
            if (GlobalTimer.milliseconds() - CapTimer > 150) {
                CapArm.setPosition(CapArmScorePos);
            }
            telemetry.addData("Capstate", CapState);
        }
        else if (CapState == 4) {
            CapArm.setPosition(CapArmScorePos - 0.07);
            CapTimer = GlobalTimer.milliseconds();
            CapState = 5;
        }
        else if (CapState == 5) {
            if (GlobalTimer.milliseconds() - CapTimer > 175) {
                CapGrab.setPosition(CapGrabOpenPos);
                CapTimer = GlobalTimer.milliseconds();
                CapState = 6;
            }
        }
        else if (CapState == 6) {
            if (GlobalTimer.milliseconds() - CapTimer > 300) {
                CapState = 0;
            }
        }
    }

    public void TurretSwivel(boolean sharedleft, boolean allianceleft, boolean centre, boolean allianceright, boolean sharedright, float turretmanualleft, float turretmanualright ,boolean turretreset) {
        manual = turretmanualleft - turretmanualright;

        if (sharedleft) {
            Turret.setTargetPosition(-200);
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(1);
            TurretPos = 1;
        }
        else if (allianceleft) {
            Turret.setTargetPosition(-160);
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(1);
            TurretPos = 2;
        }
        else if (centre) {
            Turret.setTargetPosition(0);
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(1);
            TurretPos = 3;
        }
        else if (allianceright) {
            Turret.setTargetPosition(160);
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(1);
            TurretPos = 4;
        }
        else if (sharedright) {
            Turret.setTargetPosition(200);
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(1);
            TurretPos = 5;
        }

        else if (turretreset){
            Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        else if (manual != 0) {
            Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turret.setPower(manual * 0.4);
        }



    }

    public void Intake(boolean Button) {

        if (Button) {
            if (IntakeActiveState == 0) {
                IntakeActiveState = 1; // Intake on, holding down
                IntakeState = 1;
            }
            if (IntakeActiveState == 2) {
                IntakeActiveState = 3; // Intake off, holding down
                IntakeState = 0;
            }
        }
        else {
            if (IntakeActiveState == 1) {
                IntakeActiveState = 2; // Intake on, let go
            }
            if (IntakeActiveState == 3) {
                IntakeActiveState = 0; // Intake off, let go
            }
        }

        if (IntakeState == 0) {
            IntakeServoL.setPosition(0.9);
            IntakeServoR.setPosition(0.01); // Set everything to initial position
            BucketServoRight.setPosition(0.05); // Retract Bucket
            DumpServo.setPosition(0.85);
            BucketServoLeft.setPosition(0.78);
            Intake.setPower(0);
            SlidesRight.setTargetPosition(2);
            SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlidesRight.setPower(1);
            SlidesLeft.setTargetPosition(2);
            SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlidesLeft.setPower(1);
            telemetry.addData("State", "case0");
        }
        else if (IntakeState == 1) {
            Intake.setPower(1); // Spin intake',
            if (FreightDetect.red() > 200 || gamepad2.right_bumper) { // When freight detected
                IntakeServoL.setPosition(0.39); // Flip up intakes
                IntakeServoR.setPosition(0.52);
                IntakeTimer = GlobalTimer.milliseconds(); // Start timer
                IntakeState = 2; // Switch to state 2
                telemetry.addData("State", "case1");
                gamepad1.rumble(1);
                gamepad2.rumble(1);
            }
        }
        else if (IntakeState == 2) {
            if ((GlobalTimer.milliseconds() - IntakeTimer > 400) || gamepad2.y) { // After 0.5 seconds
                Intake.setPower(-1); // Reverse intake
                IntakeState = 3; // Switch to state 3
                if (TurretPos == 1 || TurretPos == 5 || TurretPos == 3) {
                    SlidesTarget = 2;
                    SlideHeight = 0;
                }
            }
            else{
                Intake.setPower(0); // makes servo slow for some reason
            }
        }
        else if (IntakeState == 3) {
            if (GlobalTimer.milliseconds() - IntakeTimer > 900) { // After another 0.75 seconds
                Intake.setPower(0); // Stop spinning intake
                if (SlideHeight == 2){
                    BucketServoRight.setPosition(0.53); // Extend bucket
                    BucketServoLeft.setPosition(0.3);
                }
                else if (SlideHeight == 0 || SlideHeight == 1) { // bucket position for mid might be wrong
                    BucketServoRight.setPosition(0.73); // Extend bucket
                    BucketServoLeft.setPosition(0.1);
                }
                IntakeServoL.setPosition(0.85);
                IntakeServoR.setPosition(0.06);
                DumpServo.setPosition(0.7);
                SlidesRight.setTargetPosition(SlidesTarget);
                SlidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlidesRight.setPower(1); // Send slides to max
                SlidesLeft.setTargetPosition(SlidesTarget);
                SlidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlidesLeft.setPower(1); // Send slides to max

                if (gamepad1.right_bumper) {
                    IntakeState = 4; // Go to state 4
                    IntakeTimer = GlobalTimer.milliseconds(); // Reset timer
                }
            }
        }
        else if (IntakeState == 4) {
            DumpServo.setPosition(0.85);
            if (GlobalTimer.milliseconds() - IntakeTimer > 500) { // After 0.5 seconds
                IntakeState = 0;
                IntakeActiveState = 0;
            }
        }
    }

    public void SlideHeightToggle(boolean max, boolean half, boolean min, double manual) {
        if (max) {
            SlidesTarget = -1400;
            SlideHeight = 2;
        }
        else if (half) {
            SlidesTarget = -900;
            SlideHeight = 1;
        }
        else if (min) {
            SlidesTarget = 2;
            SlideHeight = 0;
        }

        SlidesTarget += 18 * manual;
    }

    public void Carousel(boolean right, boolean left) {

        if (right) {
            CarouselDirection = 1;
        }
        else if (left) {
            CarouselDirection = -1;
        }
        else {
            CarouselDirection = 0;
            CarouselTimer = 0;
            CarouselL.setPower(0);
            CarouselR.setPower(0);
        }
        if (CarouselDirection != 0) {
            if (CarouselTimer == 0) {
                CarouselTimer = GlobalTimer.milliseconds();
            }
            else {
                if (GlobalTimer.milliseconds() - CarouselTimer < 400) {
                    CarouselL.setPower(0.4 * CarouselDirection);
                    CarouselR.setPower(0.4 * CarouselDirection);
                }
                else if (GlobalTimer.milliseconds() - CarouselTimer < 1200) {
                    CarouselL.setPower(CarouselDirection * (0.4 + 0.6 * (((GlobalTimer.milliseconds() - CarouselTimer) - 400) / 800)));
                    CarouselR.setPower(CarouselDirection * (0.4 + 0.6 * (((GlobalTimer.milliseconds() - CarouselTimer) - 400) / 800)));
                }
                else {
                    CarouselL.setPower(CarouselDirection);
                    CarouselR.setPower(CarouselDirection);
                }
            }
        }
    }
}