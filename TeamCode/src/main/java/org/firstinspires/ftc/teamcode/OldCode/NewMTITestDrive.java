package org.firstinspires.ftc.teamcode;

//@disabled

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "NEwMTITestDrive")
public class NewMTITestDrive extends LinearOpMode {

    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor Intake;
    private Servo CapArm;
    private Servo CapGrab;

    double PowerBase;
    double PowerBaseTurn;
    double PowerStrafe;
    double CapArmUpPos;
    double CapGrapOpenPos;
    double CapGrapClosedPos;
    double CapArmScorePos;
    double CapState;
    double CapArmGrabPos;
    double BtnState;
    boolean CapTogglePressed;


    private void Setup() {
      //FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        PowerBase = 1;
        PowerBaseTurn = 0.55;
        PowerStrafe = 1;
        CapArmUpPos = 0.74;
        CapGrapOpenPos = 0.65;
        CapGrapClosedPos = 0.38;
        CapArmScorePos = 0.58;
        CapArmGrabPos = 0.25;
        CapState = 0;
        BtnState = 0;
        CapTogglePressed = false;
    }

    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        CapArm = hardwareMap.get(Servo.class, "CapArm");
        CapGrab = hardwareMap.get(Servo.class, "CapGrab");


        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CapArm.setPosition(CapArmUpPos);

        waitForStart();
        if (opModeIsActive()) {

            Setup();

            while (opModeIsActive()) {

                Capping(gamepad1.dpad_down);
                Strafe(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                Intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                telemetry.update();
                }
            }
        }

    public void Strafe(float LY, float LX, float RX) {
        BL.setPower(LY * (PowerBase - (LX*0.5)) - RX + LX * PowerBaseTurn);
        FL.setPower(LY * (PowerBase - (LX*0.5)) - RX - LX * PowerBaseTurn);
        BR.setPower(LY * (PowerBase - (LX*0.5)) + RX - LX * PowerBaseTurn);
        FR.setPower(LY * (PowerBase - (LX*0.5)) + RX + LX * PowerBaseTurn);
    }

    public void Capping(boolean Cap){

        if (Cap) {
            if (!CapTogglePressed) {
                CapTogglePressed = true;
                CapState = (CapState + 1) % 4;
            }
        }
        else {
            CapTogglePressed = false;
        }

        if (CapState == 0){
            CapArm.setPosition(CapArmGrabPos);
            CapGrab.setPosition(CapGrapOpenPos);
            telemetry.addData("Capstate", CapState);
        }
        else if (CapState == 1){
            CapGrab.setPosition(CapGrapClosedPos);
            telemetry.addData("Capstate", CapState);
        }
        else if (CapState == 2) {
            CapGrab.setPosition(CapGrapClosedPos);
            CapArm.setPosition(CapArmScorePos);
            telemetry.addData("Capstate", CapState);
        }
        else if (CapState == 3){
            CapGrab.setPosition(CapGrapOpenPos);
            CapArm.setPosition(CapArmUpPos);
            telemetry.addData("Capstate", CapState);
        }
    }
}


