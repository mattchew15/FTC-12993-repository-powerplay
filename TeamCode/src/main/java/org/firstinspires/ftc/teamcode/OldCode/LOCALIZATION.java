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

@TeleOp(name = "LOCALIZATION")
public class LOCALIZATION extends LinearOpMode {

    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    public double GlobalX;
    public double GlobalY;
    public double GlobalAngle;

    ElapsedTime GlobalTimer;
    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();


        waitForStart();
        if (opModeIsActive()) {

            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive()) {

                double BLpos = BL.getCurrentPosition() * 0.03 * -1;
                double BRpos = BR.getCurrentPosition() * 0.03 * -1;
                double FLpos = FL.getCurrentPosition() * 0.03 * -1;
                double FRpos = FR.getCurrentPosition() * 0.03 * -1;

                GlobalX = (-1 * (BLpos - FLpos + FRpos - BRpos));
                GlobalY = (BLpos + FLpos + FRpos + BRpos);
                GlobalAngle = ((FLpos + BLpos) - (FRpos + BRpos));

                telemetry.addData("Global X", GlobalX);
                telemetry.addData("Global Y", GlobalY);
                telemetry.addData("Global Angle", GlobalAngle);

                telemetry.addData("BLpos", BLpos);
                telemetry.addData("BRpos", BRpos);
                telemetry.addData("FLpos", FLpos);
                telemetry.addData("FRpos", FRpos);

                Strafe(gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x * 0.6);
                telemetry.update();

            }
        }
    }

    public void Strafe(double LY, double LX, double RX) {

        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (LY + LX + RX) / denominator;
        double backLeftPower = (LY - LX + RX) / denominator;
        double frontRightPower = (LY - LX - RX) / denominator;
        double backRightPower = (LY + LX - RX) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }


}