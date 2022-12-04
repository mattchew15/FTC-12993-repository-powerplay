package org.firstinspires.ftc.teamcode.Autonomous;

//@disabled

import android.graphics.Paint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name = "test")
@Disabled
public class test extends LinearOpMode {

    private DcMotor FR;

    private void Setup() {
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "FR");

        waitForStart();
        if (opModeIsActive()) {

            Setup();

            while (opModeIsActive()) {
            //FR.setPower(gamepad1.left_trigger);
            if (gamepad1.a){
                FR.setTargetPosition(1000);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setPower(1);
            }

            }
        }
    }



}