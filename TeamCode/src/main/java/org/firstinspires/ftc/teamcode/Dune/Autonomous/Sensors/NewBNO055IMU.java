package org.firstinspires.ftc.teamcode.Dune.Autonomous.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
public class NewBNO055IMU extends LinearOpMode {

    private BNO055IMU imu;

    Orientation angles;
    Boolean tipping = false;
    float roll;

    @Override
    public void runOpMode(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(com.qualcomm.hardware.bosch.BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        roll = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);

        if (roll > -85){
            tipping = true;
        }
        else {
            tipping = false;
        }
        while (opModeIsActive()) {
            telemetry.addData("Tipping: ", Boolean.toString(tipping));
            telemetry.update();
        }
    }
}

