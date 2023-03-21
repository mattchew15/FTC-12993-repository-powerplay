package org.firstinspires.ftc.teamcode.Dune.Autonomous.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class NewBNO055IMU extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    double roll;
    boolean tipp;

    @Override
    public void runOpMode() {

        //we can use more than just tipping, see external samples -> SensorBNO055IMU.java

        //defining needed parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        //init of imu and parameters
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {

            //setting angles value using IMU integrated code to use variable in next calculation
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //calculating the angle aka roll through multiple angles (look in source code if you want to know)
            roll = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));

            //setting boolean
            tipp = roll > -80 || roll < -82;

            //telemetry
            telemetry.addData("tipping", Boolean.toString(tipp));
            telemetry.addData("roll", Double.toString(roll));
            telemetry.update();
        }
    }
}
