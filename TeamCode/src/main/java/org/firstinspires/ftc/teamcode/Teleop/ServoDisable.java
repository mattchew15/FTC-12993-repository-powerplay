package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo Disable")
public class ServoDisable extends LinearOpMode {

    TurretLift turretLift = new TurretLift();

    public void runOpMode(){

        turretLift.releaseClaw();
        turretLift.linkageRelease();

        waitForStart();
        while (opModeIsActive()){

            turretLift.releaseClaw();
            turretLift.linkageRelease();

        }
    }
}
