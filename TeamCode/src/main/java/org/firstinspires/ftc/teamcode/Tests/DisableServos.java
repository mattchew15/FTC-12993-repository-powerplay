package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configs.Intake;
import org.firstinspires.ftc.teamcode.Configs.Outake;

@TeleOp(name = "Disable Servos")
public class DisableServos extends LinearOpMode {

    Intake intake = new Intake();
    Outake outake = new Outake();

    public void runOpMode(){

        intake.Intake_init(hardwareMap);
        outake.Outake_init(hardwareMap);

        intake.inClawDisable();
        intake.inHeightDisable();
        intake.inTiltDisable();

        outake.outAxelDisable();
        outake.outBraceDisable();
        outake.outTiltDisable();
        outake.outClawDisable();

        while (opModeIsActive()){

            intake.inClawDisable();
            intake.inHeightDisable();
            intake.inTiltDisable();

            outake.outAxelDisable();
            outake.outBraceDisable();
            outake.outTiltDisable();
            outake.outClawDisable();

        }

    }
}
