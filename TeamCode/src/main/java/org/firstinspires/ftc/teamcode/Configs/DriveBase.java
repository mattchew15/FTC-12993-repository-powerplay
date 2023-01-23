package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveBase {

    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;

    boolean PowerToggled;
    double PowerBase;
    double PowerBaseTurn;
    double PowerStrafe;

    public void Drivebase_init(HardwareMap hwMap){

        BL = hwMap.get(DcMotor.class, "BL");
        BR = hwMap.get(DcMotor.class, "BR");
        FL = hwMap.get(DcMotor.class, "FL");
        FR = hwMap.get(DcMotor.class, "FR");

    }

    public void motorsSetup(){
        // zero brake behavior means when motors aren't powered, they will auto brake
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse correct motors (reversing cause of motor/wheels might need to adjust)
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Drive(double LY, double LX, double RX) {
        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (LY * PowerBase - LX * PowerStrafe + RX * PowerBaseTurn) / denominator;
        double backLeftPower = (LY * PowerBase + LX * PowerStrafe + RX * PowerBaseTurn) / denominator;
        double frontRightPower = (LY * PowerBase + LX * PowerStrafe - RX * PowerBaseTurn) / denominator;
        double backRightPower = (LY * PowerBase - LX * PowerStrafe - RX * PowerBaseTurn) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }

    public void PowerToggle(boolean toggle) { // toggle code for a slow drive mode for fine adjustment
        if (toggle) {
            if (!PowerToggled) {
                if (PowerBase == 1) {
                    PowerBase = 0.33;
                    PowerBaseTurn = 0.3;
                    PowerStrafe = 0.36;
                } else {
                    //edit these values to change drivecode
                    PowerBase = 1;
                    PowerBaseTurn = 0.65;
                    PowerStrafe = 1.05;
                }
                PowerToggled = true;
            }
        }
        else {
            PowerToggled = false;
        }
    }



}
