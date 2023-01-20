package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveBase {

    private DcMotor DBLM;
    private DcMotor DBRM;
    private DcMotor DFLM;
    private DcMotor DFRM;

    boolean PowerToggled;
    double PowerBase;
    double PowerBaseTurn;
    double PowerStrafe;

    public void Drivebase_init(HardwareMap hwMap){

        DBLM = hwMap.get(DcMotor.class, "BL");
        DBRM = hwMap.get(DcMotor.class, "BR");
        DFLM = hwMap.get(DcMotor.class, "FL");
        DFRM = hwMap.get(DcMotor.class, "FR");

    }

    public void motorsSetup(){
        // zero brake behavior means when motors aren't powered, they will auto brake
        DFRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DBRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DBLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DFLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse correct motors (reversing cause of motor/wheels might need to adjust)
        DFLM.setDirection(DcMotorSimple.Direction.REVERSE);
        DBLM.setDirection(DcMotorSimple.Direction.REVERSE);
        DBRM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Drive(double LY, double LX, double RX) {
        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (LY * PowerBase - LX * PowerStrafe + RX * PowerBaseTurn) / denominator;
        double backLeftPower = (LY * PowerBase + LX * PowerStrafe + RX * PowerBaseTurn) / denominator;
        double frontRightPower = (LY * PowerBase + LX * PowerStrafe - RX * PowerBaseTurn) / denominator;
        double backRightPower = (LY * PowerBase - LX * PowerStrafe - RX * PowerBaseTurn) / denominator;

        DFLM.setPower(frontLeftPower);
        DBLM.setPower(backLeftPower);
        DFRM.setPower(frontRightPower);
        DBRM.setPower(backRightPower);
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
