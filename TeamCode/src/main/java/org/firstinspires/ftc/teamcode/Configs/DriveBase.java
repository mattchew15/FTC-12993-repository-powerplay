package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveBase {

    private DcMotorEx DBLM;
    private DcMotorEx DBRM;
    private DcMotorEx DFLM;
    private DcMotorEx DFRM;

    public void Drivebase_init(HardwareMap hwMap){

        DBLM = hwMap.get(DcMotorEx.class, "BL");
        DBRM = hwMap.get(DcMotorEx.class, "BR");
        DFLM = hwMap.get(DcMotorEx.class, "FL");
        DFRM = hwMap.get(DcMotorEx.class, "FR");

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

}
