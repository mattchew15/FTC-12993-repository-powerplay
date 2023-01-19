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

        //reverse correct motors
        DFLM.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        DBLM.setDirection(DcMotorSimple.Direction.REVERSE);
        DBRM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}
