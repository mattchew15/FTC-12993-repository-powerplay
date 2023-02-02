package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;//command + b to search
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class DriveSubSystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor FR, BR, FL, BL;

    public DriveSubSystem(MotorEx frontR, MotorEx backR, MotorEx frontL, MotorEx backL){
        FR = frontR;
        BR = backR;
        FL = frontL;
        BL = backL;
        drive = new MecanumDrive(FR, BR, FL, BL);
    }
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

}
