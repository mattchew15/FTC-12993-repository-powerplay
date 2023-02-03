package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;//command + b to search
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class DriveSubSystem extends SubsystemBase {
    private MecanumDrive drive;
    private MotorEx FR, BR, FL, BL;

    public DriveSubSystem(MotorEx frontL, MotorEx frontR, MotorEx backL, MotorEx backR){
        FR = frontR;
        BR = backR;
        FL = frontL;
        BL = backL;
        drive = new MecanumDrive(FL, FR, BL, BR);
    }
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){//: )
        drive.driveRobotCentric(-1 * strafeSpeed, forwardSpeed, turnSpeed);
    }

}
