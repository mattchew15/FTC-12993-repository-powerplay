package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.Dune.PID;

public class PositionHoldPID {
    public static double DrivebaseXKp = 0.22, DrivebaseXKi = 0.00, DrivebaseXKd = 0.018, DrivebaseXIntegralSumLimit = 10, DrivebaseXKf = 0;
    public static double DrivebaseYKp = 0.22, DrivebaseYKi = 0.00, DrivebaseYKd = 0.018, DrivebaseYIntegralSumLimit = 10, DrivebaseYKf = 0;
    public static double DrivebaseThetaKp = 2, DrivebaseThetaKi = 0.0008, DrivebaseThetaKd = 0.024, DrivebaseThetaIntegralSumLimit = 10, DrivebaseThetaKf = 0;

    public double FL_Power;
    public double BL_Power;
    public double FR_Power;
    public double BR_Power;

    // should be able to use one instance of a drivebase pid because the x,y,z translation should all be the same
    PID drivebaseXPID = new PID(DrivebaseXKp,DrivebaseXKi,DrivebaseXKd,DrivebaseXIntegralSumLimit,DrivebaseXKf);
    PID drivebaseYPID = new PID(DrivebaseYKp,DrivebaseYKi,DrivebaseYKd,DrivebaseYIntegralSumLimit,DrivebaseYKf);
    PID drivebaseThetaPID = new PID(DrivebaseThetaKp,DrivebaseThetaKi,DrivebaseThetaKd,DrivebaseThetaIntegralSumLimit,DrivebaseThetaKf);

    public void returnMotorPowers(double xTarget, double yTarget, double thetaTarget, double xRobotPosition, double yRobotPosition, double robotTheta, double maxTranslationalSpeed, double maxRotationalSpeed){
        double x = drivebaseXPID.update(xTarget,xRobotPosition,maxTranslationalSpeed); // set a target, get the robots state, and set the max speed
        double y = drivebaseYPID.update(yTarget,yRobotPosition,maxTranslationalSpeed);
        double theta = drivebaseThetaPID.update(thetaTarget,robotTheta,maxRotationalSpeed); // this pid should probably be different
        double x_rotated = x * Math.cos(robotTheta) - y * Math.sin(robotTheta);
        double y_rotated = x * Math.sin(robotTheta) + y * Math.cos(robotTheta);

        // x, y, theta input mixing
        FL_Power = ((x_rotated + y_rotated + theta));
        BL_Power = ((x_rotated - y_rotated + theta));
        FR_Power = ((x_rotated - y_rotated - theta));
        BR_Power = ((x_rotated + y_rotated - theta));
    }
}
