package org.firstinspires.ftc.teamcode.Sandstorm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class GlobalsFarHighAuto {

    // encoder positions
    public static int IntakeSlideOutTicks = -745; // this is max
    public static int LiftHighPosition = -718;
    public static int LiftHighPosition10 = -770; // could be used - this is to make lift faster for 1+10

    public static double TurretLeftposition = -5.6;
    public static double TurretRightposition = -TurretLeftposition; // use turret right position

    public static double TurretLeftPositionInternalPid = -21;
    public static double TurretRightPositionInternalPid = -TurretLeftPositionInternalPid;

    // webcam names
    public String WebCamLeftName  = "WebcamLeft";
    public String WebCamRightName  = "WebcamRight";

    // coordinates
    public static double startPoseX = 34;
    public static double startPoseY = -69;
    public static double startPoseAngle = Math.toRadians(0);

    public static double PreloadDriveX = 38.5;
    public static double PreloadDriveY = -17.7;
    public static double PreloadDriveRotation = Math.toRadians(0);

    public static double inconestackX = 37.37;
    public static double inconestackY = -18.1;
    public static double inStackRotation = Math.toRadians(1);

    public static double outconestackX = 17;
    public static double outconestackY = -18.7;
    public static double outconeStackRotation = Math.toRadians(1.5);
    public static double xValueBeforeSlidesExtend = 20;

    public static double parkLeft = -61;
    public static double parkRight = -11.5;
    public static double parkCentre = -35;

    // starting position
    public static Pose2d startPose = new Pose2d(34, -69, Math.toRadians(0));
    public static double slowerVelocityConstraintIn = 20;
    public static double slowerVelocityConstraintOut = 40;
}
