package org.firstinspires.ftc.teamcode.Sandstorm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class GlobalsFarHighAuto {

    // encoder positions
    public static int IntakeSlideOutTicks = -745; // this is max
    public static int LiftHighPosition = -695;
    public static int LiftHighPosition10 = -760; // not needed - this is to make lift faster for 1+10

    public static double TurretLeftposition = -5.55;
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

    public static double PreloadDriveX = 34;
    public static double PreloadDriveY = -17.7;
    public static double PreloadDriveRotation = Math.toRadians(0);

    public static double inconestackX = 35;
    public static double inconestackY = -17.7;
    public static double inStackRotation = Math.toRadians(-1.2);

    public static double outconestackX = 16;
    public static double outconestackY = -17.7;
    public static double outconeStackRotation = Math.toRadians(-1.2);

    public static double parkLeft = -61;
    public static double parkRight = -11.5;
    public static double parkCentre = -35;

    // starting position
    public Pose2d startPose = new Pose2d(34, -69, Math.toRadians(0));
}
