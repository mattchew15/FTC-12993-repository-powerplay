package org.firstinspires.ftc.teamcode.Sandstorm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class GlobalsCloseHighAuto {

    // encoder positions
    public static int IntakeSlideOutTicks = -600;
    public static int LiftHighPosition = -700;
    public static int LiftHighPosition10 = -775;

    public static double TurretLeftposition = -5.55;
    public static double TurretRightposition = -TurretLeftposition;

    public static int IntakeSlideNotQuiteOutTicks = IntakeSlideOutTicks + 88;
    public static int IntakeSlideBackFromStack = IntakeSlideOutTicks + 69;

    // webcam names
    public String WebCamLeftName  = "WebcamLeft";
    public String WebCamRightName  = "WebcamRight";

    // coordinates

    public static double startPoseX = 34;
    public static double startPoseY = -69;
    public static double startPoseAngle = Math.toRadians(0);
    public static double outconestackX = 42;
    public static double outconestackY = -17.7;
    public static double outconestackYOtherSide = -17.3;
    public static double outconeStackRotation = Math.toRadians(-1.2);
    // add 180 to angles

    public static double outconestackXOtherSide = -40.8;
    public static double outconeStackRotationOtherSide = Math.toRadians(170);

    public static double parkLeft = -61;
    public static double parkRight = -11.5;
    public static double parkCentre = -35;

    // starting position
    public Pose2d startPose = new Pose2d(34, -69, Math.toRadians(0));
}
