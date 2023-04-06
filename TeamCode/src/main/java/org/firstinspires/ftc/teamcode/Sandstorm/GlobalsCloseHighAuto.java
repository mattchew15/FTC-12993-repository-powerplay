package org.firstinspires.ftc.teamcode.Sandstorm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class GlobalsCloseHighAuto {

    // encoder positions
    public static int IntakeSlideOutTicks = -596;
    public static int LiftHighPosition = -704;
    public static int LiftHighPosition10 = -717;

    public static double TurretLeftposition = -5.55;
    public static double TurretRightposition = -TurretLeftposition;

    public static int IntakeSlideNotQuiteOutTicks = IntakeSlideOutTicks + 90;
    public static int IntakeSlideBackFromStack = IntakeSlideOutTicks + 66;

    // webcam names
    public String WebCamLeftName  = "WebcamLeft";
    public String WebCamRightName  = "WebcamRight";

    // coordinates

    public static double startPoseX = 34;
    public static double startPoseY = -69;
    public static double startPoseAngle = Math.toRadians(0);
    public static double outconestackX = 42;
    public static double outconestackY = -17.7;
    public static double outconestackYOtherSide = -17.2;
    public static double outconeStackRotation = Math.toRadians(-1.2);
    // add 180 to angles

    public static double outconestackXOtherSide = -outconestackX;
    public static double outconeStackRotationOtherSide = Math.toRadians(178);

    public static double parkLeft = -61;
    public static double parkRight = -11.5;
    public static double parkCentre = -35;

    // starting position
    public Pose2d startPose = new Pose2d(34, -69, Math.toRadians(0));
}
