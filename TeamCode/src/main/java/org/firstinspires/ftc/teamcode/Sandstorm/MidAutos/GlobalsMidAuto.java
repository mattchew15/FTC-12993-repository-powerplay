package org.firstinspires.ftc.teamcode.Sandstorm.MidAutos;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class GlobalsMidAuto { // copy and paste close high autos and replace the variables with the mid ones, makes easy to tune

    // encoder positions
    public static int IntakeSlideOutTicks = -677;
    public static int LiftMidPosition = -363;

    public static double TurretLeftposition = -2.93;
    public static double TurretRightposition = -TurretLeftposition;

    public static double TurretRightPositionInternalPid = 21;

    public static int IntakeSlideNotQuiteOutTicks = IntakeSlideOutTicks + 90;
    public static int IntakeSlideBackFromStack = IntakeSlideOutTicks + 95;
    public static int IntakeSlideBackFurtherFromStack = IntakeSlideOutTicks + 87;

    // webcam names
    public String WebCamLeftName  = "WebcamLeft";
    public String WebCamRightName  = "WebcamRight";

    // coordinates

    public static double startPoseX = 34;
    public static double startPoseY = -69;
    public static double startPoseAngle = Math.toRadians(0);
    public static double outconestackX = 39.95;
    public static double outconestackY = -23;
    public static double outconeStackRotation = Math.toRadians(7.3);
    // add 180 to angles

    public static double stackTargetX = 68;
    public static double stackTargetY = -19;

    public static double poleTargetX = 25;
    public static double poleTargetY = -29.6;

    public static double parkLeft = -61;
    public static double parkRight = -12;
    public static double parkCentre = -35;
    public static double parkY = -17.6;
    public static double parkRotation = Math.toRadians(0);

    // starting position
    public Pose2d startPose = new Pose2d(34, -69, Math.toRadians(0));
}
