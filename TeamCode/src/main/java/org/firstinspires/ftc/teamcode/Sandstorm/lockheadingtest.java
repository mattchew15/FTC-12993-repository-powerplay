import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class lockheadingtest extends LinearOpMode {
    Pose2d target = new Pose2d(0, 0, 0);
    public static double kP = 0.7;
    public static double kF = 0.08;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        drive.setPoseEstimate(target);

        while(opModeIsActive()) {
            double error = pointToTarget(target, drive.getPoseEstimate());

            if(gamepad1.a) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, kP*error + kF*Math.signum(error)));
            }

            telemetry.addData("Current robot heading: ", drive.getPoseEstimate().getHeading());
            telemetry.addData("Current robot heading wrapped: ", Angle.normDelta(drive.getPoseEstimate().getHeading()));
            telemetry.addData("Error: ", error);
            telemetry.addData("Error (Deg): ", Math.toDegrees(error));
            telemetry.update();
            drive.update();
        }
    }

    public double pointToTarget(Pose2d target, Pose2d current) {
        Vector2d difference = target.vec().minus(current.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();
        double targetTheta = theta - current.getHeading();

        return Angle.normDelta(targetTheta);
    }
}