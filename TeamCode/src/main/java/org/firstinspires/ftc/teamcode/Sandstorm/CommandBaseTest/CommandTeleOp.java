package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.DriveCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.IntakeSlidesClawCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.LiftMotorToPostionCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.MotorCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.ServoCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.DriveSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeArmSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeClawSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeSlideMotorSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeSlideSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.LiftMotorSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.MotorSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.ServoSubSystem;

@TeleOp(name = "CommandTeleOp")
public class CommandTeleOp extends CommandOpMode {//initalize hardware and subsystems and scedules commands :)

    private MotorEx FR, BR, FL, BL, IntakeMotor, LiftMotor, IntakeSlideMotor, IntakeLift, IntakeClaw;
    private DriveSubSystem driveSubSystem;
    private DriveCommand driveCommand;
    private MotorSubSystem motorSubSystem;
    private MotorCommand motorCommand;
    private LiftMotorSubSystem liftMotorSubSystem;
    private LiftMotorToPostionCommand liftMotorToPostionCommand;
    private IntakeSlideMotorSubSystem intakeSlideMotorSubSystem;
    private ServoSubSystem servoSubSystem;
    private ServoCommand servoCommand;
    private IntakeSlideSubSystem intakeSlideSubSystem;
    private IntakeClawSubSystem intakeClawSubSystem;
    private IntakeArmSubSystem intakeArmSubSystem;
    private IntakeSlidesClawCommand intakeSlidesClawCommand;
    public boolean liftMotorToPosActive = false, servoComActive = false, motorComActive = false;
    private GamepadEx Gamepadex1;
    private GamepadEx Gamepadex2;


    @Override
    public void initialize() {
        FR = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435);
        BR = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435);
        FL = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435);
        BL = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435);//if gobilda motor add new prametter GoBILDA
        IntakeMotor = new MotorEx(hardwareMap, "IntakeMotor", Motor.GoBILDA.RPM_312);
        LiftMotor = new MotorEx(hardwareMap, "LiftMotor", Motor.GoBILDA.RPM_312);
        IntakeSlideMotor = new MotorEx(hardwareMap, "IntakeSlideMotor", Motor.GoBILDA.RPM_312);

        FL.setInverted(true);
        Gamepadex1 = new GamepadEx(gamepad1);
        Gamepadex2 = new GamepadEx(gamepad2);

        //motors
        driveSubSystem = new DriveSubSystem(FL, FR, BL, BR);//FR BR FL BL
        driveCommand = new DriveCommand(driveSubSystem, Gamepadex1::getLeftX, Gamepadex1::getLeftY, Gamepadex1::getRightX);
        motorSubSystem = new MotorSubSystem(IntakeMotor);
        motorCommand = new MotorCommand(motorSubSystem, Gamepadex1);
        liftMotorSubSystem = new LiftMotorSubSystem(LiftMotor);
        liftMotorToPostionCommand = new LiftMotorToPostionCommand(liftMotorSubSystem, Gamepadex1);
        intakeSlideMotorSubSystem = new IntakeSlideMotorSubSystem(IntakeSlideMotor);
        //servos
        servoSubSystem = new ServoSubSystem(hardwareMap);
        servoCommand = new ServoCommand(servoSubSystem, Gamepadex1);
        intakeSlideSubSystem = new IntakeSlideSubSystem(hardwareMap);
        intakeClawSubSystem = new IntakeClawSubSystem(hardwareMap);
        intakeArmSubSystem = new IntakeArmSubSystem(hardwareMap);
        intakeSlidesClawCommand = new IntakeSlidesClawCommand(intakeSlideSubSystem, intakeSlideMotorSubSystem, intakeArmSubSystem, intakeClawSubSystem, Gamepadex1, Gamepadex2, Gamepadex2::getLeftY);


        if(!liftMotorToPostionCommand.isActive) {
            Gamepadex1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(motorCommand);
            Gamepadex1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(servoCommand);
            Gamepadex1.getGamepadButton(GamepadKeys.Button.A).whenPressed(liftMotorToPostionCommand);
            telemetry.addData("Target postion", liftMotorToPostionCommand.TargetPostion);
            telemetry.addData("current postion", liftMotorToPostionCommand.LiftMotorPostion);
            telemetry.addData("is active command", liftMotorSubSystem.isActiveCommand);
            telemetry.addData("is finished run", liftMotorSubSystem.isFinishedRun);
            telemetry.addData("is active", liftMotorToPostionCommand.isActive);
        }
        driveSubSystem.setDefaultCommand(driveCommand);
    }
    @Override
    public void run(){
        super.run();
        telemetry.addData("Target postion", liftMotorToPostionCommand.TargetPostion);
        telemetry.addData("current postion", liftMotorToPostionCommand.LiftMotorPostion);
        telemetry.addData("is active command", liftMotorSubSystem.isActiveCommand);
        telemetry.addData("is finished run", liftMotorSubSystem.isFinishedRun);
        telemetry.addData("is active", liftMotorToPostionCommand.isActive);
        telemetry.update();
    }
}

