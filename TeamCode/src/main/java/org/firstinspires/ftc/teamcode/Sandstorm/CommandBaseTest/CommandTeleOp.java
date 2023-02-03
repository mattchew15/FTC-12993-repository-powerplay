package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.DriveCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.MotorCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.ServoCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.DriveSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.MotorSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.ServoSubSystem;

@TeleOp(name = "CommandTeleOp")
public class CommandTeleOp extends CommandOpMode {//initalize hardware and subsystems and scedules commands :)

    private MotorEx FR, BR, FL, BL, IntakeMotor;
    private DriveSubSystem driveSubSystem;
    private DriveCommand driveCommand;
    private MotorSubSystem motorSubSystem;
    private MotorCommand motorCommand;
    private ServoSubSystem servoSubSystem;
    private ServoCommand servoCommand;

    private GamepadEx Gamepadex1;

    @Override
    public void initialize() {
        FR = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435);
        BR = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435);
        FL = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435);
        BL = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435);//if gobilda motor add new prametter GoBILDA
        IntakeMotor = new MotorEx(hardwareMap, "IntakeMotor", Motor.GoBILDA.RPM_312);
        FL.setInverted(true);
        //FR.setInverted(true);
        //BR.setInverted(true);
        //BL.setInverted(true);

        Gamepadex1 = new GamepadEx(gamepad1);

        driveSubSystem = new DriveSubSystem(FR, BR, FL, BL);
        driveCommand = new DriveCommand(driveSubSystem, Gamepadex1::getLeftX, Gamepadex1::getLeftY, Gamepadex1::getRightX);
        motorSubSystem = new MotorSubSystem(IntakeMotor);
        motorCommand = new MotorCommand(motorSubSystem, Gamepadex1);
        servoSubSystem = new ServoSubSystem(hardwareMap);
        servoCommand = new ServoCommand(servoSubSystem, Gamepadex1);

        Gamepadex1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(motorCommand);
        Gamepadex1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(servoCommand);

        register(servoSubSystem);
        driveSubSystem.setDefaultCommand(driveCommand);
    }
}
