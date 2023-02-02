package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command.DriveCommand;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.DriveSubSystem;

@TeleOp(name = "CommandTeleOp")
public class CommandTeleOp extends CommandOpMode {//initalize hardware and subsystems and scedules commands

    private MotorEx FR, BR, FL, BL;
    private DriveSubSystem driveSubSystem;
    private DriveCommand driveCommand;

    private GamepadEx Gamepad1;

    @Override
    public void initialize() {
        FR = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435);
        BR = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435);
        FL = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435);
        BL = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435);//if gobilda motor add new prametter GoBILDA

        Gamepad1 = new GamepadEx(gamepad1);

        driveSubSystem = new DriveSubSystem(FR, BR, FL, BL);
        driveCommand = new DriveCommand(driveSubSystem, Gamepad1::getLeftX, Gamepad1::getLeftY, Gamepad1::getRightX);

        register(driveSubSystem);
        driveSubSystem.setDefaultCommand(driveCommand);
    }
}
