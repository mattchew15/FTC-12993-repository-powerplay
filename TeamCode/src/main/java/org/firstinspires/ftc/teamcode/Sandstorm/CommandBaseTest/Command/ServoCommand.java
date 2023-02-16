package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.ServoSubSystem;

public class ServoCommand extends CommandBase {
    private ServoSubSystem servoSubSystem;
    private GamepadEx gamepadex1;
    public boolean isActive = false;

    public ServoCommand(ServoSubSystem servoSubSystem, GamepadEx gamepadex1){
        this.servoSubSystem = servoSubSystem;
        this.gamepadex1 = gamepadex1;

        addRequirements(servoSubSystem);
    }
    @Override
    public void initialize(){
        isActive = true;
    }
    @Override
    public void execute(){
        if (gamepadex1.getButton(GamepadKeys.Button.X)){// : )
            servoSubSystem.open();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean interrupted){
        servoSubSystem.close();
        isActive = false;
    }
}
