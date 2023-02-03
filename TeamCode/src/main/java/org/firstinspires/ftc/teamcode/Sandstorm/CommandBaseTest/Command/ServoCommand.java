package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.ServoSubSystem;

public class ServoCommand extends CommandBase {
    private ServoSubSystem servoSubSystem;
    private GamepadEx Gamepadex1;

    public ServoCommand(ServoSubSystem servoSubSystem, GamepadEx Gamepadex1){
        this.servoSubSystem = servoSubSystem;
        this.Gamepadex1 = Gamepadex1;

        addRequirements(servoSubSystem);
    }
    @Override
    public void execute(){
        if (Gamepadex1.getButton(GamepadKeys.Button.X)){// : )
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
    }
}
