package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.MotorSubSystem;

public class MotorCommand extends CommandBase {
    private MotorSubSystem motorSubSystem;
    private GamepadEx Gamepadex1;
    //private boolean isRuning = false; )
    public boolean isAcitve = false;

    public MotorCommand(MotorSubSystem motorSubSystem, GamepadEx Gamepadex1){
        this.motorSubSystem = motorSubSystem;
        this.Gamepadex1 = Gamepadex1;

        addRequirements(motorSubSystem);
    }
    @Override
    public void initialize(){
        isAcitve = true;
    }
    @Override
    public void execute() {
        if (Gamepadex1.getButton(GamepadKeys.Button.B)){
            motorSubSystem.spin();
        }
    }
    @Override
    public boolean isFinished(){
    return false;
    }

    @Override
    public void end(boolean interrupted){
        motorSubSystem.stop();
        isAcitve = false;
    }
}
