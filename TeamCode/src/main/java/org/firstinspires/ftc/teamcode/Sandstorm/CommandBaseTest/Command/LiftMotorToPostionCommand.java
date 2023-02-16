package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.CommandTeleOp;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.LiftMotorSubSystem;

public class LiftMotorToPostionCommand extends CommandBase {
    private LiftMotorSubSystem liftMotorSubSystem;
    private GamepadEx Gamepadex1;
    public int TargetPostion = 0;
    public double LiftMotorPostion;
    public boolean isActive = false;

    public LiftMotorToPostionCommand(LiftMotorSubSystem liftMotorSubSystem, GamepadEx Gamepadex1){
        this.liftMotorSubSystem = liftMotorSubSystem;
        this.Gamepadex1 = Gamepadex1;

        addRequirements(liftMotorSubSystem);
    }
    @Override
    public void initialize(){
        liftMotorSubSystem.stop();
        isActive = true;
        liftMotorSubSystem.isActiveCommand = true;
        TargetPostion = 0;
    }

    @Override
    public void execute(){
        if (Gamepadex1.getButton(GamepadKeys.Button.X)){
            TargetPostion = 300;
            liftMotorSubSystem.isActiveCommand = false;
        }
        else if (Gamepadex1.getButton(GamepadKeys.Button.Y)) {
            //TargetPostion = 600;
            liftMotorSubSystem.isActiveCommand = false;
        }
        else if (Gamepadex1.getButton(GamepadKeys.Button.B)){
            TargetPostion = 900;
            liftMotorSubSystem.isActiveCommand = false;
        }
        if (!liftMotorSubSystem.isActiveCommand){
            liftMotorSubSystem.spinToPostion(TargetPostion);
        }
    }

    @Override
    public boolean isFinished(){
        return liftMotorSubSystem.isFinishedRun;
    }

    @Override
    public void end(boolean interrupted){
        liftMotorSubSystem.stop();
        TargetPostion = 0;
        isActive = false;
        liftMotorSubSystem.isActiveCommand = true;
        liftMotorSubSystem.isFinishedRun = false;
    }
}
