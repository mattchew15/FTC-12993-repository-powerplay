package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeArmSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeClawSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeSlideMotorSubSystem;
import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.IntakeSlideSubSystem;

import java.util.function.DoubleSupplier;

public class IntakeSlidesClawCommand extends CommandBase {
    private IntakeSlideSubSystem intakeSlideSubSystem;
    private IntakeClawSubSystem intakeClawSubSystem;
    private IntakeArmSubSystem intakeArmSubSystem;
    private IntakeSlideMotorSubSystem intakeSlideMotorSubSystem;
    private DoubleSupplier elevator;
    private GamepadEx gamepadex1, gamepadex2;
    public double IntakeElevatorReadyPos = 0.575, IntakeArmReadyPos = 0.81, IntakeClawOpenPos = 0.52;
    public int IntakeSlideReadyPos = 0;
    public double mode = 1;

    public IntakeSlidesClawCommand(IntakeSlideSubSystem intakeSlideSubSystem, IntakeSlideMotorSubSystem intakeSlideMotorSubSystem, IntakeArmSubSystem intakeArmSubSystem, IntakeClawSubSystem intakeClawSubSystem, GamepadEx gamepadex1, GamepadEx gamepadex2, DoubleSupplier elevator){
        this.intakeSlideSubSystem = intakeSlideSubSystem;
        this.intakeSlideMotorSubSystem = intakeSlideMotorSubSystem;
        this.intakeArmSubSystem = intakeArmSubSystem;
        this.intakeClawSubSystem = intakeClawSubSystem;
        this.elevator = elevator;
        this.gamepadex1 = gamepadex1;
        this.gamepadex2 = gamepadex2;
    }
    
    @Override
    public void initialize(){
        intakeArmSubSystem.MoveToPosition(IntakeArmReadyPos);
        intakeSlideSubSystem.MoveToPosition(IntakeElevatorReadyPos);
        intakeClawSubSystem.MoveToPosition(IntakeClawOpenPos);
        intakeSlideMotorSubSystem.moveInTick(IntakeSlideReadyPos);

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
}
