package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem.DriveSubSystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase { //command is a rigid state machine that controlls a subsystem and makes bindings :)
    private DriveSubSystem driveSubSystem;
    private DoubleSupplier strafe, forward, turn;

    public DriveCommand(DriveSubSystem driveSubSystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubSystem = driveSubSystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;

        addRequirements(driveSubSystem);
    }
    @Override
    public void execute(){
        driveSubSystem.drive(forward.getAsDouble(), turn.getAsDouble(), strafe.getAsDouble());
    }
}
