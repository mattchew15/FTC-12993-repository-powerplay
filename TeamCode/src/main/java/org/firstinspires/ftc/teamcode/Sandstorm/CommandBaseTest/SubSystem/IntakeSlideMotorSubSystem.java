package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class IntakeSlideMotorSubSystem extends SubsystemBase {
    private MotorEx IntakeSlideMotor;
    public int amountToMove = -400;

    public IntakeSlideMotorSubSystem(MotorEx intakeSlideMotor){
        IntakeSlideMotor = intakeSlideMotor;
    }
    public void spin() {
        IntakeSlideMotor.setRunMode(MotorEx.RunMode.RawPower);
        IntakeSlideMotor.set(1);
    }
    public void stop(){
        IntakeSlideMotor.setRunMode(MotorEx.RunMode.RawPower);
        IntakeSlideMotor.set(0);
    }
    public void moveInTick(int targetPostion){
        IntakeSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        amountToMove = targetPostion - IntakeSlideMotor.getCurrentPosition();
        IntakeSlideMotor.setTargetPosition(targetPostion);
        IntakeSlideMotor.setPositionTolerance(20);
        while (!IntakeSlideMotor.atTargetPosition()){
            IntakeSlideMotor.set(0.5);
        }
        IntakeSlideMotor.stopMotor();
    }
}

