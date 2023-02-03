package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class MotorSubSystem extends SubsystemBase {
    private MotorEx IntakeMotor;

    public MotorSubSystem(MotorEx intakeMotor){//: )
        IntakeMotor = intakeMotor;
    }
    public void spin() {
        IntakeMotor.setRunMode(MotorEx.RunMode.RawPower);
        IntakeMotor.set(1);
    }
    public void stop(){
        IntakeMotor.setRunMode(MotorEx.RunMode.RawPower);
        IntakeMotor.set(0);
    }

}
