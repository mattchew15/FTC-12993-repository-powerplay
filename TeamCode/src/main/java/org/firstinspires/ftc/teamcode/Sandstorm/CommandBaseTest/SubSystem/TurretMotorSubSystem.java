package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class TurretMotorSubSystem extends SubsystemBase {
    private MotorEx TurretMotor;

    public TurretMotorSubSystem(MotorEx turretMotor){
        TurretMotor = turretMotor;
    }
    public void spin() {
        TurretMotor.setRunMode(MotorEx.RunMode.RawPower);
        TurretMotor.set(1);
    }
    public void stop(){
        TurretMotor.setRunMode(MotorEx.RunMode.RawPower);
        TurretMotor.set(0);
    }
}
