package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeArmSubSystem extends CommandBase {
    private Servo IntakeArmServo;

    public IntakeArmSubSystem(HardwareMap hardwareMap){
        IntakeArmServo = hardwareMap.get(Servo.class, "IntakeArmServo");
    }
    public void periodic(){

    }

    public void MoveToPosition(double targetPostion){
        IntakeArmServo.setPosition(targetPostion);
    }
}
