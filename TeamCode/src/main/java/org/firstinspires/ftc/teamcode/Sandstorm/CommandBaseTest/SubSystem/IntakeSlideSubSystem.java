package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSlideSubSystem extends SubsystemBase {
    private Servo IntakeSlideServo;

    public IntakeSlideSubSystem(HardwareMap hardwareMap){
        IntakeSlideServo = hardwareMap.get(Servo.class, "IntakeSlideServo");
    }
    public void periodic(){

    }

    public void MoveToPosition(double targetPostion){
        IntakeSlideServo.setPosition(targetPostion);
    }
}
