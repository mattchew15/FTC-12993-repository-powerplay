package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClawSubSystem extends SubsystemBase {
    private Servo IntakeClawServo;

    public IntakeClawSubSystem(HardwareMap hardwareMap){
        IntakeClawServo = hardwareMap.get(Servo.class, "IntakeClawServo");
    }
    public void periodic(){

    }

    public void MoveToPosition(double targetPostion){
        IntakeClawServo.setPosition(targetPostion);
    }
}
