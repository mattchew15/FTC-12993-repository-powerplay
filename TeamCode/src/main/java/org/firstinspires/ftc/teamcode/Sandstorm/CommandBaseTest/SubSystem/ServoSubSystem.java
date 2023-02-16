package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubSystem extends SubsystemBase {//: )
    private Servo servo;

    private double openPos = 0.52;
    private double closePos = 0.0;

    public ServoSubSystem(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "servoTest");
    }

    public void periodic(){

    }

    public void open(){
        servo.setPosition(openPos);
    }

    public void close(){
        servo.setPosition(closePos);
    }
}
