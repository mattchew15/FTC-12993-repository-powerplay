package org.firstinspires.ftc.teamcode.Sandstorm.CommandBaseTest.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class LiftMotorSubSystem extends SubsystemBase {
    private MotorEx LiftMotor;
    public double tolerance = 50;
    public boolean isFinishedRun = false, isActiveCommand = true;

    public LiftMotorSubSystem(MotorEx liftMotor){
        LiftMotor = liftMotor;
    }
    public void spin() {
        LiftMotor.setRunMode(MotorEx.RunMode.RawPower);
        LiftMotor.set(1);
    }
    public void stop(){
        LiftMotor.setRunMode(MotorEx.RunMode.RawPower);
        LiftMotor.set(0);
    }
    public void spinToPostion(int amountToMove){
        isFinishedRun = false;
        LiftMotor.setRunMode(Motor.RunMode.PositionControl);
        LiftMotor.setPositionCoefficient(0.005);
        double kP = LiftMotor.getPositionCoefficient();
        LiftMotor.setTargetPosition(amountToMove);
        LiftMotor.setPositionTolerance(tolerance);
        while (!LiftMotor.atTargetPosition()){
            LiftMotor.set(0.5);
        }
        LiftMotor.stopMotor();
        isFinishedRun = true;
    }
}
