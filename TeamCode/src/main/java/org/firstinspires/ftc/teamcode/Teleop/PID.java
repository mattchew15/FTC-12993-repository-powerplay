package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    ElapsedTime timer = new ElapsedTime();
    double Kp;
    double Ki;
    double Kd;
    double integralSumLimit = 0.3; // max integral value, should have its own constructor variable to change
    double lastError;
    double integralSum;
    double error;


    public PID(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
    }

    public double update(double target, double state, double maxOutput) { // parameter of the method is the target,
        // PID logic and then return the output
        error = target-state;
        double derivative = (error-lastError)/timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        if (output > maxOutput){ // basically cuts the PID so the motor can run at the max speed
            output = maxOutput;
        }
        lastError = error;

        return output;
    }

    public double returnError(){
        return error;
    }

}
