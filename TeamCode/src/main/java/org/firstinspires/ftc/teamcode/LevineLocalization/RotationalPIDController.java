package org.firstinspires.ftc.teamcode.LevineLocalization;

public class RotationalPIDController {
    private double kP;
    private double kI;
    private double kD;
    private double integral;
    private double previousError;

    public RotationalPIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.integral = 0;
        this.previousError = 0;
    }

    public double comuteRotationalPID(double error){
        integral += error;
        double derivative = error - previousError;

        double output =  (kP * error) + (kI * integral) + (kD * derivative);
        previousError = error;
        return output;
    }
}
