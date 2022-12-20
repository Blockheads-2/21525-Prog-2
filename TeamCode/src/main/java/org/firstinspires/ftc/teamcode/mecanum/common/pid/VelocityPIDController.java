package org.firstinspires.ftc.teamcode.mecanum.common.pid;

public class VelocityPIDController {
    double p;
    double i;
    double d;
    double targetVelocity;
    public VelocityPIDController(double velocity, double Kp,double Ki,double Kd) {
        targetVelocity = velocity;
        p = Kp;
        i = Ki;
        d = Kd;

    }
    public double controller(double currentVelocity) {
        // P
        double error = targetVelocity - currentVelocity;

        // I
        i = 0;

        // D
        double d = 0;

        return Math.tanh(p * error);
    }
}