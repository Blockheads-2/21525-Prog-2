package org.firstinspires.ftc.teamcode.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ArmPID {
    private double kp, ki, kd;
    private double pError;

    private ElapsedTime timer = new ElapsedTime();
    private double targetClicks = Integer.MAX_VALUE;
    private double initClicks = 0;

    private double prevError = 0;
    private double prevTime = 0;
    private double accumulatedError = 0;

    private final static Logger LOGGER =
            Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    public ArmPID(){
        timer.reset();
    }

    public double update(double currClicks){
        double deltaClicks = currClicks - initClicks;
        //proportion
        double error = targetClicks - deltaClicks;

        //integral
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error); //ensures that accumulatedError and the error have the same sign
        accumulatedError += error;
        if (Math.abs(error) < 1) accumulatedError = 0; //if the error is pretty much non-existent, then we don't need to fine-tune much (the accumulated error would clash with already-good results).

        //derivative
        double slope = 0;
        if (prevTime > 0) slope = (error-prevError) / (timer.milliseconds() - prevTime); //if-statement makes sure that update() has been called at least once.
        prevError = error;
        prevTime = timer.milliseconds();

        double motorPower = Math.tanh(kp * error + ki * accumulatedError + kd * slope) * 0.9 + (0.1 * Math.signum(error));
        //double motorPower =  Math.tanh(kp * error + ki * accumulatedError + kd * slope);

        return motorPower;
    }

    public void setTargets(double targetClicks, double initClicks, double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.targetClicks = targetClicks;
        if (this.targetClicks != targetClicks) this.initClicks = initClicks;
    }
}
