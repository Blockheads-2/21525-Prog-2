package org.firstinspires.ftc.teamcode.common.kinematics.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.swerve.teleop.TrackJoystick;

public class RevisedKinematics {
    protected Constants constants = new Constants();

    private double lx;
    private double ly;
    private double rx;
    private double ry;

    public enum DriveType{
        LINEAR,
        SNAP,
        STOP,
        TURN,
        SPLINE,
        NOT_INITIALIZED
    }
    public DriveType type = DriveType.NOT_INITIALIZED;

    //robot's power
    private double leftRotatePower = 0.0;
    private double rightRotatePower = 0.0;
    private double spinPower = 0.0;

    public double telLeftRotatePower = 0.0;
    public double telRightRotatePower = 0.0;
    public double telSpinPower = 0.0;

    double translatePerc = 0.6; //placeholder for now
    double rotatePerc = 0.4;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicksR = 0; //make protected later
    public int spinClicksL = 0; //make protected later
    public int rightThrottle = 1;
    public int leftThrottle = -1;
    public double target = 0;
    public double turnAmountL = 0;
    public double turnAmountR = 0;

    //current orientation
    GlobalPosSystem posSystem;
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation
    boolean initPole = true;

    public Accelerator accelerator;
    TrackJoystick joystickTracker;

    //PIDs
    protected SnapSwerveModulePID snapLeftWheelPID;
    protected SnapSwerveModulePID snapRightWheelPID;
    public double kp = 0.03;
    public double ki = 0;
    public double kd = 0.01;

    public boolean firstMovement = true;

    public RevisedKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();

        snapLeftWheelPID.setTargets(kp, ki, kd);
        snapRightWheelPID.setTargets(kp, ki, kd);

        accelerator = new Accelerator();
        joystickTracker = new TrackJoystick();

        rightThrottle = 1;
        leftThrottle = -1;
    }

    public void logic(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;

        //tracking the joystick's movement
        joystickTracker.trackJoystickL(lx, ly);

        if (noMovementRequests()) type = DriveType.STOP;
        else type = DriveType.LINEAR;

        //determining current position
        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];

        //determining targets, and how much we want to turn
        target = Math.toDegrees(Math.atan2(lx, ly));
        if (lx == 0 && ly == 0) target = 0;
        else if (lx==0 && ly < 0) target=180;
        turnAmountL = wheelOptimization(target, leftCurrentW);
        turnAmountR = wheelOptimization(target, rightCurrentW);

        //determining spin power
        spinPower = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));
        spinClicksL = (int)(spinPower * 100 * leftThrottle);
        spinClicksR = (int)(spinPower * 100 * rightThrottle);

        //determining rotational power
        leftRotatePower = snapLeftWheelPID.update(turnAmountL);
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotatePower = snapRightWheelPID.update(turnAmountR);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

        //determining whether to focus more on spinning or more on rotating
        rotatePerc = (Math.abs(turnAmountL) + Math.abs(turnAmountR)) / 70; //turnAmount / 35 --> percentage
        if (rotatePerc > 0.5) rotatePerc = 0.5;
        translatePerc = 1 - rotatePerc;

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();

        //determining values from right stick input.
        rightStick();

        //unnecessary function but useful for telemetry
        stop();
    }

    public void firstMovement(){
        if (joystickTracker.getChange() > 90 || noMovementRequests()) firstMovement = true;
        if (firstMovement){
            if (Math.abs(turnAmountL) >= constants.degreeTOLERANCE || Math.abs(turnAmountR) >= constants.degreeTOLERANCE){
                translatePerc = 0;
                rotatePerc = 1;
                spinPower = 0;
                spinClicksL = 0;
                spinClicksR = 0;
            } else{
                firstMovement = false;
                translatePerc = 0.6;
                rotatePerc = 0.4;
            }
        }
    }

    public void rightStick(){
        if (Math.abs(leftCurrentW) < constants.degreeTOLERANCE && Math.abs(rightCurrentW) < constants.degreeTOLERANCE && lx == 0 && ly == 0 && (rx != 0 || ry != 0)){
//            leftThrottle = leftThrottle;
            rightThrottle *= -1;
            spinClicksL = (int) (rx * 100 * leftThrottle);
            spinClicksR = (int) (rx * 100 * rightThrottle);
            spinPower = rx;

            rotatePerc = (Math.abs(turnAmountL) + Math.abs(turnAmountR)) / 60.0; //turnAmount / 30 --> percentage
            if (rotatePerc > 0.5) rotatePerc = 0.5;
            translatePerc = 1 - rotatePerc;

            turnAmountL = -leftCurrentW;
            leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
            leftRotatePower = snapLeftWheelPID.update(turnAmountL);

            turnAmountR = -rightCurrentW;
            rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);
            rightRotatePower = snapRightWheelPID.update(turnAmountR);


            type = DriveType.TURN;
        } else{
            //spline
        }
    }

    public void stop(){
        if (type == DriveType.STOP){
            target = 0;
            turnAmountL = 0;
            turnAmountR = 0;

            spinPower = 0;
            spinClicksR = 0;
            spinClicksL = 0;

            rightRotatePower = 0;
            leftRotatePower = 0;
            rightRotClicks = 0;
            leftRotClicks = 0;

            translatePerc = 0;
            rotatePerc = 0;
        }
    }

    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount1 = target - currentW;
        double turnAmount2 = target2 - current2;
        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);

        rightThrottle = 1;
        leftThrottle = -1;


        return turnAmount;
    }

//    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
//        double target2 = (target < 0 ? target + 360 : target);
//        double current2 = (currentW < 0 ? currentW + 360 : currentW);
//
//        double turnAmount1 = target - clamp(currentW + (initPole ? 0 : 180));
//        double turnAmount2 = target2 - clampConventional(current2 + (initPole ? 0 : 180));
//
//        double turnAmount = (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);
//
//        if(Math.abs(turnAmount) > 90){
//            initPole = !initPole;
//            turnAmount %= 180;
//            turnAmount *= -1;
//
//            this.rightThrottle *= -1;
//            this.leftThrottle *= -1;
//        }
//        return turnAmount;
//    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;
        if (degrees == -180) degrees = 180;

        if (degrees < -180){
            degrees = 180 - (Math.abs(degrees) - 180);
        } else if (degrees > 180){
            degrees = -180 + (Math.abs(degrees) - 180);
        }
        return degrees;
    }

    public double clampConventional(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < 0){
            degrees = 360 - Math.abs(degrees);
        } else if (degrees >= 360){
            degrees = Math.abs(degrees) - 360;
        }
        return degrees;
    }

    public double[] getPower(){
        double[] motorPower = new double[4];
        motorPower[0] = (spinPower * translatePerc + leftRotatePower * rotatePerc); //top left
        motorPower[1] = (spinPower * translatePerc + leftRotatePower * rotatePerc); //bottom left
        motorPower[2] = (spinPower * translatePerc + rightRotatePower * rotatePerc); //top right
        motorPower[3] = (spinPower * translatePerc + rightRotatePower * rotatePerc); //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i]);
            motorPower[i] *= constants.POWER_LIMITER;
//            if (motorPower[i] > constants.POWER_LIMITER) motorPower[i] = constants.POWER_LIMITER;
//            else if (motorPower[i] < -constants.POWER_LIMITER) motorPower[i] = -constants.POWER_LIMITER;
        }

        telLeftRotatePower = leftRotatePower * rotatePerc * constants.POWER_LIMITER;
        telRightRotatePower = rightRotatePower * rotatePerc * constants.POWER_LIMITER;
        telSpinPower = spinPower * translatePerc * constants.POWER_LIMITER;

        return motorPower;
    }


    public int[] getClicks(){
        int[] clicks = new int[4];
        clicks[0] = spinClicksL + leftRotClicks; //left
        clicks[1] = -spinClicksL + leftRotClicks; //left
        clicks[2] = spinClicksR + rightRotClicks; //right
        clicks[3] = -spinClicksR  + rightRotClicks; //right
        return clicks;
    }

    public DriveType getDriveType(){
        return type;
    }

    public boolean noMovementRequests(){
        return (lx==0 && ly==0 && rx==0 && ry==0);
    }

    public double getTarget(){
        return target;
    }
    public double getLTurnAmount(){
        return turnAmountL;
    }
    public double getRTurnAmount(){
        return turnAmountR;
    }


    public void switchLeftSpinDirection(){
        leftThrottle *= -1;
    }

    public void switchRightSpinDirection(){
        rightThrottle *= -1;
    }
}