package org.firstinspires.ftc.teamcode.common.kinematics.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.AutoPID;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;
import org.firstinspires.ftc.teamcode.swerve.teleop.TrackJoystick;

public class AutoKinematics {
    protected Constants constants = new Constants();

    double x;
    double y;
    double finalAngle;
    double power;

    public enum DriveType{
        LINEAR,
        SNAP,
        STOP,
        TURN,
        NOT_INITIALIZED
    }
    public DriveType type = DriveType.NOT_INITIALIZED;

    //robot's power
    public double leftRotatePower = 0.0;
    public double rightRotatePower = 0.0;
    public double spinPower = 0.0;

    double translatePerc = 0.6; //placeholder for now
    double rotatePerc = 0.4;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicksR = 0; //make protected later
    public int spinClicksL = 0; //make protected later
    public int rightThrottle = 1;
    public int leftThrottle = 1;

    public double target = 0;
    public double turnAmountL = 0;
    public double turnAmountR = 0;

    //current orientation
    GlobalPosSystem posSystem;
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation

    public Accelerator accelerator;
//    TrackJoystick joystickTracker;

    //PIDs
    SnapSwerveModulePID snapLeftWheelPID;
    SnapSwerveModulePID snapRightWheelPID;
    AutoPID spinPID;


    public boolean firstMovement = true;

    public AutoKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;

        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();
        spinPID = new AutoPID();

        snapLeftWheelPID.setTargets(0.03, 0, 0.01);
        snapRightWheelPID.setTargets(0.03, 0, 0.01);

        accelerator = new Accelerator();
//        joystickTracker = new TrackJoystick();

        rightThrottle = 1;
        leftThrottle = -1;
    }

    public void setPos(double x, double y, double theta, double speed){
        this.x = x;
        this.y = y;
        this.finalAngle = theta;
        this.power = speed;

        spinPID.setTargets(x, y, 0.03, 0, 0.01);
    }

    public void logic(){
        //tracking the joystick's movement
//        joystickTracker.trackJoystickL(x, y);

        if (noMovementRequests()) type = DriveType.STOP;
        else type = DriveType.LINEAR;

        //determining current position
        leftCurrentW = posSystem.getPositionArr()[2];
        rightCurrentW = posSystem.getPositionArr()[3];
        currentR = posSystem.getPositionArr()[4];

        //determining targets, and how much we want to turn
        target = Math.toDegrees(Math.atan2(x, y));
        if (x == 0 && y == 0) target = 0;
        else if (x==0 && y < 0) target=180;
        turnAmountL = wheelOptimization(target, leftCurrentW);
        turnAmountR = wheelOptimization(target, rightCurrentW);

        //determining spin power
        spinPower = spinPID.update(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
        spinClicksL = (int)(spinPower * 100 * leftThrottle);
        spinClicksR = (int)(spinPower * 100 * rightThrottle);

        //determining rotational power
        leftRotatePower = snapLeftWheelPID.update(turnAmountL);
        leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
        rightRotatePower = snapRightWheelPID.update(turnAmountR);
        rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);

        //determining whether to focus more on spinning or more on rotating
        rotatePerc = (Math.abs(turnAmountL) + Math.abs(turnAmountR)) / 180.0; //turnAmount / 90 --> percentage
        if (rotatePerc > 0.5) rotatePerc = 0.5;
        translatePerc = 1 - rotatePerc;

        //determining "firstMovement" actions, if it is the robot's "firstMovement."
        firstMovement();

        //determining values from right stick input.
//        rightStick();

        //unnecessary function but useful for telemetry
        stop();
    }

    public void firstMovement(){
//        if (joystickTracker.getChange() > 90 || noMovementRequests()) firstMovement = true;
        if (noMovementRequests()) firstMovement = true;
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

//    public void rightStick(){
//        if (Math.abs(leftCurrentW - currentR) < constants.degreeTOLERANCE && Math.abs(rightCurrentW - currentR) < constants.degreeTOLERANCE && lx == 0 && ly == 0 && (rx != 0 || ry != 0)){
//            leftThrottle = 1;
//            rightThrottle = 1;
//            spinClicksL = (int) (rx * 100 * leftThrottle);
//            spinClicksR = (int) (rx * 100 * rightThrottle);
//            spinPower = rx;
//
//            rotatePerc = (Math.abs(turnAmountL) + Math.abs(turnAmountR)) / 60.0; //turnAmount / 30 --> percentage
//            if (rotatePerc > 0.5) rotatePerc = 0.5;
//            translatePerc = 1 - rotatePerc;
//
//            turnAmountL = -leftCurrentW;
//            leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE);
//            leftRotatePower = snapLeftWheelPID.update(turnAmountL);
//
//            turnAmountR = -rightCurrentW;
//            rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE);
//            rightRotatePower = snapRightWheelPID.update(turnAmountR);
//
//
//            type = DriveType.TURN;
//        } else{
//            //spline
//        }
//    }

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

//        if(Math.abs(turnAmount) > 90){
//            turnAmount %= 180;
//            turnAmount *= -1;
//
//            if(Math.abs(turnAmount) > 180){
//                turnAmount = 360 - Math.abs(turnAmount);
//            }
////            this.translateSwitchMotors *= -1; //needs to be fixed, or else the wheels will oscilate a lot.
//        }

        return turnAmount;
    }

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

    public double[] getPower(){
        double[] motorPower = new double[4];
        motorPower[0] = spinPower * translatePerc + leftRotatePower * rotatePerc; //top left
        motorPower[1] = spinPower * translatePerc + leftRotatePower * rotatePerc; //bottom left
        motorPower[2] = spinPower * translatePerc + rightRotatePower * rotatePerc; //top right
        motorPower[3] = spinPower * translatePerc + rightRotatePower * rotatePerc; //bottom right

        for (int i = 0; i < 4; i++){
            motorPower[i] = accelerator.update(motorPower[i]);
        }
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
        return (x==0 && y==0 && finalAngle==0);
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


    public void switchSpinDirection(){
        leftThrottle *= -1;
        rightThrottle *= -1;
    }
}

