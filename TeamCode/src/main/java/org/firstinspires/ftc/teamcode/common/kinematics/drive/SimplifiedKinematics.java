package org.firstinspires.ftc.teamcode.common.kinematics.drive;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;

public class SimplifiedKinematics {
    protected Constants constants = new Constants();

    private double lx;
    private double ly;
    private double rx;
    private double ry;

    public enum DriveType{
        LINEAR,
        SNAP,
        TURN,
        STOP,
        NOT_INITIALIZED
    }
    public DriveType type = DriveType.NOT_INITIALIZED;

    //robot's power
    double leftRotatePower = 0.0;
    double rightRotatePower = 0.0;
    double spinPower = 0.0;
    double translatePerc = 0.0;
    double rotatePerc = 0.0;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicks = 0; //make protected later

    public int leftTurnDirectionW = 1;
    public int rightTurnDirectionW = 1;
    public int spinDirectionR = -1;
    public int spinDirectionL = -1;

    double target;
    double turnAmountL;
    double turnAmountR;
    private enum Module{
        RIGHT,
        LEFT
    }

    //current orientation
    GlobalPosSystem posSystem;
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation

    public Accelerator accelerator;

    //PIDs
    protected SnapSwerveModulePID snapLeftWheelPID;
    protected SnapSwerveModulePID snapRightWheelPID;

    double[] motorPower = new double[4];

//    protected RotateSwerveModulePID tableSpinWheelPID;
//    protected LinearCorrectionPID linearCorrectionPID;
//    protected RotateSwerveModulePID counteractSplinePID;

    public SimplifiedKinematics(GlobalPosSystem posSystem){
        this.posSystem = posSystem;
        
        snapLeftWheelPID = new SnapSwerveModulePID();
        snapRightWheelPID = new SnapSwerveModulePID();  

        snapLeftWheelPID=new SnapSwerveModulePID();
        snapRightWheelPID=new SnapSwerveModulePID();
        snapLeftWheelPID.setTargets(0.03, 0, 0.01);
        snapRightWheelPID.setTargets(0.03, 0, 0.01);

        accelerator = new Accelerator();
    }

    public void logic(){
        leftCurrentW = posSystem.getLeftWheelW();
        rightCurrentW = posSystem.getRightWheelW();
        currentR = posSystem.getPositionArr()[4];

        target = Math.toDegrees(Math.atan2(lx, ly));
        if (lx == 0 && ly == 0) target = 0;
        else if (lx==0 && ly < 0) target=180;
        target=clamp(target);
        wheelOptimization(target, leftCurrentW, Module.LEFT, false);
        wheelOptimization(target, rightCurrentW, Module.RIGHT, false);

        if (noMovementRequests()){
            type=DriveType.STOP;
            spinPower = 0;
            leftRotatePower = 0;
            rightRotatePower = 0;

            spinClicks = 0;
            rightRotClicks = 0;
            leftRotClicks = 0;

            translatePerc = 0;
            rotatePerc = 0;

        } else if(shouldSnap()){
            type=DriveType.SNAP;
            wheelOptimization(target, leftCurrentW, Module.LEFT, true);
            wheelOptimization(target, rightCurrentW, Module.RIGHT, true);

            spinPower = 0;
            leftRotatePower = snapLeftWheelPID.update(turnAmountL * leftTurnDirectionW); //turnAmountL approaches 0, which is why we set our PID target to 0.
            rightRotatePower = snapRightWheelPID.update(turnAmountR * rightTurnDirectionW);

            spinClicks = 0;
            rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE) * rightTurnDirectionW;
            leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE) * leftTurnDirectionW;
            translatePerc=0;
            rotatePerc=1;
        } else{
            type=DriveType.LINEAR;

            if (spinDirectionL != spinDirectionR){
                //then something is wrong
                spinDirectionL = spinDirectionR;
            }
            spinPower = Math.sqrt(Math.pow(lx,2) + Math.pow(ly, 2));
            spinClicks = (int)(spinPower * 100) * spinDirectionR;
            leftRotClicks = 0;
            rightRotClicks = 0;

            translatePerc = 1;
            rotatePerc = 0;
        }
    }

    public void wheelOptimization(double target, double currentW, Module module, boolean after){ //returns how much the wheels should rotate in which direction
//        if (spinDirectionR == -1 && spinDirectionL == -1){
//            currentW += 180;
//            currentW = clamp(currentW);
//        }

        double turnAmount = target - currentW;
        int turnDirection = (int)Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180){
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }

        if (target == 180){
            turnAmount = 0;
            spinDirectionL = 1;
            spinDirectionR = 1;
        } else{
            spinDirectionL=-1;
            spinDirectionR=-1;
        }

//        if(Math.abs(turnAmount) > 90){
//            target += 180;
//            target = clamp(target);
//            turnAmount = target - currentW;
//            turnDirection *= -1;
//            if(Math.abs(turnAmount) > 180){
//                turnAmount = 360 - Math.abs(turnAmount);
//            }
//            this.target=target-180;
//            this.target=clamp(this.target);
//            if(after){
//                if (module == Module.RIGHT){
//                    spinDirectionR = -1;
//                } else if (module==Module.LEFT){
//                    spinDirectionL = -1;
//                }
//            }
//        }

        switch (module){
            case RIGHT:
                rightTurnDirectionW = turnDirection;
                turnAmountR = Math.abs(turnAmount);
                break;

            case LEFT:
                leftTurnDirectionW = turnDirection;
                turnAmountL =  Math.abs(turnAmount);
                break;
        }
    }

    public boolean shouldSnap(){
        return (Math.abs(turnAmountL) >= constants.degreeTOLERANCE || Math.abs(turnAmountR) >= constants.degreeTOLERANCE);
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }

    public void getGamepad(double lx, double ly, double rx, double ry){
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
    }

    public double[] getPower(){

        motorPower[0] = spinPower * translatePerc + leftRotatePower * rotatePerc; //top left
        motorPower[1] = spinPower * translatePerc + leftRotatePower * rotatePerc; //bottom left
        motorPower[2] = spinPower * translatePerc + rightRotatePower * rotatePerc; //top right
        motorPower[3] = spinPower * translatePerc + rightRotatePower * rotatePerc; //bottom right

        motorPower[0] = accelerator.update(motorPower[0], type);
        motorPower[1] = accelerator.update(motorPower[1], type);
        motorPower[2] = accelerator.update(motorPower[2], type);
        motorPower[3] = accelerator.update(motorPower[3], type);

        return motorPower;
    }


    public int[] getClicks(){
        int[] clicks = new int[4];
        clicks[0] = spinClicks + leftRotClicks; //left
        clicks[1] = -spinClicks + leftRotClicks; //left
        clicks[2] = spinClicks + rightRotClicks; //right
        clicks[3] = -spinClicks  + rightRotClicks; //right
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
    public int getRightDirectionW(){
        return rightTurnDirectionW;
    }
    public int getLeftDirectionW(){
        return leftTurnDirectionW;
    }
    public double getLTurnAmount(){
        return turnAmountL;
    }
    public double getRTurnAmount(){
        return turnAmountR;
    }

    public void switchRotateDirection(){
        rightTurnDirectionW *= -1;
        leftTurnDirectionW *= -1;
    }

    public void switchSpinDirection(){
        spinDirectionR *= -1;
        spinDirectionL *= -1;
    }
}

