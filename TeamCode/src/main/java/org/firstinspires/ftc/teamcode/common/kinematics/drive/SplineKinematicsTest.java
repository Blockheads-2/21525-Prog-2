//package org.firstinspires.ftc.teamcode.common.kinematics.drive;
//
//import org.firstinspires.ftc.teamcode.common.Accelerator;
//import org.firstinspires.ftc.teamcode.common.Reset;
//import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
//import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
//import org.firstinspires.ftc.teamcode.common.pid.LinearCorrectionPID;
//import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;
//import org.firstinspires.ftc.teamcode.common.pid.SnapSwerveModulePID;
//import org.firstinspires.ftc.teamcode.teleop.TrackJoystick;
//
//public class SplineKinematicsTest {
//    protected Constants constants = new Constants();
//
//    private double lx;
//    private double ly;
//    private double rx;
//    private double ry;
//
//    public enum DriveType{
//        LINEAR,
//        SPLINE,
//        SNAP,
//        STOP,
//        NOT_INITIALIZED
//    }
//    public DriveType type = DriveType.NOT_INITIALIZED;
//
//    //robot's power
//    double leftRotatePower = 0.0;
//    double rightRotatePower = 0.0;
//    double spinPower = 0.0;
//    double translatePerc = 0.0;
//    double rotatePerc = 0.0;
//    double leftThrottle = 1;
//    double rightThrottle = 1;
//
//    //target clicks
//    public int rightRotClicks = 0;
//    public int leftRotClicks = 0;
//    public int spinClicks = 0; //make protected later
//
//    public int leftTurnDirectionW = 1;
//    public int rightTurnDirectionW = 1;
//    public int spinDirectionR = 1;
//    public int spinDirectionL = 1;
//
//    double target;
//    double turnAmountL;
//    double turnAmountR;
//    private enum Module{
//        RIGHT,
//        LEFT
//    }
//
//    //current orientation
//    GlobalPosSystem posSystem;
//    double leftCurrentW; //current wheel orientation
//    double rightCurrentW;
//    double currentR; //current robot header orientation
//
//
//    public Accelerator accelerator;
//    TrackJoystick trackJoystick = new TrackJoystick();
//
//    //PIDs
//    protected SnapSwerveModulePID snapLeftWheelPID;
//    protected SnapSwerveModulePID snapRightWheelPID;
//
//    double[] motorPower = new double[4];
//
//    boolean firstMovement = true;
//
//    public SplineKinematicsTest(GlobalPosSystem posSystem){
//        this.posSystem = posSystem;
//
//        snapLeftWheelPID = new SnapSwerveModulePID();
//        snapRightWheelPID = new SnapSwerveModulePID();
//
//        snapLeftWheelPID=new SnapSwerveModulePID();
//        snapRightWheelPID=new SnapSwerveModulePID();
//        snapLeftWheelPID.setTargets(0.03, 0, 0.01);
//        snapRightWheelPID.setTargets(0.03, 0, 0.01);
//
//        accelerator = new Accelerator();
//    }
//
//    public void logic(){
//        trackJoystick.getCurrentJoystickL();
//        leftCurrentW = posSystem.getLeftWheelW();
//        rightCurrentW = posSystem.getRightWheelW();
//        currentR = posSystem.getPositionArr()[4];
//
//        target = Math.toDegrees(Math.atan2(lx, ly));
//        if (lx == 0 && ly == 0) target = 0;
//        else if (lx==0 && ly < 0) target=180;
//        target=clamp(target);
//        wheelOptimization(target, leftCurrentW, Module.LEFT, false);
//        wheelOptimization(target, rightCurrentW, Module.RIGHT, false);
//
//        if (noMovementRequests()){
//            type=DriveType.STOP;
//            spinPower = 0;
////            leftRotatePower = 0;
//            rightRotatePower = 0;
//
//            spinClicks = 0;
//            rightRotClicks = 0;
////            leftRotClicks = 0;
//
//            translatePerc = 0;
//            rotatePerc = 0;
//
//        } else if (shouldSpline(Math.max(turnAmountL, turnAmountR))) {
//            type=DriveType.SPLINE;
//
//            spinPower = Math.sqrt(Math.pow(lx,2) + Math.pow(ly, 2));
//            if ((rightCurrentW + leftCurrentW)/2.0 >= constants.degreeTOLERANCE){
//                double throttle = 1;
//                double t = lx / 2.0;
//                if (lx == 0) throttle = 1;
//                else {
//                    throttle = (lx > ly ? Math.abs(ly/lx) : Math.abs(lx/ly));
//                    throttle = Math.abs((throttle - t)/(throttle + t));
//                }
//
//                int direction = (leftTurnDirectionW == 1 ? 1 : -1);
//                rightThrottle = (direction == 1 ? throttle : 1);
//                leftThrottle = (direction == 1 ? 1 : throttle);
//            } else{
//                leftThrottle = 1;
//                rightThrottle = 1;
//            }
//        } else if(shouldSnap()){
//            type=DriveType.SNAP;
//            wheelOptimization(target, leftCurrentW, Module.LEFT, true);
//            wheelOptimization(target, rightCurrentW, Module.RIGHT, true);
//
//            spinPower = 0;
//            leftRotatePower = snapLeftWheelPID.update(turnAmountL * leftTurnDirectionW); //turnAmountL approaches 0, which is why we set our PID target to 0.
//            rightRotatePower = snapRightWheelPID.update(turnAmountR * rightTurnDirectionW);
//
//            spinClicks = 0;
//            rightRotClicks = (int)(turnAmountR * constants.CLICKS_PER_DEGREE) * rightTurnDirectionW;
//            leftRotClicks = (int)(turnAmountL * constants.CLICKS_PER_DEGREE) * leftTurnDirectionW;
//            translatePerc=0;
//            rotatePerc=1;
//        } else{
//            type=DriveType.LINEAR;
//
//            if (spinDirectionL != spinDirectionR){
//                //then something is wrong
//                spinDirectionL = spinDirectionR;
//            }
//            spinPower = Math.sqrt(Math.pow(lx,2) + Math.pow(ly, 2));
//            spinClicks = (int)(spinPower * 100) * spinDirectionR;
//            leftRotClicks = 0;
//            rightRotClicks = 0;
//
//            translatePerc = 1;
//            rotatePerc = 0;
//        }
//    }
//
//    public void wheelOptimization(double target, double currentW, Module module, boolean after){ //returns how much the wheels should rotate in which direction
//        double turnAmount = target - currentW;
//        int turnDirection = (int)Math.signum(turnAmount);
//        spinDirectionR = 1;
//        spinDirectionL = 1;
//
//        if(Math.abs(turnAmount) > 180){
//            turnAmount = 360 - Math.abs(turnAmount);
//            turnDirection *= -1;
//        }
//
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
//
//        switch (module){
//            case RIGHT:
//                rightTurnDirectionW = turnDirection;
//                turnAmountR = Math.abs(turnAmount);
//                break;
//
//            case LEFT:
//                leftTurnDirectionW = turnDirection;
//                turnAmountL =  Math.abs(turnAmount);
//                break;
//        }
//    }
//
//    public boolean shouldSnap(){
//        return (Math.abs(turnAmountL) >= constants.degreeTOLERANCE || Math.abs(turnAmountR) >= constants.degreeTOLERANCE);
//    }
//
//    public boolean shouldSpline(double turnAmount){
//        if (Math.abs(turnAmount) <= 45 && !firstMovement){
//            if (trackJoystick.getCurrentJoystickL() != trackJoystick.getPrevJoystickL()){ //if the joystick has actually moved
//                return true;
//            } else{ //the joystick has not moved
//                 return (type==DriveType.SPLINE); //if doing x and joystick has not moved, keep doing x.
//            }
//        }
//        return false;
//    }
//
//    public double clamp(double degrees){
//        if (Math.abs(degrees) >= 360) degrees %= 360;
//
//        if (degrees < -179 || degrees > 180) {
//            int modulo = (int)Math.signum(degrees) * -180;
//            degrees = Math.floorMod((int)degrees, modulo);
//        }
//        return degrees;
//    }
//
//    public void getGamepad(double lx, double ly, double rx, double ry){
//        this.lx = lx;
//        this.ly = ly;
//        this.rx = rx;
//        this.ry = ry;
//    }
//
//    public double[] getPower(){
//
//        motorPower[0] = spinPower * translatePerc + leftRotatePower * rotatePerc; //top left
//        motorPower[1] = spinPower * translatePerc + leftRotatePower * rotatePerc; //bottom left
//        motorPower[2] = spinPower * translatePerc + rightRotatePower * rotatePerc; //top right
//        motorPower[3] = spinPower * translatePerc + rightRotatePower * rotatePerc; //bottom right
//
//        motorPower[0] = accelerator.update(motorPower[0], type) * leftThrottle;
//        motorPower[1] = accelerator.update(motorPower[1], type) * leftThrottle;
//        motorPower[2] = accelerator.update(motorPower[2], type) * rightThrottle;
//        motorPower[3] = accelerator.update(motorPower[3], type) * rightThrottle;
//
//        return motorPower;
//    }
//
//
//    public int[] getClicks(){
//        int[] clicks = new int[4];
//        clicks[0] = spinClicks + leftRotClicks; //left
//        clicks[1] = -spinClicks + leftRotClicks; //left
//        clicks[2] = spinClicks + rightRotClicks; //right
//        clicks[3] = -spinClicks  + rightRotClicks; //right
//        return clicks;
//    }
//
//    public DriveType getDriveType(){
//        return type;
//    }
//
//    public boolean noMovementRequests(){
//        if (lx==0 && ly==0 && rx==0 && ry==0) {
//            firstMovement = true;
//            return true;
//        }
//        return false;
//    }
//
//    public double getTarget(){
//        return target;
//    }
//    public int getRightDirectionW(){
//        return rightTurnDirectionW;
//    }
//    public int getLeftDirectionW(){
//        return leftTurnDirectionW;
//    }
//    public double getLTurnAmount(){
//        return turnAmountL;
//    }
//    public double getRTurnAmount(){
//        return turnAmountR;
//    }
//}
//
//
//
