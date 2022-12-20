package org.firstinspires.ftc.teamcode.common.gps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.Kinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.RevisedKinematics;

import java.util.HashMap;

public class GlobalPosSystem {

    Constants constants = new Constants();
    Kinematics kinematics;
    RevisedKinematics revisedKinematics;

    private double[] positionArr = new double[5];
    public HashMap<String, Integer> motorClicksPose = new HashMap<>();
    public HashMap<String, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    Orientation lastOrientation;
    Orientation currentOrientation;
    double currAngle = 0;

    public GlobalPosSystem(HardwareDrive robot){
        this.robot = robot;

        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
        motorClicksPose.put("botL", robot.botL.getCurrentPosition());

        prevMotorClicks.put("topR", motorClicksPose.get("topR"));
        prevMotorClicks.put("botR", motorClicksPose.get("botR"));
        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
        prevMotorClicks.put("botL", motorClicksPose.get("botL"));

        currentOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void grabKinematics(Kinematics k){
        kinematics = k;
    }

    public void grabKinematics(RevisedKinematics k){
        revisedKinematics = k;
    }

    public void calculatePos(){
        updateHash();
        calculateWheel();
        calculateHeader();
        calculateRobot();
    }

    public void calculateWheel(){
        double rotateL = (robot.topL.getCurrentPosition() + robot.botL.getCurrentPosition()) / 2.0; //total rotation of left module
        rotateL *= constants.DEGREES_PER_CLICK;
        double rotateR = (robot.topR.getCurrentPosition() + robot.botR.getCurrentPosition()) / 2.0; //total rotation of right module
        rotateR *= constants.DEGREES_PER_CLICK;

        positionArr[2] = clamp(rotateL);
        positionArr[3] = clamp(rotateR);
    }

    public void calculateHeader(){ //add this after implementing the extension hub onto the robot
        // Get current orientation
        currentOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = currentOrientation.firstAngle - lastOrientation.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle <= -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle -= deltaAngle; //we subtract instead of add because we subtract the c
        lastOrientation = currentOrientation;
        positionArr[4] = currAngle;
    }

    public void calculateRobot(){
        //right
        int topR = motorClicksPose.get("topR") - prevMotorClicks.get("topR"); //change in top right
        int botR = motorClicksPose.get("botR") - prevMotorClicks.get("botR"); //change in bottom right
        double translationalInchesR = (topR - botR) / 2.0;
        translationalInchesR *= constants.INCHES_PER_CLICK;
        translationalInchesR *= Math.signum(revisedKinematics.rightThrottle);
        double currentAngleR = positionArr[3];

        //left
        int topL = motorClicksPose.get("topL") - prevMotorClicks.get("topL"); //change in top left
        int botL = motorClicksPose.get("botL") - prevMotorClicks.get("botL"); //change in bottom left
        double translationalInchesL = (topL - botL) / 2.0;
        translationalInchesL *= constants.INCHES_PER_CLICK;
        translationalInchesL *= Math.signum(revisedKinematics.leftThrottle);
        double currentAngleL = positionArr[2];

        double splineOrientation = 0.0;
        double baseAngle = (currentAngleL + currentAngleR) / 2.0;
        baseAngle = Math.toRadians(baseAngle);
        double hypotenuse = (translationalInchesL + translationalInchesR) / 2.0;

//        if (revisedKinematics.getDriveType() == RevisedKinematics.DriveType.SPLINE){
//            double bigArc = Math.max(translationalInchesL, translationalInchesR); //unit: inches
//            double smallArc = Math.min(translationalInchesL, translationalInchesR); //unit: inches
//            if (Math.abs(bigArc - smallArc) <= 0.1){
//                double radius = ((bigArc + smallArc) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) / (bigArc - smallArc); //unit: inches
//                double theta = (bigArc - smallArc) / (2 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER); //unit: radians
//                hypotenuse = Math.sqrt((2 * radius * radius) * (1 - Math.cos(theta)));
//                splineOrientation = Math.toDegrees(theta);
//                baseAngle = (Math.PI - theta) / 2.0; //unit: radians
//                baseAngle = (Math.PI / 2.0) - baseAngle;
//            } //problem: this assumes that the modules are parallel.
//        } else if (revisedKinematics.getDriveType() == RevisedKinematics.DriveType.TURN){
//            positionArr[2] += positionArr[4];
//            positionArr[3] += positionArr[4];
//        }


        update(hypotenuse * Math.sin(baseAngle), hypotenuse * Math.cos(baseAngle), 0, 0, 0);
    }

    public void update ( double x, double y, double leftWheelW, double rightWheelW, double robotR){
        //update
        positionArr[0] += (x);
        positionArr[1] += (y);
        positionArr[2] += leftWheelW;
        positionArr[3] += rightWheelW;
        positionArr[4] += robotR;

        positionArr[2] = clamp(positionArr[2]);
        positionArr[3] = clamp(positionArr[3]);
        positionArr[4] = clamp(positionArr[4]);
    }

    public double getLeftWheelW(){
        return positionArr[2];
    }

    public double getRightWheelW(){
        return positionArr[3];
    }

    public void hardResetGPS(){
//        positionArr[2]=positionArr[4];
//        positionArr[3]=positionArr[4];
        positionArr[2]=0;
        positionArr[3]=0;

        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
        motorClicksPose.put("botL", robot.botL.getCurrentPosition());

        prevMotorClicks.put("topR", motorClicksPose.get("topR"));
        prevMotorClicks.put("botR", motorClicksPose.get("botR"));
        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
        prevMotorClicks.put("botL", motorClicksPose.get("botL"));
    }

    public void updateHash(){
        prevMotorClicks.put("topR", motorClicksPose.get("topR"));
        prevMotorClicks.put("botR", motorClicksPose.get("botR"));
        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
        prevMotorClicks.put("botL", motorClicksPose.get("botL"));

        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
        motorClicksPose.put("botL", robot.botL.getCurrentPosition());
    }

//    private boolean goodGap(){
//        return (Math.abs(robot.topR.getCurrentPosition() - prevMotorClicks.get("topR")) > constants.clickTOLERANCE || Math.abs(robot.botR.getCurrentPosition() - prevMotorClicks.get("botR")) > constants.clickTOLERANCE);
//    }

    public double[] getPositionArr() {
        return positionArr;
    }

    public int[] getMotorClicks(){
        int[] clicks = new int[4];
        clicks[0] = robot.topL.getCurrentPosition();
        clicks[1] = robot.botL.getCurrentPosition();
        clicks[2] = robot.topR.getCurrentPosition();
        clicks[3] = robot.botR.getCurrentPosition();

        return clicks;
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

    public Kinematics.DriveType getDriveType() {
        return kinematics.getDriveType();
    }

    public HashMap<String, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}