package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.kinematics.drive.SimplifiedKinematics;

public class Accelerator {
    ElapsedTime accelerationTimer;
    private boolean isAccelerateCycle = false;

    SimplifiedKinematics.DriveType currentDriveType = SimplifiedKinematics.DriveType.NOT_INITIALIZED;
    SimplifiedKinematics.DriveType prevDriveType = SimplifiedKinematics.DriveType.NOT_INITIALIZED;

    public Accelerator(){
        accelerationTimer = new ElapsedTime();
    }


    public double update(double power){
        if (power == 0) {
            accelerationTimer.reset();
            return 0.0;
        }

        double accelerationFactor = (Math.pow(accelerationTimer.seconds(), 0.5)/ 2) + 0.4;
        power *= accelerationFactor;

        if (power > 1) power = 1;
        else if (power < -1) power = -1;

        return power;
    }


    public double update(double power, SimplifiedKinematics.DriveType dType){
        prevDriveType = currentDriveType;
        currentDriveType = dType;

        if (power == 0) {
            isAccelerateCycle = false;
            accelerationTimer.reset();
            return 0.0;
        }

        if (prevDriveType != currentDriveType){
            accelerationTimer.reset();
            isAccelerateCycle = true;
        }

        double accelerationFactor = (Math.pow(accelerationTimer.seconds(), 3)/3.0) + 0.1;
        power *= accelerationFactor;

        if (power > 1) power = 1;
        else if (power < -1) power = -1;

        return power;
    }

//    public double update(double power){
//        if (power == 0) {
//            isAccelerateCycle = false;
//            accelerationTimer.reset();
//            return 0.0;
//        }
//
//        if (!isAccelerateCycle){
//            accelerationTimer.reset();
//            isAccelerateCycle = true;
//        }
//
//        accelerationFactor = (Math.pow(accelerationTimer.seconds(), 3)/3.0) + 0.1;
//        power *= accelerationFactor;
//
//        if (power > 1) power = 1;
//        else if (power < -1) power = -1;
//
//        return power;
//    }
}
