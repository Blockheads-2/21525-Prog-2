package org.firstinspires.ftc.teamcode.mecanum.common.positioning;

import org.firstinspires.ftc.teamcode.mecanum.common.Constants;

public class MathConstHead {
    double psi = 0; //final x
    double omega = 0; //final y
    double distance = 0;
    double angle = 0;

    double psiSquared = 0;
    double omegaSquared = 0;
    double psiOverOmega = 0;

    Constants constants = new Constants();
    AbsPose absPose = new AbsPose();


    //Input the Final Position

    /* ----Implement once finish abs pose----
    public void setFinalPose(double xPose, double yPose){
        absPose.setAbsDistance(xPose,yPose);

        psi = absPose.returnDistanceX();
        omega = absPose.returnDistanceY();
    }
     */

    public void setFinalPose(double xPose, double yPose){
        psi = xPose;
        omega = yPose;
    }

    public double returnDistance(){
        double addPoses = Math.pow(psi,2) + Math.pow(omega,2);

        distance = Math.sqrt(addPoses);

        return distance;
    }

    public double returnAngle(){

        angle = Math.atan2(psi,omega);

        return angle;
    }
}
