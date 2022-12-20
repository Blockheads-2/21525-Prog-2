package org.firstinspires.ftc.teamcode.mecanum.common.positioning;

public class AbsPose {
    private double absPoseX = 0;
    private double absPoseY = 0;
    private double mu = 0;
    private double nu = 0;

    public void setAbsDistance(double mu, double nu){
        this.mu = mu;
        this.nu = nu;
    }

    public double returnDistanceX(){
        double xi = mu - absPoseX;
        absPoseX += mu;
        return xi;
    }

    public double returnDistanceY(){
        double chi = nu - absPoseY;
        absPoseY += nu;
        return chi;
    }




}
