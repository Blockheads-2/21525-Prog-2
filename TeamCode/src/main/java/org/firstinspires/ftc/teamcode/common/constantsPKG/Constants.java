package org.firstinspires.ftc.teamcode.common.constantsPKG;

public class Constants {
    public double LOOP_ITERATION_TIME = 0.025; //must test later on

    //Drive Train Constants
    public double LOAD_ON = 0.6; //assumption
    public double RPM = 1150 * LOAD_ON; //690.  Not very accurate so don't rely on this number.
    public double RPS = RPM / 60.0; //11.5 ish motor revolutions per second, with load
    public double POWER_LIMITER = 0.9;

    public double CLICKS_PER_BLUE_REV = 537.7; //clicks per rev of motors
    public double WHEEL_DIAMETER = 92 / 25.4; //3.622 inches
    public double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; //11.378 inches
    public double MAX_VELOCITY_DT = 2700; // unit is clicks/sec
    public double clickTOLERANCE = 10; //number of clicks or degrees the robot can be off by
    public double degreeTOLERANCE = 5;

    public double MAX_VELOCITY;

    //Arm Constants
    public double CLICKS_PER_BASE_REV = 5281.1; //clicks per rev of base arm motor
    public double CLICKS_PER_TOP_REV =  1120.0; //clicks per rev of top arm motor
    public double DEGS_PER_BASE_CLICK = CLICKS_PER_BASE_REV / 360; //degrees per base motor click
    public double DEGS_PER_TOP_CLICK = CLICKS_PER_TOP_REV / 360; //degrees per top motor click
    public double RATIO_CLICKS = CLICKS_PER_BASE_REV / CLICKS_PER_TOP_REV; //ratio of clicks between rev clicks to gobilda clicks
    public double ARM_BASE_RADIUS = 440; //radius of first stage of arm(base powered by 5204 motor) (mm)
    public double ARM_TOP_RADIUS = 400; //radius of top stage of arm(powered by rev motor) (mm)
    public double ARM_TOP_GEAR_REDUCTION = 16.0 / 40; //motor is geared down for more torque(40% increase of torque)
    public double DEGS_PER_TOP_RADIUS_CLICK = DEGS_PER_TOP_CLICK * ARM_TOP_GEAR_REDUCTION;

    //Claw Constants
    public double INITIALIZED_ARM_SERVO = 0;
    public double CLAW_POSITION_TO_DEGREES = 360;
    public double INITIALIZED_CLAW = 0.2;
    public double CLAW_POSITION = 0.5;
    public double CLAW_RADIUS = 134.359; //mm

    //Swerve constants
        //module translation
    public double BLUE_REV_PER_GREEN = 17.0 / 16.0; //assuming other gear isn't messing stuff up, 17 revolutions of the input shaft = 16 revolutions of the wheel
    public double CLICKS_PER_INCH = (BLUE_REV_PER_GREEN * CLICKS_PER_BLUE_REV * (1.0 / WHEEL_CIRCUMFERENCE)) * (2.0/3.0); //~13.549 clicks per inch
    public double INCHES_PER_CLICK = 1.0 / CLICKS_PER_INCH; //~0.029 inches per click

    //module rotation
    public double BLUE_REVS_PER_PURPLE = 85.0 / 24.0; //~3.54 Blue revs per 1 Purple rev
    public double CLICKS_PER_PURPLE_REV = BLUE_REVS_PER_PURPLE * CLICKS_PER_BLUE_REV;
    public double CLICKS_PER_DEGREE = BLUE_REVS_PER_PURPLE * CLICKS_PER_BLUE_REV * (1/360.0); //1.427 clicks per degree
    public double DEGREES_PER_CLICK = 1.0 / CLICKS_PER_DEGREE;

    public double DEGREES_PER_INCH = CLICKS_PER_INCH * DEGREES_PER_CLICK;
    public double INCHES_PER_DEGREE = 1.0 / DEGREES_PER_INCH;

    public double tableSpinRotPercAllocation = 0.5;
    public double tableSpinSpinPercAllocation = 0.5;


    //Arm Constants!!
    //Bottom
    public int bottomMotorBottom = 1175;
    public int topMotorBottom = 60;
    public double armServoBottom = 0.05;

    //Low
    public int bottomMotorLow = 140;
    public int topMotorLow = 120;
    public double armServoLow = 0.4;

    //Mid
    public int bottomMotorMid = 420;
    public int topMotorMid = 340;
    public double armServoMid = 0.4;

    //High
    public int bottomMotorHigh = 450;
    public int topMotorHigh = 450;
    public double armServoHigh = 0.7;

    //Claw Position
    public double clawUp = 0.5;
    public double clawDown = 0.8;
    public double openClaw = 0;
    public double closeClaw = 1;



    //Distance Between swerve module and Center
    public double MM_INCH = 1/2.56;
    public double DISTANCE_BETWEEN_MODULE_AND_CENTER = 3.406; //3.405512
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;
}
