package org.firstinspires.ftc.teamcode.mecanum.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

public class HardwareDrive
{
    //Motors
    public DcMotorEx  lf   = null;
    public DcMotorEx  rf   = null;
    public DcMotorEx  lb   = null;
    public DcMotorEx  rb   = null;
    public DcMotorEx  abl;
    public DcMotorEx abr;
    public DcMotorEx at;

    //Servos
    public Servo armServo;
    public Servo claw;

    //Sensor
    public BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    Constants constants = new Constants();

    /* Constructor */
    public HardwareDrive(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lf = hwMap.get(DcMotorEx.class, "left_front");
        rf = hwMap.get(DcMotorEx.class, "right_front");
        lb = hwMap.get(DcMotorEx.class, "left_back");
        rb = hwMap.get(DcMotorEx.class, "right_back");
        at = hwMap.get(DcMotorEx.class, "arm_top");
        abl = hwMap.get(DcMotorEx.class, "arm_base_left");
        abr = hwMap.get(DcMotorEx.class, "arm_base_right");

        claw = hwMap.get(Servo.class, "claw");
        armServo = hwMap.get(Servo.class, "arm_servo");

        //IMU initiation
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //Reverse Drivetrain Motors
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);

        //Reverse Arm Motors
        at.setDirection(DcMotorEx.Direction.FORWARD);
        abl.setDirection(DcMotorEx.Direction.FORWARD);
        abr.setDirection(DcMotorEx.Direction.REVERSE);

        // Set all motors to zero power
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        at.setPower(0);
        abl.setPower(0);
        abr.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        at.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        abl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        abr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
