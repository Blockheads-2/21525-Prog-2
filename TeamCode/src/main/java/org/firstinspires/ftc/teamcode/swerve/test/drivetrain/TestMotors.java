package org.firstinspires.ftc.teamcode.swerve.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.RevisedKinematics;

@TeleOp(name="Test Motors", group="Drive")
//@Disabled
public class TestMotors extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;
    Constants constants = new Constants();
    private double[] posData = new double[4];

    private ElapsedTime runtime = new ElapsedTime();
    int distanceClicks;
    int rotClicks;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);
        kinematics = new RevisedKinematics(posSystem);
        posSystem.grabKinematics(kinematics);
        kinematics.leftThrottle = -1;
        kinematics.rightThrottle = 1;
        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        double distance = constants.WHEEL_CIRCUMFERENCE;
        double rot = 0;
        distanceClicks = (int)(distance * constants.CLICKS_PER_INCH); //rotation clicks
        rotClicks = (int)(rot * constants.CLICKS_PER_DEGREE);

        drive(distanceClicks, rotClicks);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        //  robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() { //When "start" is pressed
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        DriveTrainPowerEncoder();
    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){
        telemetry.addData("Xpos", posSystem.getPositionArr()[0]);
        telemetry.addData("Ypos", posSystem.getPositionArr()[1]);
        telemetry.addData("left W", posSystem.getPositionArr()[2]);
        telemetry.addData("right W", posSystem.getPositionArr()[3]);
        telemetry.addData("R", posSystem.getPositionArr()[4]);
        telemetry.addData("Target Rot", rotClicks);
        telemetry.addData("Target Distance", distanceClicks);

        telemetry.addData("TopL Clicks", robot.topL.getCurrentPosition());
        telemetry.addData("BotL Clicks", robot.botL.getCurrentPosition());
        telemetry.addData("TopR Clicks", robot.topR.getCurrentPosition());
        telemetry.addData("BotR Clicks", robot.botR.getCurrentPosition());

        telemetry.addData("TopL Mode", robot.topL.getMode());
        telemetry.addData("BotL Mode", robot.botL.getMode());
        telemetry.addData("TopR Mode", robot.topR.getMode());
        telemetry.addData("BotR Mode", robot.botR.getMode());

        telemetry.update();
    }


    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }


    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();

        robot.botL.setPower(0.5);
        robot.topL.setPower(0.5);
        robot.botR.setPower(0.5);
        robot.topR.setPower(0.5);
    }

    public void drive(int distanceClicks, int rotClicks){
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - (distanceClicks * kinematics.leftThrottle) + rotClicks);
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + (distanceClicks * kinematics.leftThrottle) + rotClicks);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() - (distanceClicks * kinematics.rightThrottle) + rotClicks);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + (distanceClicks * kinematics.rightThrottle) + rotClicks);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}