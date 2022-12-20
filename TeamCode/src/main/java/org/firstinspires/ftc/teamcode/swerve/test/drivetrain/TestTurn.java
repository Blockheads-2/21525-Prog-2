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

@TeleOp(name="Test Turn", group="Drive")
//@Disabled
public class TestTurn extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Constants constants = new Constants();

    private ElapsedTime runtime = new ElapsedTime();
    int distanceClicks;

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

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        double distance = -(Math.PI/2) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;
        distanceClicks = (int)(distance * constants.CLICKS_PER_INCH); //rotation clicks
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() { //When "start" is pressed
        drive(distanceClicks);
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
        telemetry.addData("Target Distance", distanceClicks);

        telemetry.addData("TopL Clicks", robot.topL.getCurrentPosition());
        telemetry.addData("BotL Clicks", robot.botL.getCurrentPosition());
        telemetry.addData("TopR Clicks", robot.topR.getCurrentPosition());
        telemetry.addData("BotR Clicks", robot.botR.getCurrentPosition());
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

    public void drive(int distanceClicks){
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + distanceClicks);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - distanceClicks);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() - distanceClicks);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + distanceClicks);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}