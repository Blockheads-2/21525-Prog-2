package org.firstinspires.ftc.teamcode.mecanum.teleop;

import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanum.common.Constants;
import org.firstinspires.ftc.teamcode.mecanum.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Button;

@TeleOp(name="Single Base Drive", group="Drive")
//@Disabled
public class SingleDriver extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();

    public int abPos = 0, atPos = 0;

    Button bottomButton = new Button();
    Button lowButton = new Button();
    Button midButton = new Button();
    Button highButton = new Button();


    boolean clawClose = false;
    private boolean microAdjustMode = false;

    private int prevPosition = 0;

    private double drivePower = 0.75;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
        prevPosition = robot.abl.getCurrentPosition();

    }

    @Override
    public void init_loop() {
        robot.at.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.at.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.abr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        robot.claw.setPosition(constants.openClaw);
        robot.armServo.setPosition(constants.clawUp);
    }

    @Override
    public void loop() {
        UpdatePlayer1();
        UpdateTelemetry();
        UpdateButton();
    }


    void UpdateButton(){
        bottomButton.update(gamepad1.x);
        lowButton.update(gamepad1.a);
        midButton.update(gamepad1.b);
        highButton.update(gamepad1.y);
    }

    void UpdatePlayer1(){
        DriveTrainBase(drivePower);
        DriveMicroAdjust(0.4);
        ClawControl();
    }



    void UpdateTelemetry(){

        telemetry.addData("LF", robot.lf.getCurrentPosition());
        telemetry.addData("LB", robot.lb.getCurrentPosition());
        telemetry.addData("RF", robot.rf.getCurrentPosition());
        telemetry.addData("RB", robot.rb.getCurrentPosition());

        telemetry.addData("Current Position", robot.at.getCurrentPosition());
        telemetry.addData("Current Bottom Arm # Clicks", robot.abl.getCurrentPosition());

        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    void DriveTrainBase(double drivePower){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn

        robot.lf.setPower((directionY + directionR + directionX) * drivePower);
        robot.rf.setPower((directionY - directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR - directionX) * drivePower);
        robot.rb.setPower((directionY - directionR + directionX) * drivePower);

    }

    void DriveMicroAdjust(double power) {
        if (microAdjustMode){
            if (gamepad1.dpad_up) {
                robot.lf.setPower(power);
                robot.rf.setPower(power);
                robot.lb.setPower(power);
                robot.rb.setPower(power);
            } else if (gamepad1.dpad_down) {
                robot.lf.setPower(-power);
                robot.rf.setPower(-power);
                robot.lb.setPower(-power);
                robot.rb.setPower(-power);
            } else if (gamepad1.dpad_right) {
                robot.lf.setPower(power);
                robot.rf.setPower(-power);
                robot.lb.setPower(-power);
                robot.rb.setPower(power);
            } else if (gamepad1.dpad_left) {
                robot.lf.setPower(-power);
                robot.rf.setPower(power);
                robot.lb.setPower(power);
                robot.rb.setPower(-power);
            }
        }
        else{
            if (gamepad1.dpad_up) {
                robot.at.setTargetPosition(robot.at.getCurrentPosition() + 10);
                robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.at.setPower(power);
            } else if (gamepad1.dpad_down) {
                robot.at.setTargetPosition(robot.at.getCurrentPosition() - 10);
                robot.at.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.at.setPower(power);
            } else if (gamepad1.dpad_right) {
                robot.abl.setTargetPosition(robot.abl.getCurrentPosition() + 50);
                robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.abl.setPower(power);

                robot.abr.setTargetPosition(robot.abr.getCurrentPosition() + 50);
                robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.abr.setPower(power);
            } else if (gamepad1.dpad_left) {
                robot.abl.setTargetPosition(robot.abl.getCurrentPosition() - 50);
                robot.abl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.abl.setPower(power);

                robot.abr.setTargetPosition(robot.abr.getCurrentPosition() - 50);
                robot.abr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.abr.setPower(power);
            }
        }
        if (gamepad1.right_bumper){
            microAdjustMode = !microAdjustMode;
        }
    }


    void ClawControl(){
        if (gamepad1.left_bumper){
            if (clawClose){
                robot.claw.setPosition(0.9);
            }
            else{
                robot.claw.setPosition(0.5);
            }
            clawClose = !clawClose;
        }
        robot.armServo.setPosition(0.7 * gamepad1.right_trigger);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }

}
