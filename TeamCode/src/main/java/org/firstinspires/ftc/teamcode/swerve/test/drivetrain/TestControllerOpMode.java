package org.firstinspires.ftc.teamcode.swerve.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.swerve.teleop.TrackJoystick;

@TeleOp(name="Test Controller", group="Drive")
//@Disabled
public class TestControllerOpMode extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    double target = 0;
    double turnAmount = 0;
    double current = 0;
    boolean firstMovement = false;

    TrackJoystick joystickTracker;
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);
        joystickTracker = new TrackJoystick();

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        DriveTrainBase();
    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){
        telemetry.addData("X gamepad", gamepad1.left_stick_x);
        telemetry.addData("Y gamepad", -gamepad1.left_stick_y);
        telemetry.addData("Target", target);
        telemetry.addData("Current", current);
        telemetry.addData("Turn Amount", turnAmount);
        telemetry.addData("Joystick Change", joystickTracker.getChange());
        telemetry.addData("> 90", joystickTracker.getAbsoluteChange() > 90);
        telemetry.addData("Current Joystick", joystickTracker.getCurrentJoystickL());
        telemetry.addData("Previous Joystick", joystickTracker.getPrevJoystickL());
        telemetry.addData("First movement", firstMovement);
        telemetry.addData("Gap time", joystickTracker.getGapTime());

        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    void DriveTrainBase(){
        if (x.getState() == Button.State.TAP){
            current += 10;
        } else if (y.getState() == Button.State.TAP){
            current -= 10;
        }

        if (a.getState() == Button.State.TAP){
            joystickTracker.changeGapTime(50);
        } else if (b.getState() == Button.State.TAP){
            joystickTracker.changeGapTime(-50);
        }

        if (x.getState() == Button.State.DOUBLE_TAP){
            firstMovement = false;
        }

        current = clamp(current);

        DriveTrainMove();
    }

    private void DriveTrainMove(){
        //outputs of joysticks
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = -gamepad1.left_stick_y; //returns a value between [-1, 1]
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = -gamepad1.right_stick_y; //returns a value between [-1, 1]

        joystickTracker.trackJoystickL(left_stick_x, left_stick_y);

        target = Math.toDegrees(Math.atan2(left_stick_x, left_stick_y));
        if (left_stick_x == 0 && left_stick_y == 0) target = 0;
        else if (left_stick_x==0 && left_stick_y < 0) target=180;

        turnAmount = wheelOptimization(target, current);

        if (joystickTracker.getAbsoluteChange() > 90) firstMovement = true;
    }

    public double wheelOptimization(double target, double currentW){ //returns how much the wheels should rotate in which direction
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (currentW < 0 ? currentW + 360 : currentW);

        double turnAmount = target - currentW;
        double turnAmount2 = target2 - current2;

        if (Math.abs(turnAmount) < Math.abs(turnAmount2)){
            return turnAmount;

        } else{
            return turnAmount2;
        }
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