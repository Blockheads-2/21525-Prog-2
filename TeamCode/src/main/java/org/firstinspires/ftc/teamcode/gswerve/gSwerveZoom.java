package org.firstinspires.ftc.teamcode.gswerve;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.RevisedKinematics;

@TeleOp(name="Grady's Swerve", group="Drive")

public class gSwerveZoom extends OpMode {
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    RevisedKinematics kinematics;

    Constants constants = new Constants();
    Reset reset;

    double clickPerRev = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0);
    double wheelGearing = 17.0 / 16.0; // Number of revs for the motor to get one rev of main wheel

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);
        kinematics = new RevisedKinematics(posSystem);
        reset = new Reset(robot, posSystem);

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

    public boolean noMovementRequests(){
        return (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0);
    }
    public void goStraight(){

        robot.topL.setTargetPosition((int)(robot.topL.getCurrentPosition() + (gamepad1.left_stick_y * 30)));
        robot.botL.setTargetPosition((int)(robot.botL.getCurrentPosition() + (gamepad1.left_stick_y * -30))); // set the bottom gear to the inverse of the top
        robot.topR.setTargetPosition((int)(robot.topR.getCurrentPosition() + (gamepad1.left_stick_y * -30)));
        robot.botR.setTargetPosition((int)(robot.botR.getCurrentPosition() + (gamepad1.left_stick_y * 30)));

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.topL.setPower(.8);
        robot.botL.setPower(.8);
        robot.topR.setPower(.8);
        robot.botR.setPower(.8);

    }

    public void loop() { //Loop between "start" and "stop"
        if (!noMovementRequests()){
            goStraight();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
