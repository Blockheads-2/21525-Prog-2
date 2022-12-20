package org.firstinspires.ftc.teamcode.swerve.auto.opmodes;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.AutoKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.RevisedKinematics;

public class AutoHubJR {
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    AutoKinematics kinematics;
    Reset reset;


    View relativeLayout;

    public enum DriveType{
        LINEAR,
        SPLINE,
        TURN_ON_CENTER,
    }

    public AutoHubJR(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        posSystem = new GlobalPosSystem(robot);
        kinematics = new AutoKinematics(posSystem);
        reset = new Reset(robot, posSystem);

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearOpMode.telemetry.addData("Status", "Waiting on Camera");
        linearOpMode.telemetry.update();
    }

    //for now, we'll leave it as last year's template, but it might be a good idea to be constantly calculating its position and
    // how much it needs to go to lessen error.
    public void Move(DriveType movementType, double x, double y, double finalAngle, double speed){

//        double powerTopL;
//        double powerBotL;
//        double powerTopR;
//        double powerBotR;

        posSystem.calculatePos();
        kinematics.setPos(x, y, finalAngle, speed);
        reset.reset(false);

        while (linearOpMode.opModeIsActive() && robot.wheelsAreBusy()){
            posSystem.calculatePos();

            kinematics.setPos(x, y, finalAngle, speed);
            kinematics.logic();

            int[] targetClicks = kinematics.getClicks();
            robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks[0]);
            robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks[1]);
            robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks[2]);
            robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks[3]);

            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double[] motorPower = kinematics.getPower();
            robot.topL.setPower(motorPower[0] * constants.POWER_LIMITER);
            robot.botL.setPower(motorPower[1] * constants.POWER_LIMITER);
            robot.topR.setPower(motorPower[2] * constants.POWER_LIMITER);
            robot.botR.setPower(motorPower[3] * constants.POWER_LIMITER);

//            powerTopL = motorPower[0] * constants.POWER_LIMITER;
//            powerBotL = motorPower[1] * constants.POWER_LIMITER;
//            powerTopR = motorPower[2] * constants.POWER_LIMITER;
//            powerBotR = motorPower[3] * constants.POWER_LIMITER;
        }

//        reset.reset(true);

        while(reset.finishedReset()){
            reset.reset(true);
        }
    }

    public void telemetryData(){

        linearOpMode.telemetry.addData("X pos", posSystem.getPositionArr()[0]);
        linearOpMode.telemetry.addData("Y pos", posSystem.getPositionArr()[1]);
        linearOpMode.telemetry.addData("Left W",  posSystem.getLeftWheelW());
        linearOpMode.telemetry.addData("Right W", posSystem.getRightWheelW());
        linearOpMode.telemetry.addData("R", posSystem.getPositionArr()[4]);

        linearOpMode.telemetry.addData("Spin Direction (Left)", kinematics.leftThrottle);
        linearOpMode.telemetry.addData("Spin Direction (Right)", kinematics.rightThrottle);

        linearOpMode.telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        linearOpMode.telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        linearOpMode.telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        linearOpMode.telemetry.addData("botR clicks", robot.botR.getCurrentPosition());
        linearOpMode.telemetry.addData("Left Rotate Power", kinematics.leftRotatePower);
        linearOpMode.telemetry.addData("Right Rotate Power", kinematics.rightRotatePower);

        linearOpMode.telemetry.addData("Right Clicks target", kinematics.spinClicksR + kinematics.rightRotClicks);

        linearOpMode.telemetry.addData("TopL Spin Power", kinematics.spinPower);
        linearOpMode.telemetry.addData("TopR Spin Power", kinematics.spinPower);
        linearOpMode.telemetry.addData("Drive Type", kinematics.getDriveType());
        linearOpMode.telemetry.addData("First movement", kinematics.firstMovement);

        linearOpMode.telemetry.update();
    }
}
