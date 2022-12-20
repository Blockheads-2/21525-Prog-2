package org.firstinspires.ftc.teamcode.swerve.auto.opmodes;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.kinematics.drive.Kinematics;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.LinearMath;
import org.firstinspires.ftc.teamcode.swerve.auto.Math.SplineMath;

public class AutoHub {
    LinearOpMode linearOpMode;
    HardwareDrive robot;
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;

    LinearMath linearMath = new LinearMath();
    SplineMath splineMath = new SplineMath();

    View relativeLayout;

    public enum DriveType{
        LINEAR,
        SPLINE,
        TURN_ON_CENTER,
    }

    public AutoHub(LinearOpMode plinear){
        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;
        robot = new HardwareDrive();
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        posSystem = new GlobalPosSystem(robot);

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
    void Move(DriveType movementType, double x, double y, double finalAngle, double speed){
        posSystem.calculatePos();

        double power;
        double timeOut;

        int topLEncoderTarget = robot.topL.getCurrentPosition();
        int botLEncoderTarget = robot.botL.getCurrentPosition();
        int topREncoderTarget = robot.topR.getCurrentPosition();
        int botREncoderTarget = robot.botR.getCurrentPosition();

        switch(movementType){
            case LINEAR:
                linearMath.setInits(posSystem.getPositionArr()[0], posSystem.getPositionArr()[1]);
                linearMath.setPos(x, y, finalAngle);
                int[] linearClicks = linearMath.getClicks();
                topLEncoderTarget += linearClicks[0];
                botLEncoderTarget += linearClicks[1];
                topREncoderTarget += linearClicks[2];
                botREncoderTarget += linearClicks[3];

                power = linearMath.getSpinPower(x, y);

                timeOut = linearMath.getRunTime(speed); //make sure to set timeOut AFTER calculating required distance, clicks, etc.
                break;

            case SPLINE:
                splineMath.setInits(robot.topR.getCurrentPosition(), robot.topL.getCurrentPosition());
                splineMath.setPos(x, y, finalAngle);

                int[] splineClicks = splineMath.getClicks();
                topLEncoderTarget += splineClicks[0];
                botLEncoderTarget += splineClicks[1];
                topREncoderTarget += splineClicks[2];
                botREncoderTarget += splineClicks[3];

//                power = splineMath.power

                timeOut = splineMath.getRunTime(speed);
                break;

            case TURN_ON_CENTER:

                break;
        }

        robot.topL.setTargetPosition(topLEncoderTarget);
        robot.botL.setTargetPosition(botLEncoderTarget);
        robot.topR.setTargetPosition(topREncoderTarget);
        robot.botR.setTargetPosition(botREncoderTarget);

        while (linearOpMode.opModeIsActive()){
            posSystem.calculatePos();
//            robot.topL.setVelocity();
//            robot.botL.setVelocity();
//            robot.topR.setVelocity();
//            robot.botR.setVelocity();
        }

        // Stop all motion;
        robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
