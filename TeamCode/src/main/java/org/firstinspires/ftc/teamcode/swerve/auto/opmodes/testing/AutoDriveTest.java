package org.firstinspires.ftc.teamcode.swerve.auto.opmodes.testing;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

@Autonomous (name = "Auto Drive Test", group = "Drive")
public class AutoDriveTest extends LinearOpMode {

    LinearOpMode linearOpMode;
    HardwareDrive robot = new HardwareDrive();
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;

    View relativeLayout;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);

        robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        Movement("Linear", 10,10,0.5);
    }

    private void Movement(String movementType, double x, double y, double power){
        switch (movementType){
            case "Linear":
                //cpde
                double targetAngle = Math.atan2(x, y);
                double targetDistance = Math.sqrt((x*x)+(y*y));

                int topLTargetRotation = (int) (targetAngle * constants.CLICKS_PER_DEGREE);
                int botLTargetRotation = (int) (targetAngle * constants.CLICKS_PER_DEGREE);
                int topRTargetRotation = (int) (targetAngle * constants.CLICKS_PER_DEGREE);
                int botRTargetRotation = (int) (targetAngle * constants.CLICKS_PER_DEGREE);

                robot.topL.setTargetPosition(topLTargetRotation);
                robot.botL.setTargetPosition(botLTargetRotation);
                robot.topR.setTargetPosition(topRTargetRotation);
                robot.botR.setTargetPosition(botRTargetRotation);

                robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.topL.setPower(power);
                robot.botL.setPower(power);
                robot.topR.setPower(power);
                robot.botR.setPower(power);



                int topLTargetDistance = (int) (targetDistance * constants.CLICKS_PER_INCH);
                int botLTargetDistance = (int) (-targetDistance * constants.CLICKS_PER_INCH);
                int topRTargetDistance = (int) (targetDistance * constants.CLICKS_PER_INCH);
                int botRTargetDistance = (int) (-targetDistance * constants.CLICKS_PER_INCH);

                robot.topL.setTargetPosition(topLTargetDistance);
                robot.botL.setTargetPosition(botLTargetDistance);
                robot.topR.setTargetPosition(topRTargetDistance);
                robot.botR.setTargetPosition(botRTargetDistance);

                robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.topL.setPower(power);
                robot.botL.setPower(power);
                robot.topR.setPower(power);
                robot.botR.setPower(power);






                break;
            case "LinearTurn":
                //codeeee
                break;
        }

    }
}
