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
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHubJR;

@Autonomous (name = "Auto Test", group = "Drive")
public class AutoTest extends LinearOpMode {

    HardwareDrive robot = new HardwareDrive();
    HardwareMap hardwareMap;
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    AutoHubJR dispatch;

    View relativeLayout;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);
        dispatch = new AutoHubJR(this);

        robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        dispatch.Move(AutoHubJR.DriveType.LINEAR, 10, 10, 0, 1);
    }
}
