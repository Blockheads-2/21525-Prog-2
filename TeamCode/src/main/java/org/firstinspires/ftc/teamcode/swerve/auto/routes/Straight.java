 package org.firstinspires.ftc.teamcode.swerve.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.swerve.auto.opmodes.AutoHubJR;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Blue Carousel", group = "Routes")
//@Disabled
public class Straight extends LinearOpMode{

    Constants constants = new Constants();

    AutoHubJR dispatch;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHubJR(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());

        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)
        }

        waitForStart();

        telemetry.update();
    }
}