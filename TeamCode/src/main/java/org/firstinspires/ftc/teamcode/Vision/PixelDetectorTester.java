package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
public class PixelDetectorTester extends LinearOpMode{
    Robot robot;
    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);

        robot.initializeWebcam();

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Pixel Case", robot.getPixelCase());
            robot.update();
            telemetry.update();
        }

        waitForStart();

        robot.stopWebcam();

        while(opModeIsActive()){
            break;
        }
    }

}
