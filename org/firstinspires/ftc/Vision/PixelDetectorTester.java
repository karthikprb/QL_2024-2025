package org.firstinspires.org.firstinspires.ftc.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.org.firstinspires.ftc.Components.Robot;

@Autonomous
public class PixelDetectorTester extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        robot.redCloseInitializeWebcam();
    }

    @Override
    public void loop() {

        //telemetry.addData("Duck Position", robot.getPixelCase());

    }
}
