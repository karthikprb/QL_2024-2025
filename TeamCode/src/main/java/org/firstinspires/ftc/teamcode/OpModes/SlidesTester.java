package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "Intake_Tester")
public class SlidesTester extends LinearOpMode {
    Slides slides;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    @Override
    public void runOpMode() {
        slides = new Slides(hardwareMap, telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);



        waitForStart();

        while (opModeIsActive()) {

            slides.operate(gamepadEx1,gamepadEx2);
            slides.write();

            gamepadEx1.loop();
            gamepadEx2.loop();
        }
    }
}

@Config
class SlidesT{
    //Set the set/start position of the servo in dashboard
    public static double pos = .01;
}