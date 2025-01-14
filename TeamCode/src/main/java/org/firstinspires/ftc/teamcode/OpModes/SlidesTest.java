package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;


@TeleOp
public class SlidesTest extends LinearOpMode {

    Slides slides;
    GamepadEx gamepadEx1;
    Mecanum_Drive drive;
    GamepadEx gamepadEx2;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap,telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        drive = new Mecanum_Drive(hardwareMap,telemetry);

        waitForStart();
        while(opModeIsActive()){
            slides.operate(gamepadEx2);
            slides.write();
            drive.drive(gamepadEx1,1,1);
            gamepadEx1.loop();
            gamepadEx2.loop();
            drive.write();
        }
    }
}
