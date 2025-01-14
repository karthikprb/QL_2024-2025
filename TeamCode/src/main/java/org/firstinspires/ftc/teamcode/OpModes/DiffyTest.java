package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Diffy;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class DiffyTest extends LinearOpMode {

    Diffy diffy;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    @Override
    public void runOpMode() throws InterruptedException {

        diffy = new Diffy(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        waitForStart();

        while (opModeIsActive()){

            diffy.operate(gamepadEx1,gamepadEx2);
            diffy.write();
            gamepadEx1.loop();
            gamepadEx2.loop();

        }
    }
}
