package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Airplane;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "AirplaneTest")
public class AirPlaneTest extends LinearOpMode {
    Airplane airplane;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    @Override
    public void runOpMode() {
        airplane = new Airplane(hardwareMap, telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);



        waitForStart();

        while (opModeIsActive()) {

            airplane.operate(gamepadEx2, telemetry);

            gamepadEx1.loop();
            gamepadEx2.loop();
            airplane.write();
        }
    }
}


/*
Top = 0.61   - 0.65
2nd = 0.65 - 0.72
3rd =0.72 - 0.78
4th = 0.78 - 0.84
bottom = 0.94
 */


