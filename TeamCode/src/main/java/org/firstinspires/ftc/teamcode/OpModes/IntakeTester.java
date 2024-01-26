package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "IntakeTester")
public class IntakeTester extends LinearOpMode {
    Intake intake;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap, telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);



        waitForStart();

        while (opModeIsActive()) {

            intake.intake(gamepadEx1, gamepadEx2, telemetry);
            //intake.intake_dropper.setPosition(IntakeT.pos);

            gamepadEx1.loop();
            gamepadEx2.loop();
            intake.write();
        }
    }
}


@Config
class IntakeT {
    //Set the set/start position of the servo in dashboard
    public static double pos = 0;
}
