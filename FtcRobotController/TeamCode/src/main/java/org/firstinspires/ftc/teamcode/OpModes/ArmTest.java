package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.Linkage;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class ArmTest extends LinearOpMode{
    Arm arm;
    Telemetry telemetry;
    GamepadEx gamepadEx1;
    @Override
    public void runOpMode() {
        arm = new Arm(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            arm.operate(gamepadEx1);
            arm.write();
            gamepadEx1.loop();
        }
    }
}