package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Linkage;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Linkage_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
    Linkage linkage;

    GamepadEx gamepadEx1;


    @Override
    public void runOpMode() {
        linkage = new Linkage(hardwareMap, telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            linkage.operate(gamepadEx1, telemetry);

            // manualSetPosition(ServoTester.pos);

            linkage.write();
            telemetry.update();

        }
    }
}

