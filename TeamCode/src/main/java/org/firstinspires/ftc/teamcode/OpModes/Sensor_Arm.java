package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "SensorArm")
public class Sensor_Arm extends LinearOpMode {
    V4B_Arm arm;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    double distanceL = 0;
    double distanceR = 0;
    ColorRangeSensor frontTray;
    ColorRangeSensor backTray;

    @Override
    public void runOpMode() {
        arm = new V4B_Arm(hardwareMap, telemetry);
        frontTray = hardwareMap.get(ColorRangeSensor.class, "trayfront");
        backTray = hardwareMap.get(ColorRangeSensor.class, "trayback");

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);



        waitForStart();

        while (opModeIsActive()) {
            distanceL = ((DistanceSensor)frontTray).getDistance(DistanceUnit.CM);
            distanceR = ((DistanceSensor)backTray).getDistance(DistanceUnit.CM);

            if(distanceL < 1 && distanceR < 1) {
                arm.armOut();
                gamepadEx1.gamepad.rumble(2000);
            } else {
                arm.armMid();
                telemetry.addData("Waiting", "Waiting");
            }

            telemetry.update();
            gamepadEx1.loop();
            gamepadEx2.loop();
            arm.write();
        }
    }
}


