package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    private VoltageSensor voltageSensor;

    public void initVoltageSensor(){
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    DcMotor motor;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap,telemetry);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);



        waitForStart();

        while (opModeIsActive()) {
           robot.operate(gamepadEx1,gamepadEx2);
           telemetry.update();
        }
    }
}