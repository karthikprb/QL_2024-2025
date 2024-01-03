package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Servo_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = "rightgrab";
    final String name2 = "leftgrab";
    Caching_Servo servo;
    Caching_Servo servo2;


    GamepadEx gamepadEx;

    public void manualSetPosition(double val){
        servo.setPosition(1 - val);
        servo2.setPosition(val);
    }
    public void write(){
        servo.write();
        servo2.write();
    }

    @Override
    public void runOpMode() {
        gamepadEx = new GamepadEx(gamepad1);
        servo = new Caching_Servo(hardwareMap, name);
        servo2 =new Caching_Servo(hardwareMap,name2);

        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(ServoTester.pos);
            servo2.setPosition(ServoTester.pos2);

            write();
            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
            gamepadEx.loop();
        }
    }
}


@Config
class ServoTester{
    //Set the set/start position of the servo in dashboard
    public static double pos = 0;
    public static double pos2 = 1;
}