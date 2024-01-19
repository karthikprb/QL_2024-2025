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
    final String name = "rightarm";
    final String name2 = "leftarm";
    Caching_Servo servo;
    Caching_Servo servo2;


    GamepadEx gamepadEx;

    public void manualSetPosition(double val){
        servo.setPosition(val);
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

        servo.setZeros(.27,.95);
        //right
        servo2.setZeros(.06,.74);
        //left

        waitForStart();
        while (opModeIsActive()) {
            manualSetPosition(ServoTester.pos);

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
}