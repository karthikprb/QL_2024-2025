package org.firstinspires.org.firstinspires.ftc.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.org.firstinspires.ftc.Wrapper.Caching_Servo;
import org.firstinspires.org.firstinspires.ftc.Wrapper.GamepadEx;

@TeleOp
public class Servo_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = ServoTester.servoName;
    final String name2 = ServoTester.servoTwoName;
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

      //  servo.setZeros(0.27, 0.95);
      //  servo.setZeros(0.06, 0.74);

        waitForStart();
        while (opModeIsActive()) {

            servo.setPosition(ServoTester.pos);
            servo2.setPosition(ServoTester.pos2);



           // manualSetPosition(ServoTester.pos);

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

    public static String servoName = "wrist";

    public static String servoTwoName = "wrist1";
}