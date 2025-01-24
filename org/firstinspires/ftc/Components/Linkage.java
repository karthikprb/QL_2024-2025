package org.firstinspires.org.firstinspires.ftc.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.org.firstinspires.ftc.Wrapper.Caching_Servo;
import org.firstinspires.org.firstinspires.ftc.Wrapper.GamepadEx;

public class Linkage {

    Caching_Servo servo;
    ElapsedTime time;
    double pos = 0;
    double out = 0;
    double mid = 0;

    double in = 0;

    double grab = 0;
    double open = 0;

    double armToggle = 0;

    Telemetry telemetry;

    public Linkage(HardwareMap map, Telemetry telemetry){
        servo = new Caching_Servo(map, "leftarm");

        this.telemetry = telemetry;
        time = new ElapsedTime();
        write();
    }

    public void operate(GamepadEx gamepad, Telemetry telemetry) {
        servo.setPosition(pos);

        while(gamepad.gamepad.right_trigger>0.2){
            time.startTime();
            if(time.time()>0.02) {
                pos += 0.01;
                time.reset();
                servo.setPosition(pos);
            }

        }

        while(gamepad.gamepad.left_trigger > 0.2){
            time.startTime();
            if(time.time()>0.02) {
                pos -= 0.01;
                time.reset();
                servo.setPosition(pos);
            }
        }
        telemetry.addData("Servo Pos", servo.getPosition());
    }
    public void write(){
        servo.write();
    }
}
