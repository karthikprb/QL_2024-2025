package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Linkage {

    Caching_Servo servo;
    ElapsedTime time;
    double pos = 0;
    double out = 0;

    double in = 1;

    Telemetry telemetry;

    public Linkage(HardwareMap map, Telemetry telemetry){
        servo = new Caching_Servo(map, "linkage");

        this.telemetry = telemetry;
        time = new ElapsedTime();
        write();
    }

    public void out(){
        servo.setPosition(out);
    }

    public void in(){
        servo.setPosition(in);
    }

    public void operate(GamepadEx gamepadEx1,GamepadEx gamepadEx2, Telemetry telemetry) {

        servo.setPosition(gamepadEx2.gamepad.left_stick_y*.5);

        telemetry.addData("Servo Pos", servo.getPosition());
    }

    public void write() {
        servo.write();
    }
}
