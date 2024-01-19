package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Airplane {
    Caching_Servo plane;
    final String name = "plane";
    private double holdPos = 0;
    private double releasePos = 0;
    private boolean bool = true;

    public Airplane(HardwareMap map, Telemetry telemetry){
        plane = new Caching_Servo(map,name);

    }
    public void release(){
        plane.setPosition(releasePos);
    }
    public void hold(){
        plane.setPosition(holdPos);
    }
    public void launch(GamepadEx gamepad2Ex){
        if(gamepad2Ex.isPress(GamepadEx.Control.left_bumper)){
            bool = false;
        }
        if(bool){
            hold();
        }else{
            release();
        }
    }
    public void write(){
        plane.write();
    }
}
