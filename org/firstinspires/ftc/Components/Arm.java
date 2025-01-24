package org.firstinspires.org.firstinspires.ftc.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.org.firstinspires.ftc.Wrapper.Caching_Servo;
import org.firstinspires.org.firstinspires.ftc.Wrapper.GamepadEx;

public class Arm {

  Caching_Servo grabber;
  Caching_Servo rightGrab;

  Caching_Servo rightArm;
  Caching_Servo leftArm;

  ElapsedTime time;

 public static double out = 0; // 0 is real position
 public static double mid = 0;

 public static double in = 1;

 public static double grab = .3;
 public static double open = 0; // 0 is real pos

 double armToggle = 0;
 double grabberToggle = 0;

    Telemetry telemetry;

    public Arm(HardwareMap map){
        rightArm = new Caching_Servo(map, "rightarm");
        leftArm = new Caching_Servo(map, "leftarm");
        grabber = new Caching_Servo(map,"grabber");

        write();
    }




    public void grab() {
        grabber.setPosition(grab);
    }

    public void open(){
        grabber.setPosition(open);

    }

    public void armMid() {
        leftArm.setPosition(mid);
        rightArm.setPosition(mid);
    }

    public void armIn() {
        leftArm.setPosition(in);
        rightArm.setPosition(in);
    }
    public void armOut() {
        leftArm.setPosition(out);
        rightArm.setPosition(out);
    }



    public void operate(GamepadEx gamepad1) {
        if(gamepad1.isPress(GamepadEx.Control.a)){
            armToggle ++;
            if(armToggle == 1){
                armIn();
            }
            else if(armToggle == 2){
                armOut();
            }else if(armToggle == 3){
                armToggle = 0;
            }

        } if(gamepad1.isPress(GamepadEx.Control.b)){
            grabberToggle++;
            if(grabberToggle == 1){
                grab();
            }else if(grabberToggle == 2){
                open();
            }else if(grabberToggle == 3){
                grabberToggle = 0;
            }
        }
        if(gamepad1.isPress(GamepadEx.Control.right_bumper)){
            armIn();
        }
        if(gamepad1.isPress(GamepadEx.Control.left_bumper)){
            armOut();
        }


    }
    public void write(){
        rightArm.write();
        leftArm.write();
        grabber.write();


    }
}
